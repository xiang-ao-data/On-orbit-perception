#!/usr/bin/env python

import rospy
from ICP import pointcloud_registration
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import open3d as o3d
import numpy as np
import copy
from std_srvs.srv import Trigger, TriggerResponse

class PointCloudRegistrationNode:
    def __init__(self):
        self.pcd_list = []
        self.camera_pcd_list = []
        self.lidar_pcd_list = []
        self.adding_points = True
        self.registration = pointcloud_registration()
        rospy.Subscriber('/lidar_points', PointCloud2, self.callback_lidar)
        rospy.Subscriber('/camera/depth/points', PointCloud2, self.callback_camera)
        rospy.Subscriber('/fuse_points', PointCloud2, self.callback)
        self.pub = rospy.Publisher('/registered_points', PointCloud2, queue_size=10)
        self.stop_service = rospy.Service('stop_adding_points', Trigger, self.stop_adding_points)
        self.register_service = rospy.Service('register_points', Trigger, self.register_points)
    
    def callback_lidar(self, msg):
        if self.adding_points:
            points = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(points)
            self.lidar_pcd_list.append(pcd)
            rospy.loginfo("Added Lidar point cloud to list")

    def callback_camera(self, msg):
        if self.adding_points:
            points = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(points)
            self.camera_pcd_list.append(pcd)
            rospy.loginfo("Added Camera point cloud to list")

    def callback(self, msg):
        if self.adding_points:
            points = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(points)
            self.pcd_list.append(pcd)
            rospy.loginfo("Added point cloud to list")

    def stop_adding_points(self, req):
        self.adding_points = False
        return TriggerResponse(success=True, message="Stopped adding points")

    def register_points(self, req):
        if len(self.pcd_list) == 0:
            rospy.loginfo("No fused point clouds to register")
            if len(self.lidar_pcd_list) != 0:
                # If there are only Lidar point clouds, use them for registration
                self.registration.set_input(self.lidar_pcd_list)
            elif len(self.camera_pcd_list) != 0:
                # If there are only Camera point clouds, use them for registration
                self.registration.set_input(self.camera_pcd_list)
            else:
                rospy.loginfo("No point clouds to register")
                return TriggerResponse(success=False, message="No point clouds to register")
        else:
            # If there are fused point clouds, use them for registration
            self.registration.set_input(self.pcd_list)
        registered_pcd = self.registration.run()

        # Convert Open3D point cloud to ROS PointCloud2
        points = np.asarray(registered_pcd.points)
        header = rospy.Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "map"
        pc2_msg = pc2.create_cloud_xyz32(header, points)

        self.pub.publish(pc2_msg)
        o3d.io.write_point_cloud("/path/to/save/registered_pcd.pcd", registered_pcd)

        return TriggerResponse(success=True, message="Point clouds registered and saved")