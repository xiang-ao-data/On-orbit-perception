#ifndef DATA_FUSION_HPP
#define DATA_FUSION_HPP

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
class FusionNode
{
private:
    message_filters::Subscriber<sensor_msgs::PointCloud2> depth_point_sub_;
    message_filters::Subscriber<sensor_msgs::PointCloud2> point_cloud_sub_;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync_;
    ros::Publisher fused_data_pub_;
    sensor_msgs::PointCloud2 fused_data_msg_;
    ros::Timer timer_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr fusion_point_cloud_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr depth_point_cloud_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_;
    tf2::Transform camera_to_lidar_transform_;
public:
    FusionNode(ros::NodeHandle& nh);
    ~FusionNode();

    void callback(const sensor_msgs::ImageConstPtr& depth_image_msg,
                  const sensor_msgs::PointCloud2ConstPtr& point_cloud_msg);
    void timerCallback(const ros::TimerEvent&);
};


FusionNode::~FusionNode()
{
}
