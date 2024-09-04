#include "Data_fusion.hpp"

FusionNode::FusionNode(ros::NodeHandle& nh)
    : depth_point_sub_(nh, "/camera/depth/points", 1),
      point_cloud_sub_(nh, "/point_cloud", 1),
      sync_(MySyncPolicy(10), depth_point_sub_, point_cloud_sub_)
{
    sync_.registerCallback(boost::bind(&FusionNode::callback, this, _1, _2));
    timer_ = nh.createTimer(ros::Duration(0.1), boost::bind(&FusionNode::timerCallback, this, _1));
    fused_data_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/fused_data", 1);
}



void FusionNode::callback(const sensor_msgs::PointCloud2ConstPtr& depth_point_msg,
                          const sensor_msgs::PointCloud2ConstPtr& point_cloud_msg)
{
    // 数据回调函数，将两个rosmsg转换为pcl::PointCloud<pcl::PointXYZ>
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*point_cloud_msg, *point_cloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr depth_point_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*depth_point_msg, *depth_point_cloud);
    depth_point_cloud_ = depth_point_cloud;
    point_cloud_ = point_cloud;
}



void FusionNode::timerCallback(const ros::TimerEvent&)
{
    // 定时器回调函数
    // 在这里实现深度图像点云和雷达点云数据的融合逻辑
    sensor_msgs::PointCloud2 fused_data;
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud = point_cloud_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr depth_point_cloud = depth_point_cloud_;
    pcl::transformPointCloud(*point_cloud, *point_cloud, camera_to_lidar_transform_);
    pcl::PointCloud<pcl::PointXYZ>::Ptr fused_point_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    *fused_point_cloud = *point_cloud + *depth_point_cloud;

    pcl::toROSMsg(*fused_point_cloud, fused_data);
    fused_data.header.frame_id = "base_link";
    fused_data.header.stamp = ros::Time::now();
    fused_data_pub_.publish(fused_data);
}
