//
// Created by valerian on 13/01/18.
//


#include <pluginlib/class_list_macros.h>
#include <rgb_to_hsv.h>


using namespace sb_pointcloud_processing;

RGBtoHSV::RGBtoHSV(){}

void RGBtoHSV::onInit()
{
    NODELET_DEBUG("Initializing Nodelet...");
    ros::NodeHandle& private_nh = getPrivateNodeHandle();
    sub = private_nh.subscribe("input", 1, &RGBtoHSV::callback, this);
    pub = private_nh.advertise<pcl::PointCloud<pcl::PointXYZHSV> >("output", 1);
    NODELET_DEBUG("Nodelet Initialized");
}

void RGBtoHSV::callback(const sensor_msgs::PointCloud2::ConstPtr &input)
{
    pcl::PCLPointCloud2::Ptr pcl_input(new pcl::PCLPointCloud2);
    pcl_conversions::toPCL (*(input), *(pcl_input));
    pcl::PointCloud<PointXYZRGB>::Ptr pcl_rgb(new pcl::PointCloud<PointXYZRGB>());
    pcl::fromPCLPointCloud2(*pcl_input, *pcl_rgb);
    impl_.setInputCloud(pcl_rgb);
    pcl::PointCloud<PointXYZHSV>::Ptr pcl_output(new pcl::PointCloud<PointXYZHSV>());
    impl_.filter(*pcl_output);
    pub.publish(*pcl_output);
}
    

PLUGINLIB_EXPORT_CLASS(RGBtoHSV, nodelet::Nodelet)
