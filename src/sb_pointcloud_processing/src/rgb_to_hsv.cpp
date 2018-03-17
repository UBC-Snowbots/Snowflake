/**
 * created by: valerian ratu
 * Created on: 2018/01/13
 * Description: A ros nodelet which takes pointcloud input in RGB colourspace
 *              and returns it in the HSV colourspace.
 */

#include <pluginlib/class_list_macros.h>
#include <rgb_to_hsv.h>

using namespace sb_pointcloud_processing;
using namespace pcl_ros;
using namespace pcl;

RGBtoHSV::RGBtoHSV() {}

void RGBtoHSV::onInit() {
    NODELET_DEBUG("Initializing Nodelet...");
    ros::NodeHandle& private_nh = getPrivateNodeHandle();
    sub = private_nh.subscribe("input", 1, &RGBtoHSV::callback, this);
    pub = private_nh.advertise<pcl::PointCloud<pcl::PointXYZHSV>>("output", 1);
    NODELET_DEBUG("Nodelet Initialized");
}

void RGBtoHSV::callback(const sensor_msgs::PointCloud2::ConstPtr& input) {
    // Obtain the ROS pointcloud and convert into PCL Pointcloud2
    pcl::PCLPointCloud2::Ptr pcl_input(new pcl::PCLPointCloud2);
    pcl_conversions::toPCL(*(input), *(pcl_input));

    // Converts from the Pointcloud2 format to PointcloudRGB
    pcl::PointCloud<PointXYZRGB>::Ptr pcl_rgb(
    new pcl::PointCloud<PointXYZRGB>());
    pcl::fromPCLPointCloud2(*pcl_input, *pcl_rgb);

    // Converts RGB pointcloud to HSV
    converter.setInputCloud(pcl_rgb);
    pcl::PointCloud<PointXYZHSV>::Ptr pcl_output(
    new pcl::PointCloud<PointXYZHSV>());
    converter.convert(*pcl_output);

    // Publishes the new cloud
    pub.publish(*pcl_output);
}

// Allows this node to be exported and registered as a nodelet
PLUGINLIB_EXPORT_CLASS(RGBtoHSV, nodelet::Nodelet)
