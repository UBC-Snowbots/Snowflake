/**
 * created by: valerian ratu
 * Created on: 2018/01/13
 * Description: A ros nodelet which takes pointcloud input in RGB colourspace
 *              and returns it in the HSV colourspace.
 */

#ifndef PROJECT_PREPROCESSINGNODE_H
#define PROJECT_PREPROCESSINGNODE_H

// ROS Includes
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

// PCL Includes
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/filters/filter.h>

#include <ColourspaceConverter.h>

using namespace pcl;
using namespace pcl_ros;

namespace sb_pointcloud_processing {

class RGBtoHSV : public nodelet::Nodelet {
  public:
    RGBtoHSV();

  private:
    virtual void onInit();

    inline void filter(const sensor_msgs::PointCloud2::ConstPtr& input,
                       sensor_msgs::PointCloud2& output) {
        pcl::PCLPointCloud2::Ptr pcl_input(new pcl::PCLPointCloud2);
        pcl_conversions::toPCL(*(input), *(pcl_input));
        pcl::PointCloud<PointXYZRGB>::Ptr pcl_rgb(
        new pcl::PointCloud<PointXYZRGB>());
        pcl::fromPCLPointCloud2(*pcl_input, *pcl_rgb);
        impl_.setInputCloud(pcl_rgb);
        pcl::PointCloud<PointXYZHSV>::Ptr pcl_output(
        new pcl::PointCloud<PointXYZHSV>());
        impl_.filter(*pcl_output);
        pcl::toROSMsg(*pcl_output, output);
    }

    void callback(const sensor_msgs::PointCloud2::ConstPtr& input);

    ColourspaceConverter impl_;

    ros::Publisher pub;
    ros::Subscriber sub;
};
}

#endif // PROJECT_PREPROCESSINGNODE_H
