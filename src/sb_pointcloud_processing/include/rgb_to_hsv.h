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


namespace sb_pointcloud_processing {

class RGBtoHSV : public nodelet::Nodelet {
  public:
    RGBtoHSV();

  private:
    virtual void onInit();

    void callback(const sensor_msgs::PointCloud2::ConstPtr& input);

    ColourspaceConverter converter;

    ros::Publisher pub;
    ros::Subscriber sub;
};
}

#endif // PROJECT_PREPROCESSINGNODE_H
