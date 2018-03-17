/**
 * created by: valerian ratu
 * Created on: 2018/01/13
 * Description: A ros nodelet which takes pointcloud input in RGB colourspace
 *              and returns it in the HSV colourspace.
 */

#ifndef SB_POINTCLOUD_PROCESSING_RGB_TO_HSV_H
#define SB_POINTCLOUD_PROCESSING_RGB_TO_HSV_H

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
    /**
     * Empty constructor
     */
    RGBtoHSV();

  private:
    /**
     * Initializes the nodelet
     */
    virtual void onInit();

    /**
     * Callback which converts a given pointcloud from RGB to HSV
     *
     * @param input the RGB pointcloud to be converted
     */
    void callback(const sensor_msgs::PointCloud2::ConstPtr& input);

    // A converter from RGB to HSV
    ColourspaceConverter converter;

    // Publishes the HSV pointcloud
    ros::Publisher pub;

    // Subscribes to the RGB pointcloud
    ros::Subscriber sub;
};
}

#endif // SB_POINTCLOUD_PROCESSING_RGB_TO_HSV_H
