//
// Created by sb on 25/03/17.
//

#ifndef PROJECT_ZED_FILTER_H
#define PROJECT_ZED_FILTER_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>

class ZedFilter {

public:
    ZedFilter(int argc, char **argv, std::string node_name);
    static pcl::PointCloud<pcl::PointXYZRGB>::Ptr filterImage(const sensor_msgs::PointCloud2::ConstPtr& zed_camera_output);

private:
    ros::Subscriber raw_image_subscriber;
    ros::Publisher filtered_image_publisher;

    void imageCallBack(const sensor_msgs::PointCloud2::ConstPtr& zed_camera_output);
    void publishFilteredImage(const pcl::PointCloud<pcl::PointXYZRGB> filtered_point_cloud);
};


#endif //PROJECT_ZED_FILTER_H
