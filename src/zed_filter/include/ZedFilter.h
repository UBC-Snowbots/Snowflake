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
#include <pcl/point_types.h>
#include <pcl/point_types_conversion.h>
#include <sensor_msgs/PointCloud2.h>
#include <sb_utils.h>
#include <PointCloudFilter.h>

class ZedFilter {

    typedef pcl::PointXYZRGB PointRGB;
    typedef pcl::PointXYZHSV PointHSV;
    typedef pcl::PointCloud<PointRGB> PointCloudRGB;
    typedef pcl::PointCloud<PointHSV> PointCloudHSV;

public:

    ZedFilter(int argc, char **argv, std::string node_name);

private:

    ros::Subscriber raw_image_subscriber;
    ros::Publisher filtered_image_publisher;
    PointCloudFilter filter;
    std::string base_link_name;

    void imageCallBack(const sensor_msgs::PointCloud2::ConstPtr& zed_camera_output);
};


#endif //PROJECT_ZED_FILTER_H
