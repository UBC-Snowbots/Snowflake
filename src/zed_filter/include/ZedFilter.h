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
#include <pcl/filters/extract_indices.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

class ZedFilter {

    typedef pcl::PointXYZRGB Point;
    typedef pcl::PointCloud<Point> PointCloudColour;


public:
    ZedFilter(int argc, char **argv, std::string node_name);
    static PointCloudColour::Ptr filterImage(const sensor_msgs::PointCloud2::ConstPtr& zed_camera_output);

private:
    ros::Subscriber raw_image_subscriber;
    ros::Publisher filtered_image_publisher;

    void imageCallBack(const sensor_msgs::PointCloud2::ConstPtr& zed_camera_output);
    void publishFilteredImage(const PointCloudColour filtered_point_cloud);
};


#endif //PROJECT_ZED_FILTER_H
