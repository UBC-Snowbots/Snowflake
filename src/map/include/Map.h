/*
 * Created By: Valerian Ratu
 * Created On: December 23, 2016
 * Description: A node which manages and organizes mapping information from various sources
 */

#ifndef PROJECT_MAP_H
#define PROJECT_MAP_H

#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_core/GridMap.hpp>
#include <grid_map_msgs/GridMap.h>
#include <grid_map_core/GridMapMath.hpp>
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <laser_geometry/laser_geometry.h>
#include <nav_msgs/Odometry.h>
#include <tf2/transform_datatypes.h>
#include <memory>

class Map {
public:
    Map(int argc, char **argv, std::string node_name);
    grid_map::Matrix getVisionLayer();
    grid_map::Matrix getLidarLayer();
    grid_map::Matrix getFootprintLayer();
    double map_width;
    double map_height;
    double map_res;

private:
    void visionCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
    void lidarCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
    void plotPointCloudOnMap(const pcl::PointCloud<pcl::PointXYZ>& cloud, std::string layer);
    void transferMap(grid_map::GridMap* fromMap, grid_map::GridMap* toMap);
    void setUpMap(grid_map::GridMap* map);

    ros::Subscriber vision_sub;
    ros::Subscriber lidar_sub;
    ros::Publisher vision_map_pub;
    ros::Publisher lidar_map_pub;
    ros::Publisher location_map_pub;
    grid_map::GridMap* map;

    ros::Publisher test_pub;

    laser_geometry::LaserProjection projector;

    void transformPointCloud(const sensor_msgs::PointCloud2& input, sensor_msgs::PointCloud2& output, std::string target_frame);
    void convertPointCloud(const sensor_msgs::PointCloud2& input, pcl::PointCloud<pcl::PointXYZ>& output);
};


#endif //PROJECT_MAP_H
