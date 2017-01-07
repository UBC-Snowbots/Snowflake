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
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <geometry_msgs/Pose2D.h>

class Map {
public:
    Map(int argc, char **argv, std::string node_name);
    grid_map::Matrix getVisionLayer();
    grid_map::Matrix getLidarLayer();
    grid_map::Matrix getFootprintLayer();
    std::pair<double, double> getCurrentLocation();


private:
    void visionCallback(const sensor_msgs::PointCloud2ConstPtr msg);
    void lidarCallback(const sensor_msgs::PointCloud2ConstPtr msg);
    void positionCallback(const geometry_msgs::Pose2DConstPtr msg);

    ros::Subscriber vision_sub;
    ros::Subscriber lidar_sub;
    ros::Subscriber position_sub;
    ros::Publisher vision_map_pub;
    ros::Publisher lidar_map_pub;
    ros::Publisher location_map_pub;
    grid_map::GridMap map;

    // TODO: Determine better datatype for position (geometry_msgs::Pose2D?),
    // TODO: currently, it's a grid_map::Position = Eigen::Vector2f, a vector of 2 floats ...
    grid_map::Position pos;
};


#endif //PROJECT_MAP_H
