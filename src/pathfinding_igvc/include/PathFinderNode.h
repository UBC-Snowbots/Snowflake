/*
 * Created By: Min Gyo Kim
 * Created On: May 23rd 2018
 * Description: Path Finder Node - subscribes to occupancy grid (nav_msgs/OccupancyGrid)
 * and goal point (geometry_msgs/Point), obtains starting point from tf tree,
 * and publishes the shortest path (nav_msgs/Path) from starting point to goal point.
 */

#ifndef PROJECT_PATHFINDERNODE_H
#define PROJECT_PATHFINDERNODE_H

#include <ros/console.h>
#include <ros/ros.h>
#include <sb_utils.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <PathFinder.h>
#include <tf/transform_listener.h>

class PathFinderNode {
    ros::Subscriber subscriber;
    ros::Publisher publisher;

    geometry_msgs::Point _goal;
    tf::TransformListener _listener;

    std::string _map_frame_name;
    std::string _robot_frame_name;

public:
    PathFinderNode(int argc, char** argv, std::string node_name);

private:
    void OccupancyGridCallback(const nav_msgs::OccupancyGrid grid);
    geometry_msgs::Point getStartPoint();
};

#endif //PROJECT_PATHFINDERNODE_H
