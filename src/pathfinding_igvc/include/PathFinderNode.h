/*
 * Created By: Min Gyo Kim
 * Created On: May 23rd 2018
 * Description: Path Finder Node - subscribes to occupancy grid
 * (nav_msgs/OccupancyGrid)
 * and goal point (geometry_msgs/Point), obtains starting point from tf tree,
 * and publishes the shortest path (nav_msgs/Path) from starting point to goal
 * point.
 */

#ifndef PROJECT_PATHFINDERNODE_H
#define PROJECT_PATHFINDERNODE_H

#include <PathFinder.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <sb_utils.h>
#include <tf/transform_listener.h>

class PathFinderNode {
  private:
    ros::Subscriber grid_subscriber;
    ros::Subscriber goal_subscriber;
    ros::Publisher publisher;

    tf::TransformListener* _listener;

    std::string _global_frame_name;
    std::string _base_frame_name;

    geometry_msgs::Point _goal;
    nav_msgs::OccupancyGrid _grid;

    bool _received_goal  = false;
    bool _receivied_grid = false;

  public:
    // constructor
    PathFinderNode(int argc, char** argv, std::string node_name);

  private:
    /**
     * Function that receives the occupancy grid.
     * It first stores the grid as a member variable of this class,
     * and checks if a goal has been received. If we have a goal,
     * then calculates the path and publishes it.
     *
     * @param grid the occupancy grid
     */
    void occupancyGridCallback(const nav_msgs::OccupancyGrid grid);

    /**
     * Function that receives the goal point (in map frame)
     * It first stores the goal as a member variable of this class,
     * and checks if a grid has been received. If we have a grid,
     * then calculates the path and publishes it.
     *
     * @param goal
     */
    void goalCallback(const geometry_msgs::Point goal);

    /**
     * Function that publishes the path.
     * It is responsible for getting the starting position by
     * getting the current position of the robot through the tf
     * tree, then calling the function that returns the path.
     */
    void publishPath();
};

#endif // PROJECT_PATHFINDERNODE_H
