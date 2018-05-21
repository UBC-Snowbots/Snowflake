/*
 * Created By: Min Gyo Kim
 * Created On: May 5th 2018
 * Description: Class that acts as an interface between PathFinderNode and
 * AStar.
 *              Deals with a lot of frame transformations and data type
 * conversions between ROS types and native types.
 *              Does not actually calculate the path itself - depends on AStar
 * to get the path.
 */

#ifndef PATHFINDING_IGVC_PATHFINDER_H
#define PATHFINDING_IGVC_PATHFINDER_H

#include <AStar.h>
#include <OccupancyGridAdapter.h>
#include <OccupancyGridResizer.h>
#include <PathConstructor.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <tf/LinearMath/Transform.h>
#include <tf/transform_datatypes.h>

class PathFinder {
  public:
    /**
     * Takes a start and goal points in world frame, and returns the shortest
     * path
     * from start to goal based on information from occupancy grid.
     *
     * @param start starting point in world frame
     * @param goal goal point in world frame
     * @param grid occupancy grid
     * @return shortest path from start to goal
     */
    static nav_msgs::Path calculatePath(geometry_msgs::Point start,
                                        geometry_msgs::Point goal,
                                        nav_msgs::OccupancyGrid grid);

    /**
     * Takes in a grid and adds space around the grid if the goal is not
     * inside the grid.
     *
     * Also calculates row and col of starting point and goal point in
     * occupancy grid (represented as AStar::GridPoint) into start_on_grid
     * and goal_on_grid.
     *
     * @param grid the occupancy grid
     * @param start the start point in map frame
     * @param goal the goal point in map frame
     * @param start_on_grid transformation of start point into occupancy grid
     * @param goal_on_grid transformation of goal point into occupancy grid
     */
    static void
    processGridAndGetStartAndGoalOnGrid(nav_msgs::OccupancyGrid& grid,
                                        geometry_msgs::Point start,
                                        geometry_msgs::Point goal,
                                        AStar::GridPoint& start_on_grid,
                                        AStar::GridPoint& goal_on_grid);
};

#endif // PATHFINDING_IGVC_PATHFINDER_H
