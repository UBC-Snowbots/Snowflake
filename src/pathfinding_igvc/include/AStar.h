/*
 * Created By: Min Gyo Kim
 * Created On: May 9th 2018
 * Description: Class that implements A* algorithm to find the shortest path
 * between two points given an occupancy grid
 */

#ifndef PATHFINDING_IGVC_ASTAR_H
#define PATHFINDING_IGVC_ASTAR_H

#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <stack>

class AStar {
  public:
    static const int GRID_FREE = 0;
    static const int GRID_OCCUPIED = 100;

    struct GridPoint {
        int col;
        int row;
        GridPoint(int c = 0, int r = 0) : col(c), row(r){};
    };

    /**
     * Takes an occupancy grid as well as start and goal points, and calculates
     * the shortest path from start to goal.
     *
     * @param occupancy_grid occupancy grid
     * @param start GridPoint containing row and column of the starting cell
     * @param goal GridPoint containing row and column of the goal cell
     * @return points stacked in order, where the top contains the starting
     * GridPoint and the bottom contains the goal GridPoint
     */
    std::stack<GridPoint> run(nav_msgs::OccupancyGrid occupancy_grid,
                              GridPoint start,
                              GridPoint goal);
};

#endif // PATHFINDING_IGVC_ASTAR_H
