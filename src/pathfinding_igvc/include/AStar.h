/*
 * Created By: Min Gyo Kim
 * Created On: May 9th 2018
 * Description: Class that implements A* algorithm to find the shortest path between two points given an occupancy grid
 */

#ifndef PATHFINDING_IGVC_ASTAR_H
#define PATHFINDING_IGVC_ASTAR_H

#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <stack>

#define GRID_FREE 0
#define GRID_OCCUPIED 100

class AStar {
public:
    struct GridPoint {
        int col;
        int row;
        GridPoint(int c, int r) : col(c), row(r) {};
    };

    std::stack<GridPoint> run(nav_msgs::OccupancyGrid occupancy_grid, GridPoint start, GridPoint goal);
};

#endif //PATHFINDING_IGVC_ASTAR_H
