//
// Created by min on 09/05/18.
//

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
