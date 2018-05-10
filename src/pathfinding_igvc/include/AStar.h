//
// Created by min on 09/05/18.
//

#ifndef PATHFINDING_IGVC_ASTAR_H
#define PATHFINDING_IGVC_ASTAR_H

#include <nav_msgs/Path.h>

class AStar {
public:
    struct GridPoint {
        int col;
        int row;
        GridPoint(int c, int r) : col(c), row(r) {};
    };

    nav_msgs::Path aStarSearch(std::vector<int8_t> grid, GridPoint start, GridPoint goal);
};

#endif //PATHFINDING_IGVC_ASTAR_H
