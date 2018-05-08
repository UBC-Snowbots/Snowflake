//
// Created by min on 05/05/18.
//

#ifndef PROJECT_ASTAR_H
#define PROJECT_ASTAR_H

#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Point.h>
#include <tf/LinearMath/Transform.h>

#define OCC_GRID_FREE 0
#define OCC_GRID_OCCUPIED 100

class AStar {
public:
    struct GridPoint {
        int col;
        int row;
        GridPoint(int c, int r) : col(c), row(r) {};
    };

    nav_msgs::OccupancyGrid _occupancy_grid;
    tf::Transform _transformation_to_grid;

    void setOccupancyGrid(nav_msgs::OccupancyGrid grid);

    void resizeMapToFitGoal(GridPoint goal);

    GridPoint convertToGridPoint(geometry_msgs::Point point);

    geometry_msgs::Point transformToGridFrame(geometry_msgs::Point point);

private:
    tf::Vector3 pointToVector(geometry_msgs::Point point);
    geometry_msgs::Point vectorToPoint(tf::Vector3 vector3);
};

#endif //PROJECT_ASTAR_H
