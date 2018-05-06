//
// Created by min on 05/05/18.
//

#ifndef PROJECT_ASTAR_H
#define PROJECT_ASTAR_H

#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Point.h>
#include <tf/LinearMath/Transform.h>

class AStar {
public:
    nav_msgs::OccupancyGrid _occupancy_grid;
    tf::Transform _transformation_to_grid;

    void setOccupancyGrid(nav_msgs::OccupancyGrid grid);

    geometry_msgs::Point transformToGridFrame(geometry_msgs::Point point);

private:
    tf::Vector3 pointToVector(geometry_msgs::Point point);
    geometry_msgs::Point vectorToPoint(tf::Vector3 vector3);
};

#endif //PROJECT_ASTAR_H
