//
// Created by min on 05/05/18.
//

#ifndef PATHFINDING_IGVC_PATHFINDER_H
#define PATHFINDING_IGVC_PATHFINDER_H

#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Point.h>
#include <tf/LinearMath/Transform.h>
#include <tf/transform_datatypes.h>
#include <AStar.h>

class PathFinder {
public:
    nav_msgs::OccupancyGrid _occupancy_grid;

    tf::Transform _transformation_to_grid;
    tf::Transform _transformation_to_map;

    // MAIN ENTRY FUNCTION
    nav_msgs::Path perform(geometry_msgs::Point start, geometry_msgs::Point goal, nav_msgs::OccupancyGrid grid);

    // ASTAR ALGO
    nav_msgs::Path calculatePath(AStar::GridPoint start, AStar::GridPoint goal);

    void setOccupancyGrid(nav_msgs::OccupancyGrid grid);

    void resizeMapToFitGoal(AStar::GridPoint goal);

    nav_msgs::Path constructPath(std::stack<AStar::GridPoint> points);

    geometry_msgs::PoseStamped constructPoseStamped(geometry_msgs::Point point, double angle);

    static double getAngleBetweenPoints(geometry_msgs::Point from, geometry_msgs::Point to);

    static tf::Quaternion getQuaternionFromAngle(double angle);

    AStar::GridPoint convertToGridPoint(geometry_msgs::Point point);

    geometry_msgs::Point convertToMapPoint(AStar::GridPoint point);

    geometry_msgs::Point transformToGridFrame(geometry_msgs::Point point);

    geometry_msgs::Point transformToMapFrame(geometry_msgs::Point point);

    static tf::Vector3 pointToVector(geometry_msgs::Point point);
    static geometry_msgs::Point vectorToPoint(tf::Vector3 vector3);
};

#endif //PATHFINDING_IGVC_PATHFINDER_H
