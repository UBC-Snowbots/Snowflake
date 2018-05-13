/*
 * Created By: Min Gyo Kim
 * Created On: May 5th 2018
 * Description: Class that acts as an interface between PathFinderNode and AStar.
 *              Deals with a lot of frame transformations and data type conversions between ROS types and native types.
 *              Does not actually calculate the path itself - depends on AStar to get the path.
 */

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
