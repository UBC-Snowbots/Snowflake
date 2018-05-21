/*
 * Created By: Min Gyo Kim
 * Created On: May 14th 2018
 * Description: Implementation of path constructor
 */

#include <PathConstructor.h>

PathConstructor::PathConstructor(OccupancyGridAdapter occupancy_grid_adapter) {
    this->_occupancy_grid_adapter = &occupancy_grid_adapter;
}

nav_msgs::Path
PathConstructor::constructPath(std::stack<AStar::GridPoint> points) {
    nav_msgs::Path path;

    while (!points.empty()) {
        geometry_msgs::Point current_point =
        this->_occupancy_grid_adapter->convertFromGridToMapPoint(points.top());
        points.pop();

        double angle = 0.0;
        if (!points.empty()) {
            geometry_msgs::Point next_point =
            this->_occupancy_grid_adapter->convertFromGridToMapPoint(
            points.top());
            angle =
            PathFinderUtils::getAngleBetweenPoints(current_point, next_point);
        }

        geometry_msgs::PoseStamped pose_stamped =
        PathFinderUtils::constructPoseStamped(current_point, angle);
        path.poses.push_back(pose_stamped);
    }

    return path;
}
