/*
 * Created By: Min Gyo Kim
 * Created On: May 14th 2018
 * Description: Implementation of path constructor
 */

#include <PathConstructor.h>

PathConstructor::PathConstructor(
std::shared_ptr<OccupancyGridAdapter> occupancy_grid_adapter_ptr) {
    this->_occupancy_grid_adapter = occupancy_grid_adapter_ptr;
}

nav_msgs::Path
PathConstructor::constructPath(std::stack<AStar::GridPoint> grid_points) {
    nav_msgs::Path path;

    while (!grid_points.empty()) {
        AStar::GridPoint current_grid_point = grid_points.top();
        geometry_msgs::Point current_map_point =
        this->_occupancy_grid_adapter->convertFromGridToMapPoint(
        current_grid_point);
        grid_points.pop();

        double angle = 0.0;
        if (!grid_points.empty()) {
            AStar::GridPoint next_grid_point = grid_points.top();
            geometry_msgs::Point next_map_point =
            this->_occupancy_grid_adapter->convertFromGridToMapPoint(
            next_grid_point);
            angle = PathFinderUtils::getAngleBetweenPoints(current_map_point,
                                                           next_map_point);
        }

        geometry_msgs::PoseStamped pose_stamped =
        PathFinderUtils::constructPoseStamped(current_map_point, angle);
        path.poses.push_back(pose_stamped);
    }

    return path;
}
