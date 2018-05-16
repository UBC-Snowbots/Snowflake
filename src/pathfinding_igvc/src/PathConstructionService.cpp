//
// Created by min on 14/05/18.
//

#include <PathConstructionService.h>

PathConstructionService PathConstructionService::buildService(
OccupancyGridConversionService occupancy_grid_conversion_service) {
    PathConstructionService path_construction_service;
    path_construction_service._occupancy_grid_conversion_service =
    occupancy_grid_conversion_service;
    return path_construction_service;
}

nav_msgs::Path
PathConstructionService::constructPath(std::stack<AStar::GridPoint> points) {
    nav_msgs::Path path;

    while (!points.empty()) {
        geometry_msgs::Point current_point =
        this->_occupancy_grid_conversion_service.convertToMapPoint(
        points.top());
        points.pop();

        double angle = 0.0;
        if (!points.empty()) {
            geometry_msgs::Point next_point =
            this->_occupancy_grid_conversion_service.convertToMapPoint(
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