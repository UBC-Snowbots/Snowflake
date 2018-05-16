//
// Created by min on 13/05/18.
//

#include <FrameTransformationService.h>

FrameTransformationService
FrameTransformationService::buildService(tf::Quaternion rotation,
                                         tf::Vector3 position) {
    FrameTransformationService service;
    service._transformation_to_map  = tf::Transform(rotation, position);
    service._transformation_to_grid = service._transformation_to_map.inverse();
    return service;
}

geometry_msgs::Point
FrameTransformationService::transformToGridFrame(geometry_msgs::Point point) {
    tf::Vector3 map_point  = PathFinderUtils::pointToVector(point);
    tf::Vector3 grid_point = this->_transformation_to_grid * map_point;
    return PathFinderUtils::vectorToPoint(grid_point);
}

geometry_msgs::Point
FrameTransformationService::transformToMapFrame(geometry_msgs::Point point) {
    tf::Vector3 grid_point = PathFinderUtils::pointToVector(point);
    tf::Vector3 map_point  = this->_transformation_to_map * grid_point;
    return PathFinderUtils::vectorToPoint(map_point);
}