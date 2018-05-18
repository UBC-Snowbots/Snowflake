/*
 * Created By: Min Gyo Kim
 * Created On: May 13th 2018
 * Description: Implementation of frame transformation service
 */

#include <FrameTransformationService.h>

FrameTransformationService::FrameTransformationService(tf::Quaternion rotation,
                                         tf::Vector3 position) {
    this->_transformation_to_map = tf::Transform(rotation, position);
    this->_transformation_to_grid = this->_transformation_to_map.inverse();
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