//
// Created by min on 05/05/18.
//

#include <AStar.h>
#include <tf/transform_datatypes.h>

void AStar::setOccupancyGrid(nav_msgs::OccupancyGrid grid) {
    this->_occupancy_grid = grid;

    // setup transformation matrix from map to grid frame
    tf::Vector3 origin_position = pointToVector(grid.info.origin.position);
    tf::Quaternion origin_quaternion;
    tf::quaternionMsgToTF(grid.info.origin.orientation, origin_quaternion);
    this->_transformation_to_grid = tf::Transform(origin_quaternion, origin_position).inverse();
}

geometry_msgs::Point AStar::transformToGridFrame(geometry_msgs::Point point) {
    tf::Vector3 map_point = pointToVector(point);
    tf::Vector3 grid_point = this->_transformation_to_grid * map_point;
    return vectorToPoint(grid_point);
}

tf::Vector3 AStar::pointToVector(geometry_msgs::Point point) {
    return tf::Vector3(point.x, point.y, point.z);
}

geometry_msgs::Point AStar::vectorToPoint(tf::Vector3 vector3) {
    geometry_msgs::Point p;
    p.x = vector3.x();
    p.y = vector3.y();
    p.z = vector3.z();
    return p;
}

