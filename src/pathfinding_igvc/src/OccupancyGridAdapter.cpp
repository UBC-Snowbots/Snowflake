/*
 * Created By: Min Gyo Kim
 * Created On: May 13th 2018
 * Description: Implementation of occupancy grid conversion service
 */

#include <OccupancyGridAdapter.h>

OccupancyGridAdapter::OccupancyGridAdapter(nav_msgs::MapMetaData info) {
    // setup transformation matrix from map to grid frame
    // convert geometry_msgs::Point to tf::Vector3
    tf::Vector3 origin_position =
    PathFinderUtils::pointToVector(info.origin.position);
    // convert geometry_msgs::Quaternion to tf::Quaternion
    tf::Quaternion origin_quaternion;
    tf::quaternionMsgToTF(info.origin.orientation, origin_quaternion);

    this->_transformation_service = new FrameTransformer(
    origin_quaternion, origin_position);
    this->_grid_info = info;
}

AStar::GridPoint
OccupancyGridAdapter::convertFromMapToGridPoint(geometry_msgs::Point point) {
    geometry_msgs::Point point_in_grid_frame =
    this->_transformation_service->transformFromMapToGridFrame(point);

    // cell (0,0) is in the bottom left of the grid
    int col = point_in_grid_frame.x / this->_grid_info.resolution;
    int row = point_in_grid_frame.y / this->_grid_info.resolution;

    col = col < 0 ? col -1 : col;
//    col = col >= this->_grid_info.width ? this->_grid_info.width - 1 : col;

    row = row < 0 ? row - 1 : row;
//    row = row >= this->_grid_info.height ? this->_grid_info.height - 1 : row;

    return AStar::GridPoint(col, row);
}

geometry_msgs::Point
OccupancyGridAdapter::convertFromGridToMapPoint(AStar::GridPoint grid_point) {
    geometry_msgs::Point point;
    point.x = grid_point.col * this->_grid_info.resolution;
    point.y = grid_point.row * this->_grid_info.resolution;

    return this->_transformation_service->transformFromGridToMapFrame(point);
}
