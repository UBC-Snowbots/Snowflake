//
// Created by min on 05/05/18.
//

#include <AStar.h>
#include <tf/transform_datatypes.h>

// MAIN ENTRY FUNCTION
nav_msgs::Path AStar::perform(geometry_msgs::Point start, geometry_msgs::Point goal, nav_msgs::OccupancyGrid grid) {
    setOccupancyGrid(grid);
    AStar::GridPoint start_on_grid = convertToGridPoint(start);
    AStar::GridPoint goal_on_grid = convertToGridPoint(goal);
    resizeMapToFitGoal(goal_on_grid);
    return calculatePath(start_on_grid, goal_on_grid);
}

// ASTAR ALGO
nav_msgs::Path AStar::calculatePath(AStar::GridPoint start, AStar::GridPoint goal) {
    //TODO
    return nav_msgs::Path();
}

void AStar::setOccupancyGrid(nav_msgs::OccupancyGrid grid) {
    this->_occupancy_grid = grid;

    // setup transformation matrix from map to grid frame
    // convert geometry_msgs::Point to tf::Vector3
    tf::Vector3 origin_position = pointToVector(grid.info.origin.position);
    // convert geometry_msgs::Quaternion to tf::Quaternion
    tf::Quaternion origin_quaternion;
    tf::quaternionMsgToTF(grid.info.origin.orientation, origin_quaternion);

    this->_transformation_to_grid = tf::Transform(origin_quaternion, origin_position).inverse();
}

void AStar::resizeMapToFitGoal(AStar::GridPoint goal) {
    if (goal.col >= (int)this->_occupancy_grid.info.width) {
        unsigned int col_diff = goal.col - this->_occupancy_grid.info.width + 1;

        auto it = this->_occupancy_grid.data.begin() + this->_occupancy_grid.info.width;

        for (unsigned int row = 0; row < this->_occupancy_grid.info.height; row++) {
            it = this->_occupancy_grid.data.insert(it, col_diff, OCC_GRID_FREE);
            it += this->_occupancy_grid.info.width + col_diff;
        }

        this->_occupancy_grid.info.width += col_diff;
    } else if (goal.col < 0) {
        unsigned int col_diff = abs(goal.col);

        auto it = this->_occupancy_grid.data.begin();

        for (unsigned int row = 0; row < this->_occupancy_grid.info.height; row++) {
            it = this->_occupancy_grid.data.insert(it, col_diff, OCC_GRID_FREE);
            it += this->_occupancy_grid.info.width + col_diff;
        }

        this->_occupancy_grid.info.width += col_diff;
        this->_occupancy_grid.info.origin.position.x -= col_diff * this->_occupancy_grid.info.resolution;
    }

    if (goal.row >= (int)this->_occupancy_grid.info.height) {
        unsigned int row_diff = goal.row - this->_occupancy_grid.info.height + 1;
        unsigned int num_additional_cells = row_diff * this->_occupancy_grid.info.width;

        this->_occupancy_grid.data.insert(this->_occupancy_grid.data.end(), num_additional_cells, OCC_GRID_FREE);

        this->_occupancy_grid.info.height += row_diff;
    } else if (goal.row < 0) {
        unsigned int row_diff = abs(goal.row);
        unsigned int num_additional_cells = row_diff * this->_occupancy_grid.info.width;

        this->_occupancy_grid.data.insert(this->_occupancy_grid.data.begin(), num_additional_cells, OCC_GRID_FREE);

        this->_occupancy_grid.info.height += row_diff;
        this->_occupancy_grid.info.origin.position.y -= row_diff * this->_occupancy_grid.info.resolution;
    }
}

AStar::GridPoint AStar::convertToGridPoint(geometry_msgs::Point point) {
    geometry_msgs::Point point_in_grid_frame = transformToGridFrame(point);

    // assumes that cell (0,0) is in the bottom left of the grid
    // TODO: ensure this is true
    int col = point_in_grid_frame.x / this->_occupancy_grid.info.resolution;
    int row = point_in_grid_frame.y / this->_occupancy_grid.info.resolution;

    col = col < 0 ? col - 1 : col;
    row = row < 0 ? row - 1 : row;

    return AStar::GridPoint(col, row);
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

