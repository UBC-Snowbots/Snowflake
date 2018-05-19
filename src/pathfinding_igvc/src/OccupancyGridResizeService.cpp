/*
 * Created By: Min Gyo Kim
 * Created On: May 14th 2018
 * Description: Implementation of occupancy grid resize service
 */

#include <OccupancyGridResizeService.h>

void OccupancyGridResizeService::addSpaceAroundGrid(nav_msgs::OccupancyGrid& grid) {
    addSpaceLeft(grid);
    addSpaceRight(grid);
    addSpaceUp(grid);
    addSpaceDown(grid);
}

void OccupancyGridResizeService::addSpaceLeft(nav_msgs::OccupancyGrid& grid) {
    auto it = grid.data.begin();

    for (unsigned int row = 0; row < grid.info.height; row++) {
        it = grid.data.insert(it, 1, AStar::GRID_FREE);
        it += grid.info.width + 1;
    }

    grid.info.width++;
    grid.info.origin.position.x -= grid.info.resolution;
}

void OccupancyGridResizeService::addSpaceRight(nav_msgs::OccupancyGrid& grid) {
    auto it = grid.data.begin() + grid.info.width;

    for (unsigned int row = 0; row < grid.info.height; row++) {
        it = grid.data.insert(it, 1, AStar::GRID_FREE);
        it += grid.info.width + 1;
    }

    grid.info.width++;
}

void OccupancyGridResizeService::addSpaceUp(nav_msgs::OccupancyGrid& grid) {
    grid.data.insert(grid.data.end(), grid.info.width, AStar::GRID_FREE);

    grid.info.height++;
}

void OccupancyGridResizeService::addSpaceDown(nav_msgs::OccupancyGrid& grid) {
    grid.data.insert(grid.data.begin(), grid.info.width, AStar::GRID_FREE);

    grid.info.height++;
    grid.info.origin.position.y -= grid.info.resolution;
}
