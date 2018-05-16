/*
 * Created By: Min Gyo Kim
 * Created On: May 14th 2018
 * Description: Implementation of occupancy grid resize service
 */

#include <OccupancyGridResizeService.h>

void OccupancyGridResizeService::resizeOccupancyGridToFitGoal(
nav_msgs::OccupancyGrid& grid, AStar::GridPoint goal) {
    if (goal.col >= (int) grid.info.width) {
        unsigned int col_diff = goal.col - grid.info.width + 1;

        auto it = grid.data.begin() + grid.info.width;

        for (unsigned int row = 0; row < grid.info.height; row++) {
            it = grid.data.insert(it, col_diff, GRID_FREE);
            it += grid.info.width + col_diff;
        }

        grid.info.width += col_diff;
    } else if (goal.col < 0) {
        unsigned int col_diff = abs(goal.col);

        auto it = grid.data.begin();

        for (unsigned int row = 0; row < grid.info.height; row++) {
            it = grid.data.insert(it, col_diff, GRID_FREE);
            it += grid.info.width + col_diff;
        }

        grid.info.width += col_diff;
        grid.info.origin.position.x -= col_diff * grid.info.resolution;
    }

    if (goal.row >= (int) grid.info.height) {
        unsigned int row_diff             = goal.row - grid.info.height + 1;
        unsigned int num_additional_cells = row_diff * grid.info.width;

        grid.data.insert(grid.data.end(), num_additional_cells, GRID_FREE);

        grid.info.height += row_diff;
    } else if (goal.row < 0) {
        unsigned int row_diff             = abs(goal.row);
        unsigned int num_additional_cells = row_diff * grid.info.width;

        grid.data.insert(grid.data.begin(), num_additional_cells, GRID_FREE);

        grid.info.height += row_diff;
        grid.info.origin.position.y -= row_diff * grid.info.resolution;
    }
}
