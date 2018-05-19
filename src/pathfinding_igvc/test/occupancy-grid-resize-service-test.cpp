/*
 * Created By: Min Gyo Kim
 * Created On: May 14th 2018
 * Description: Unit tests for occupancy grid resize service
 */

#include <OccupancyGridAdapter.h>
#include <OccupancyGridResizer.h>
#include "PathFinderTestUtils.h"
#include <gtest/gtest.h>

TEST(OccupancyGridResizer, TestAddSpaceAroundGrid) {
    /* origin of OccupancyGrid */
    // initialize origin of occupancy grid
    geometry_msgs::Pose origin =
    PathFinderTestUtils::constructPose(3.0, 3.0, 0.0);

    /* map_meta_data of OccupancyGrid */
    // initialize map_meta_data
    nav_msgs::MapMetaData map_meta_data;
    map_meta_data.resolution = 2.0;
    map_meta_data.width      = 2;
    map_meta_data.height     = 3;
    // add origin to map_meta_data
    map_meta_data.origin = origin;

    /* OccupancyGrid */
    // initialize occupancy grid
    nav_msgs::OccupancyGrid grid;
    // set map_meta_data
    grid.info = map_meta_data;
    grid.data = std::vector<int8_t>(6, AStar::GRID_OCCUPIED);

    OccupancyGridResizer::addSpaceAroundGrid(grid);

    EXPECT_EQ(map_meta_data.width + 2, grid.info.width);
    EXPECT_EQ(map_meta_data.height + 2, grid.info.height);
    EXPECT_FLOAT_EQ(map_meta_data.origin.position.x - map_meta_data.resolution, grid.info.origin.position.x);
    EXPECT_FLOAT_EQ(map_meta_data.origin.position.y - map_meta_data.resolution, grid.info.origin.position.y);

    std::vector<int8_t> expected_data = {
            AStar::GRID_FREE, AStar::GRID_FREE, AStar::GRID_FREE, AStar::GRID_FREE,
            AStar::GRID_FREE, AStar::GRID_OCCUPIED, AStar::GRID_OCCUPIED, AStar::GRID_FREE,
            AStar::GRID_FREE, AStar::GRID_OCCUPIED, AStar::GRID_OCCUPIED, AStar::GRID_FREE,
            AStar::GRID_FREE, AStar::GRID_OCCUPIED, AStar::GRID_OCCUPIED, AStar::GRID_FREE,
            AStar::GRID_FREE, AStar::GRID_FREE, AStar::GRID_FREE, AStar::GRID_FREE,
    };

    EXPECT_EQ(expected_data, grid.data);
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
