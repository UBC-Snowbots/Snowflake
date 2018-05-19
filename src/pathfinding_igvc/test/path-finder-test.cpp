/*
 * Created By: Min Gyo Kim
 * Created On: May 19th 2018
 * Description: Unit Tests for Path Finder
 */

#include <gtest/gtest.h>
#include <PathFinder.h>
#include "PathFinderTestUtils.h"

TEST(PathFinder, ProcessGridAndGetStartAndGoalOnGrid) {
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

    /* Starting point in map frame */
    geometry_msgs::Point start;
    start.x = 4.8;
    start.y = 7.9;

    /* Goal point in map frame */
    geometry_msgs::Point goal;
    goal.x = 9999;
    goal.y = 6.3;

    /* Declaration of starting and goal point in grid frame */
    AStar::GridPoint start_on_grid;
    AStar::GridPoint goal_on_grid;

    /* Function being tested */
    PathFinder::processGridAndGetStartAndGoalOnGrid(grid, start, goal, start_on_grid, goal_on_grid);

    /* Verify that grid has been resized */
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

    /* Verify that starting point has been correctly transformed into the resized grid frame */
    EXPECT_EQ(1, start_on_grid.col);
    EXPECT_EQ(3, start_on_grid.row);

    /* Verify that goal point has been correctly transformed into the resized grid frame and fit into the grid */
    EXPECT_EQ(3, goal_on_grid.col);
    EXPECT_EQ(2, goal_on_grid.row);
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
