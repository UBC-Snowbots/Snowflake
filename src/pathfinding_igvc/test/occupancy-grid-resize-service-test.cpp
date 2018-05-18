/*
 * Created By: Min Gyo Kim
 * Created On: May 14th 2018
 * Description: Unit tests for occupancy grid resize service
 */

#include <OccupancyGridConversionService.h>
#include <OccupancyGridResizeService.h>
#include <PathFinderTestUtils.h>
#include <gtest/gtest.h>

TEST(OccupancyGridResizeService, TestResizeMapExpandRight) {
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

    geometry_msgs::Point point;
    point.x = 8.0;
    point.y = 3.0;
    point.z = 0.0;

    AStar::GridPoint grid_point =
    OccupancyGridConversionService(map_meta_data)
    .convertToGridPoint(point);

    EXPECT_NEAR(grid_point.col, 2.0, 0.01);
    EXPECT_NEAR(grid_point.row, 0.0, 0.01);

    OccupancyGridResizeService::resizeOccupancyGridToFitGoal(grid, grid_point);

    EXPECT_EQ(grid.data.size(), 9);
    EXPECT_EQ(grid.info.width, 3);
    EXPECT_EQ(grid.info.height, 3);
    EXPECT_EQ(grid.info.resolution, 2.0);
    EXPECT_EQ(grid.info.origin.position.x, origin.position.x);
    EXPECT_EQ(grid.info.origin.position.y, origin.position.y);

    std::vector<int8_t> expected_data = {AStar::GRID_OCCUPIED,
                                         AStar::GRID_OCCUPIED,
                                         AStar::GRID_FREE,
                                         AStar::GRID_OCCUPIED,
                                         AStar::GRID_OCCUPIED,
                                         AStar::GRID_FREE,
                                         AStar::GRID_OCCUPIED,
                                         AStar::GRID_OCCUPIED,
                                         AStar::GRID_FREE};
    EXPECT_EQ(grid.data, expected_data);
}

TEST(OccupancyGridResizeService, TestResizeMapExpandLeft) {
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

    geometry_msgs::Point point;
    point.x = 0.0;
    point.y = 3.0;
    point.z = 0.0;

    AStar::GridPoint grid_point =
    OccupancyGridConversionService(map_meta_data)
    .convertToGridPoint(point);

    ASSERT_NEAR(grid_point.col, -2.0, 0.01);
    ASSERT_NEAR(grid_point.row, 0.0, 0.01);

    OccupancyGridResizeService::resizeOccupancyGridToFitGoal(grid, grid_point);

    EXPECT_EQ(grid.data.size(), 12);
    EXPECT_EQ(grid.info.width, 4);
    EXPECT_EQ(grid.info.height, 3);
    EXPECT_EQ(grid.info.resolution, 2.0);
    EXPECT_EQ(grid.info.origin.position.x, -1.0);
    EXPECT_EQ(grid.info.origin.position.y, 3.0);

    std::vector<int8_t> expected_data = {AStar::GRID_FREE,
                                         AStar::GRID_FREE,
                                         AStar::GRID_OCCUPIED,
                                         AStar::GRID_OCCUPIED,
                                         AStar::GRID_FREE,
                                         AStar::GRID_FREE,
                                         AStar::GRID_OCCUPIED,
                                         AStar::GRID_OCCUPIED,
                                         AStar::GRID_FREE,
                                         AStar::GRID_FREE,
                                         AStar::GRID_OCCUPIED,
                                         AStar::GRID_OCCUPIED};
    EXPECT_EQ(grid.data, expected_data);
}

TEST(OccupancyGridResizeService, TestResizeMapExpandUp) {
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

    geometry_msgs::Point point;
    point.x = 3.0;
    point.y = 12.0;
    point.z = 0.0;

    AStar::GridPoint grid_point =
    OccupancyGridConversionService(map_meta_data)
    .convertToGridPoint(point);

    EXPECT_NEAR(grid_point.col, 0.0, 0.01);
    EXPECT_NEAR(grid_point.row, 4.0, 0.01);

    OccupancyGridResizeService::resizeOccupancyGridToFitGoal(grid, grid_point);

    EXPECT_EQ(grid.data.size(), 10);
    EXPECT_EQ(grid.info.width, 2);
    EXPECT_EQ(grid.info.height, 5);
    EXPECT_EQ(grid.info.resolution, 2.0);
    EXPECT_EQ(grid.info.origin.position.x, origin.position.x);
    EXPECT_EQ(grid.info.origin.position.y, origin.position.y);

    std::vector<int8_t> expected_data = {
    AStar::GRID_OCCUPIED,
    AStar::GRID_OCCUPIED,
    AStar::GRID_OCCUPIED,
    AStar::GRID_OCCUPIED,
    AStar::GRID_OCCUPIED,
    AStar::GRID_OCCUPIED,
    AStar::GRID_FREE,
    AStar::GRID_FREE,
    AStar::GRID_FREE,
    AStar::GRID_FREE,
    };

    EXPECT_EQ(grid.data, expected_data);
}

TEST(OccupancyGridResizeService, TestResizeMapExpandDown) {
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

    geometry_msgs::Point point;
    point.x = 3.0;
    point.y = 0.0;
    point.z = 0.0;

    AStar::GridPoint grid_point =
    OccupancyGridConversionService(map_meta_data)
    .convertToGridPoint(point);

    EXPECT_NEAR(grid_point.col, 0.0, 0.01);
    EXPECT_NEAR(grid_point.row, -2.0, 0.01);

    OccupancyGridResizeService::resizeOccupancyGridToFitGoal(grid, grid_point);

    EXPECT_EQ(grid.data.size(), 10);
    EXPECT_EQ(grid.info.width, 2);
    EXPECT_EQ(grid.info.height, 5);
    EXPECT_EQ(grid.info.resolution, 2.0);
    EXPECT_EQ(grid.info.origin.position.x, 3.0);
    EXPECT_EQ(grid.info.origin.position.y, -1.0);

    std::vector<int8_t> expected_data = {
    AStar::GRID_FREE,
    AStar::GRID_FREE,
    AStar::GRID_FREE,
    AStar::GRID_FREE,
    AStar::GRID_OCCUPIED,
    AStar::GRID_OCCUPIED,
    AStar::GRID_OCCUPIED,
    AStar::GRID_OCCUPIED,
    AStar::GRID_OCCUPIED,
    AStar::GRID_OCCUPIED,
    };

    EXPECT_EQ(grid.data, expected_data);
}

TEST(OccupancyGridResizeService, TestResizeMapExpandLeftAndDown) {
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

    geometry_msgs::Point point;
    point.x = 0.0;
    point.y = 0.0;
    point.z = 0.0;

    AStar::GridPoint grid_point =
    OccupancyGridConversionService(map_meta_data)
    .convertToGridPoint(point);

    EXPECT_NEAR(grid_point.col, -2.0, 0.01);
    EXPECT_NEAR(grid_point.row, -2.0, 0.01);

    OccupancyGridResizeService::resizeOccupancyGridToFitGoal(grid, grid_point);

    EXPECT_EQ(grid.data.size(), 20);
    EXPECT_EQ(grid.info.width, 4);
    EXPECT_EQ(grid.info.height, 5);
    EXPECT_EQ(grid.info.resolution, 2.0);
    EXPECT_EQ(grid.info.origin.position.x, -1.0);
    EXPECT_EQ(grid.info.origin.position.y, -1.0);

    std::vector<int8_t> expected_data = {
    AStar::GRID_FREE,     AStar::GRID_FREE,     AStar::GRID_FREE, AStar::GRID_FREE,     AStar::GRID_FREE,
    AStar::GRID_FREE,     AStar::GRID_FREE,     AStar::GRID_FREE, AStar::GRID_FREE,     AStar::GRID_FREE,
    AStar::GRID_OCCUPIED, AStar::GRID_OCCUPIED, AStar::GRID_FREE, AStar::GRID_FREE,     AStar::GRID_OCCUPIED,
    AStar::GRID_OCCUPIED, AStar::GRID_FREE,     AStar::GRID_FREE, AStar::GRID_OCCUPIED, AStar::GRID_OCCUPIED,
    };

    EXPECT_EQ(grid.data, expected_data);
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
