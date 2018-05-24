/*
 * Created By: Min Gyo Kim
 * Created On: May 19th 2018
 * Description: Unit Tests for Path Finder
 */

#include "PathFinderTestUtils.h"
#include <PathFinder.h>
#include <gtest/gtest.h>

signed char _ = AStar::GRID_FREE;
signed char X = AStar::GRID_OCCUPIED;

TEST(PathFinder, TestPathWithNoObstacle) {
    /* origin of OccupancyGrid */
    // initialize origin of occupancy grid
    geometry_msgs::Pose origin =
    PathFinderTestUtils::constructPose(3.0, 3.0, 0.0);

    /* mapMetaData of OccupancyGrid */
    // initialize mapMetaData
    nav_msgs::MapMetaData mapMetaData;
    mapMetaData.resolution = 2.0;
    mapMetaData.width      = 2;
    mapMetaData.height     = 2;
    // add origin to mapMetaData
    mapMetaData.origin = origin;

    /* OccupancyGrid */
    // initialize occupancy grid
    nav_msgs::OccupancyGrid grid;

    // set mapMetaData
    grid.info = mapMetaData;
    grid.data = std::vector<int8_t>(4, _);

    geometry_msgs::Point start;
    start.x = 3.0;
    start.y = 3.0;
    geometry_msgs::Point goal;
    goal.x = 5.0;
    goal.y = 5.0;

    nav_msgs::Path path = PathFinder::calculatePath(start, goal, grid);

    ASSERT_EQ(path.poses.size(), 2);

    EXPECT_FLOAT_EQ(path.poses[0].pose.position.x, 3.0);
    EXPECT_FLOAT_EQ(path.poses[0].pose.position.y, 3.0);

    EXPECT_FLOAT_EQ(path.poses[1].pose.position.x, 5.0);
    EXPECT_FLOAT_EQ(path.poses[1].pose.position.y, 5.0);
}

TEST(PathFinder, TestFullPath) {
    /* origin of OccupancyGrid */
    // initialize origin of occupancy grid
    geometry_msgs::Pose origin =
    PathFinderTestUtils::constructPose(0.0, 0.0, 0.0);

    /* mapMetaData of OccupancyGrid */
    // initialize mapMetaData
    nav_msgs::MapMetaData mapMetaData;
    mapMetaData.resolution = 1.0;
    mapMetaData.width      = 10;
    mapMetaData.height     = 9;
    // add origin to mapMetaData
    mapMetaData.origin = origin;

    /* OccupancyGrid */
    // initialize occupancy grid
    nav_msgs::OccupancyGrid grid;

    // set mapMetaData
    grid.info = mapMetaData;
    grid.data = {_, _, _, X, X, X, _, X, X, _, _, X, _, _, _, _, X, _,
                 _, _, _, X, X, X, X, _, X, X, X, _, _, X, _, _, _, _,
                 X, _, X, X, _, _, _, X, _, _, _, X, _, X, X, X, _, X,
                 _, X, X, X, X, _, _, _, _, X, _, _, X, _, X, _, _, _,
                 _, X, _, _, _, X, _, _, _, X, _, _, _, _, X, _, _, _};

    geometry_msgs::Point start;
    start.x = 0.0;
    start.y = 0.0;
    geometry_msgs::Point goal;
    goal.x = 0.0;
    goal.y = 8.0;

    nav_msgs::Path path = PathFinder::calculatePath(start, goal, grid);

    ASSERT_EQ(path.poses.size(), 9);

    std::vector<float> expected_x = {
    0.0, 0.0, 0.0, 0.0, 1.0, 2.0, 1.0, 0.0, 0.0};
    std::vector<float> expected_y = {
    0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0};

    for (int i = 0; i < path.poses.size(); i++) {
        EXPECT_FLOAT_EQ(expected_x[i], path.poses[i].pose.position.x);
        EXPECT_FLOAT_EQ(expected_y[i], path.poses[i].pose.position.y);
    }
}

TEST(PathFinder, PathFindingWhenGoalNotInGrid) {
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
    grid.data = {
    X, X, X, X, _, _,
    };

    /* Starting point in map frame */
    geometry_msgs::Point start;
    start.x = 3.0;
    start.y = 7.0;

    /* Goal point in map frame */
    geometry_msgs::Point goal;
    goal.x = 3.0;
    goal.y = -9999;

    nav_msgs::Path path = PathFinder::calculatePath(start, goal, grid);

    EXPECT_EQ(path.poses.size(), 4);

    std::vector<float> expected_x = {3.0, 1.0, 1.0, 3.0};
    std::vector<float> expected_y = {7.0, 5.0, 3.0, 1.0};
    for (int i = 0; i < path.poses.size(); i++) {
        EXPECT_FLOAT_EQ(expected_x[i], path.poses[i].pose.position.x);
        EXPECT_FLOAT_EQ(expected_y[i], path.poses[i].pose.position.y);
    }
}

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
    grid.data = std::vector<int8_t>(6, X);

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
    PathFinder::processGridAndGetStartAndGoalOnGrid(
    grid, start, goal, start_on_grid, goal_on_grid);

    /* Verify that grid has been resized */
    EXPECT_EQ(map_meta_data.width + 2, grid.info.width);
    EXPECT_EQ(map_meta_data.height + 2, grid.info.height);
    EXPECT_FLOAT_EQ(map_meta_data.origin.position.x - map_meta_data.resolution,
                    grid.info.origin.position.x);
    EXPECT_FLOAT_EQ(map_meta_data.origin.position.y - map_meta_data.resolution,
                    grid.info.origin.position.y);

    std::vector<int8_t> expected_data = {
    _, _, _, _, _, X, X, _, _, X, X, _, _, X, X, _, _, _, _, _,
    };

    EXPECT_EQ(expected_data, grid.data);

    /* Verify that starting point has been correctly transformed into the
     * resized grid frame */
    EXPECT_EQ(1, start_on_grid.col);
    EXPECT_EQ(3, start_on_grid.row);

    /* Verify that goal point has been correctly transformed into the resized
     * grid frame and fit into the grid */
    EXPECT_EQ(3, goal_on_grid.col);
    EXPECT_EQ(2, goal_on_grid.row);
}

TEST(PathFinder, ProcessGridAndGetStartAndGoalOnGridWithAngle) {
    /* origin of OccupancyGrid */
    // initialize origin of occupancy grid
    geometry_msgs::Pose origin =
    PathFinderTestUtils::constructPose(3.0, 3.0, M_PI / 6);

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
    grid.data = std::vector<int8_t>(6, X);

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
    PathFinder::processGridAndGetStartAndGoalOnGrid(
    grid, start, goal, start_on_grid, goal_on_grid);

    /* Verify that grid has been resized */
    EXPECT_EQ(map_meta_data.width + 2, grid.info.width);
    EXPECT_EQ(map_meta_data.height + 2, grid.info.height);

    float expected_origin_x = map_meta_data.origin.position.x -
                              map_meta_data.resolution * cos(M_PI / 6) +
                              map_meta_data.resolution * sin(M_PI / 6);
    float expected_origin_y = map_meta_data.origin.position.y -
                              map_meta_data.resolution * sin(M_PI / 6) -
                              map_meta_data.resolution * cos(M_PI / 6);
    EXPECT_FLOAT_EQ(expected_origin_x, grid.info.origin.position.x);
    EXPECT_FLOAT_EQ(expected_origin_y, grid.info.origin.position.y);

    std::vector<int8_t> expected_data = {
    _, _, _, _, _, X, X, _, _, X, X, _, _, X, X, _, _, _, _, _,
    };

    EXPECT_EQ(expected_data, grid.data);
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
