/*
 * Created By: Min Gyo Kim
 * Created On: May 20th 2018
 * Description: Tests for AStar class
 */

#include "PathFinderTestUtils.h"
#include <AStar.h>
#include <gtest/gtest.h>

signed char _ = AStar::GRID_FREE;
signed char X = AStar::GRID_OCCUPIED;

TEST(AStar, FullPathTest) {
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

    AStar::GridPoint start(0.0, 0.0);
    AStar::GridPoint goal(8.0, 0.0);

    std::stack<AStar::GridPoint> path = AStar::run(grid, start, goal);

    std::vector<int> expected_x = {0, 0, 0, 0, 1, 2, 1, 0, 0};
    std::vector<int> expected_y = {0, 1, 2, 3, 4, 5, 6, 7, 8};

    int i = 0;
    while (!path.empty()) {
        AStar::GridPoint point = path.top();
        path.pop();

        EXPECT_EQ(expected_x[i], point.col);
        EXPECT_EQ(expected_y[i], point.row);

        i++;
    }
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
