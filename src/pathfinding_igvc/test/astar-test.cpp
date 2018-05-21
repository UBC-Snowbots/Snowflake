//
// Created by min on 20/05/18.
//

#include <AStar.h>
#include "PathFinderTestUtils.h"
#include <gtest/gtest.h>

TEST(AStar, FullPathTest) {
    /* origin of OccupancyGrid */
    // initialize origin of occupancy grid
    geometry_msgs::Pose origin =
            PathFinderTestUtils::constructPose(0.0, 0.0, 0.0);

    /* mapMetaData of OccupancyGrid */
    // initialize mapMetaData
    nav_msgs::MapMetaData mapMetaData;
    mapMetaData.resolution = 1.0;
    mapMetaData.width = 10;
    mapMetaData.height = 9;
    // add origin to mapMetaData
    mapMetaData.origin = origin;

    /* OccupancyGrid */
    // initialize occupancy grid
    nav_msgs::OccupancyGrid grid;

    // set mapMetaData
    grid.info = mapMetaData;
    grid.data = {
            AStar::GRID_FREE, AStar::GRID_FREE, AStar::GRID_FREE, AStar::GRID_OCCUPIED, AStar::GRID_OCCUPIED, AStar::GRID_OCCUPIED, AStar::GRID_FREE, AStar::GRID_OCCUPIED, AStar::GRID_OCCUPIED, AStar::GRID_FREE,
            AStar::GRID_FREE, AStar::GRID_OCCUPIED, AStar::GRID_FREE, AStar::GRID_FREE, AStar::GRID_FREE, AStar::GRID_FREE, AStar::GRID_OCCUPIED, AStar::GRID_FREE, AStar::GRID_FREE, AStar::GRID_FREE,
            AStar::GRID_FREE, AStar::GRID_OCCUPIED, AStar::GRID_OCCUPIED, AStar::GRID_OCCUPIED, AStar::GRID_OCCUPIED, AStar::GRID_FREE, AStar::GRID_OCCUPIED, AStar::GRID_OCCUPIED, AStar::GRID_OCCUPIED, AStar::GRID_FREE,
            AStar::GRID_FREE, AStar::GRID_OCCUPIED, AStar::GRID_FREE, AStar::GRID_FREE, AStar::GRID_FREE, AStar::GRID_FREE, AStar::GRID_OCCUPIED, AStar::GRID_FREE, AStar::GRID_OCCUPIED, AStar::GRID_OCCUPIED,
            AStar::GRID_FREE, AStar::GRID_FREE, AStar::GRID_FREE, AStar::GRID_OCCUPIED, AStar::GRID_FREE, AStar::GRID_FREE, AStar::GRID_FREE, AStar::GRID_OCCUPIED, AStar::GRID_FREE, AStar::GRID_OCCUPIED,
            AStar::GRID_OCCUPIED, AStar::GRID_OCCUPIED, AStar::GRID_FREE, AStar::GRID_OCCUPIED, AStar::GRID_FREE, AStar::GRID_OCCUPIED, AStar::GRID_OCCUPIED, AStar::GRID_OCCUPIED, AStar::GRID_OCCUPIED, AStar::GRID_FREE,
            AStar::GRID_FREE, AStar::GRID_FREE, AStar::GRID_FREE, AStar::GRID_OCCUPIED, AStar::GRID_FREE, AStar::GRID_FREE, AStar::GRID_OCCUPIED, AStar::GRID_FREE, AStar::GRID_OCCUPIED, AStar::GRID_FREE,
            AStar::GRID_FREE, AStar::GRID_FREE, AStar::GRID_FREE, AStar::GRID_OCCUPIED, AStar::GRID_FREE, AStar::GRID_FREE, AStar::GRID_FREE, AStar::GRID_OCCUPIED, AStar::GRID_FREE, AStar::GRID_FREE,
            AStar::GRID_FREE, AStar::GRID_OCCUPIED, AStar::GRID_FREE, AStar::GRID_FREE, AStar::GRID_FREE, AStar::GRID_FREE, AStar::GRID_OCCUPIED, AStar::GRID_FREE, AStar::GRID_FREE, AStar::GRID_FREE
    };

    AStar::GridPoint start(0.0, 0.0);
    AStar::GridPoint goal(8.0, 0.0);

    std::stack<AStar::GridPoint> path = AStar::run(grid, start, goal);

    int i = 0;
    while (!path.empty()) {
        AStar::GridPoint point = path.top();
        path.pop();
        std::cout << i++ << ": (" << point.row << "," << point.col << ")" << std::endl;
    }
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
