/*
 * Created By: Min Gyo Kim
 * Created On: May 14th 2018
 * Description: Unit tests for path finder utils class
 */

#include <PathFinderUtils.h>
#include <gtest/gtest.h>
#include "PathFinderTestUtils.h"

TEST(PathFinderUtils, TestGetAngleBetweenPoints) {
    geometry_msgs::Point from;
    from.x = -99.0;
    from.y = -99.0;
    from.z = 0.0;

    geometry_msgs::Point to;
    to.x = from.x - sqrt(3);
    to.y = from.y - 1.0;
    to.z = 0.0;

    double angle     = PathFinderUtils::getAngleBetweenPoints(from, to);
    tf::Quaternion q = PathFinderUtils::getQuaternionFromAngle(angle);
    EXPECT_FLOAT_EQ(M_PI + M_PI / 6, q.getAngle());
}


TEST(PathFinderUtils, TestFitPointInsideGridInNegativeRow) {
    /* origin of OccupancyGrid */
    // initialize origin of occupancy grid
    geometry_msgs::Pose origin =
            PathFinderTestUtils::constructPose(3.0, 3.0, 0.0);

    /* mapMetaData of OccupancyGrid */
    // initialize mapMetaData
    nav_msgs::MapMetaData map_meta_data;
    map_meta_data.resolution = 2.0;
    map_meta_data.width      = 4;
    map_meta_data.height     = 5;
    // add origin to mapMetaData
    map_meta_data.origin = origin;

    AStar::GridPoint grid_point = AStar::GridPoint(1, -100);

    PathFinderUtils::fitPointInsideGrid(map_meta_data, grid_point);

    EXPECT_EQ(grid_point.col, 1);
    EXPECT_EQ(grid_point.row, 0);
}

TEST(PathFinderUtils, TestFitPointInsideGridInPositiveRow) {
    /* origin of OccupancyGrid */
    // initialize origin of occupancy grid
    geometry_msgs::Pose origin =
            PathFinderTestUtils::constructPose(3.0, 3.0, 0.0);

    /* mapMetaData of OccupancyGrid */
    // initialize mapMetaData
    nav_msgs::MapMetaData map_meta_data;
    map_meta_data.resolution = 2.0;
    map_meta_data.width      = 4;
    map_meta_data.height     = 5;
    // add origin to mapMetaData
    map_meta_data.origin = origin;

    AStar::GridPoint grid_point = AStar::GridPoint(3, 99999);

    PathFinderUtils::fitPointInsideGrid(map_meta_data, grid_point);

    EXPECT_EQ(grid_point.col, 3);
    EXPECT_EQ(grid_point.row, map_meta_data.height-1);
}

TEST(PathFinderUtils, TestFitPointInsideGridInNegativeCol) {
    /* origin of OccupancyGrid */
    // initialize origin of occupancy grid
    geometry_msgs::Pose origin =
            PathFinderTestUtils::constructPose(3.0, 3.0, 0.0);

    /* mapMetaData of OccupancyGrid */
    // initialize mapMetaData
    nav_msgs::MapMetaData map_meta_data;
    map_meta_data.resolution = 2.0;
    map_meta_data.width      = 4;
    map_meta_data.height     = 5;
    // add origin to mapMetaData
    map_meta_data.origin = origin;

    AStar::GridPoint grid_point = AStar::GridPoint(-99999, 2);

    PathFinderUtils::fitPointInsideGrid(map_meta_data, grid_point);

    EXPECT_EQ(grid_point.col, 0);
    EXPECT_EQ(grid_point.row, 2);
}

TEST(PathFinderUtils, TestFitPointInsideGridInPositiveCol) {
    /* origin of OccupancyGrid */
    // initialize origin of occupancy grid
    geometry_msgs::Pose origin =
            PathFinderTestUtils::constructPose(3.0, 3.0, 0.0);

    /* mapMetaData of OccupancyGrid */
    // initialize mapMetaData
    nav_msgs::MapMetaData map_meta_data;
    map_meta_data.resolution = 2.0;
    map_meta_data.width      = 4;
    map_meta_data.height     = 5;
    // add origin to mapMetaData
    map_meta_data.origin = origin;

    AStar::GridPoint grid_point = AStar::GridPoint(8888, 4);

    PathFinderUtils::fitPointInsideGrid(map_meta_data, grid_point);

    EXPECT_EQ(grid_point.col, map_meta_data.width - 1);
    EXPECT_EQ(grid_point.row, 4);
}

TEST(PathFinderUtils, TestFitPointInsideGridInNegativeColNegativeRow) {
    /* origin of OccupancyGrid */
    // initialize origin of occupancy grid
    geometry_msgs::Pose origin =
            PathFinderTestUtils::constructPose(3.0, 3.0, 0.0);

    /* mapMetaData of OccupancyGrid */
    // initialize mapMetaData
    nav_msgs::MapMetaData map_meta_data;
    map_meta_data.resolution = 2.0;
    map_meta_data.width      = 4;
    map_meta_data.height     = 5;
    // add origin to mapMetaData
    map_meta_data.origin = origin;

    AStar::GridPoint grid_point = AStar::GridPoint(-77777, -77777);

    PathFinderUtils::fitPointInsideGrid(map_meta_data, grid_point);

    EXPECT_EQ(grid_point.col, 0);
    EXPECT_EQ(grid_point.row, 0);
}

TEST(PathFinderUtils, TestFitPointInsideGridInPositiveXPositiveY) {
    /* origin of OccupancyGrid */
    // initialize origin of occupancy grid
    geometry_msgs::Pose origin =
            PathFinderTestUtils::constructPose(3.0, 3.0, 0.0);

    /* mapMetaData of OccupancyGrid */
    // initialize mapMetaData
    nav_msgs::MapMetaData map_meta_data;
    map_meta_data.resolution = 2.0;
    map_meta_data.width      = 4;
    map_meta_data.height     = 5;
    // add origin to mapMetaData
    map_meta_data.origin = origin;

    AStar::GridPoint grid_point = AStar::GridPoint(77777, 77777);

    PathFinderUtils::fitPointInsideGrid(map_meta_data, grid_point);

    EXPECT_EQ(grid_point.col, map_meta_data.width - 1);
    EXPECT_EQ(grid_point.row, map_meta_data.height - 1);
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
