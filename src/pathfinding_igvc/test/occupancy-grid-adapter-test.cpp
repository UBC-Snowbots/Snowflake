/*
 * Created By: Min Gyo Kim
 * Created On: May 14th 2018
 * Description: Unit tests for occupancy grid conversion service
 */

#include <OccupancyGridAdapter.h>
#include "PathFinderTestUtils.h"
#include <gtest/gtest.h>

TEST(OccupancyGridAdapter, TestIndexOfPointInGrid) {
    /* origin of OccupancyGrid */
    // initialize origin of occupancy grid
    geometry_msgs::Pose origin =
    PathFinderTestUtils::constructPose(3.0, 3.0, 0.0);

    /* mapMetaData of OccupancyGrid */
    // initialize mapMetaData
    nav_msgs::MapMetaData map_meta_data;
    map_meta_data.resolution = 2.0;
    map_meta_data.width      = 2;
    map_meta_data.height     = 2;
    // add origin to mapMetaData
    map_meta_data.origin = origin;

    /* OccupancyGridAdapter */
    OccupancyGridAdapter service =
    OccupancyGridAdapter(map_meta_data);

    geometry_msgs::Point point;
    point.x = 6.2;
    point.y = 3.1;
    point.z = 0.0;

    AStar::GridPoint grid_point = service.convertFromMapToGridPoint(point);

    EXPECT_EQ(grid_point.col, 1);
    EXPECT_EQ(grid_point.row, 0);
}

TEST(OccupancyGridAdapter, TestIndexOfPointOutsideOfGridInNegativeY) {
    /* origin of OccupancyGrid */
    // initialize origin of occupancy grid
    geometry_msgs::Pose origin =
            PathFinderTestUtils::constructPose(3.0, 3.0, 0.0);

    /* mapMetaData of OccupancyGrid */
    // initialize mapMetaData
    nav_msgs::MapMetaData map_meta_data;
    map_meta_data.resolution = 2.0;
    map_meta_data.width      = 2;
    map_meta_data.height     = 2;
    // add origin to mapMetaData
    map_meta_data.origin = origin;

    /* OccupancyGridAdapter */
    OccupancyGridAdapter service =
            OccupancyGridAdapter(map_meta_data);

    geometry_msgs::Point point;
    point.x = 6.2;
    point.y = -100;

    AStar::GridPoint grid_point = service.convertFromMapToGridPoint(point);

    EXPECT_EQ(grid_point.col, 1);
    EXPECT_EQ(grid_point.row, 0);
}

TEST(OccupancyGridAdapter, TestIndexOfPointOutsideOfGridInPositiveY) {
    /* origin of OccupancyGrid */
    // initialize origin of occupancy grid
    geometry_msgs::Pose origin =
            PathFinderTestUtils::constructPose(3.0, 3.0, 0.0);

    /* mapMetaData of OccupancyGrid */
    // initialize mapMetaData
    nav_msgs::MapMetaData map_meta_data;
    map_meta_data.resolution = 2.0;
    map_meta_data.width      = 2;
    map_meta_data.height     = 2;
    // add origin to mapMetaData
    map_meta_data.origin = origin;

    /* OccupancyGridAdapter */
    OccupancyGridAdapter service =
            OccupancyGridAdapter(map_meta_data);

    geometry_msgs::Point point;
    point.x = 6.2;
    point.y = 99999;

    AStar::GridPoint grid_point = service.convertFromMapToGridPoint(point);

    EXPECT_EQ(grid_point.col, 1);
    EXPECT_EQ(grid_point.row, map_meta_data.height-1);
}

TEST(OccupancyGridAdapter, TestIndexOfPointOutsideOfGridInNegativeX) {
    /* origin of OccupancyGrid */
    // initialize origin of occupancy grid
    geometry_msgs::Pose origin =
            PathFinderTestUtils::constructPose(3.0, 3.0, 0.0);

    /* mapMetaData of OccupancyGrid */
    // initialize mapMetaData
    nav_msgs::MapMetaData map_meta_data;
    map_meta_data.resolution = 2.0;
    map_meta_data.width      = 2;
    map_meta_data.height     = 2;
    // add origin to mapMetaData
    map_meta_data.origin = origin;

    /* OccupancyGridAdapter */
    OccupancyGridAdapter service =
            OccupancyGridAdapter(map_meta_data);

    geometry_msgs::Point point;
    point.x = -100;
    point.y = 3.1;

    AStar::GridPoint grid_point = service.convertFromMapToGridPoint(point);

    EXPECT_EQ(grid_point.col, 0);
    EXPECT_EQ(grid_point.row, 0);
}

TEST(OccupancyGridAdapter, TestIndexOfPointOutsideOfGridInPositiveX) {
    /* origin of OccupancyGrid */
    // initialize origin of occupancy grid
    geometry_msgs::Pose origin =
            PathFinderTestUtils::constructPose(3.0, 3.0, 0.0);

    /* mapMetaData of OccupancyGrid */
    // initialize mapMetaData
    nav_msgs::MapMetaData map_meta_data;
    map_meta_data.resolution = 2.0;
    map_meta_data.width      = 2;
    map_meta_data.height     = 2;
    // add origin to mapMetaData
    map_meta_data.origin = origin;

    /* OccupancyGridAdapter */
    OccupancyGridAdapter service =
            OccupancyGridAdapter(map_meta_data);

    geometry_msgs::Point point;
    point.x = 999;
    point.y = 3.1;

    AStar::GridPoint grid_point = service.convertFromMapToGridPoint(point);

    EXPECT_EQ(grid_point.col, map_meta_data.width - 1);
    EXPECT_EQ(grid_point.row, 0);
}

TEST(OccupancyGridAdapter, TestIndexOfPointOutsideOfGridInNegativeXNegativeY) {
    /* origin of OccupancyGrid */
    // initialize origin of occupancy grid
    geometry_msgs::Pose origin =
            PathFinderTestUtils::constructPose(3.0, 3.0, 0.0);

    /* mapMetaData of OccupancyGrid */
    // initialize mapMetaData
    nav_msgs::MapMetaData map_meta_data;
    map_meta_data.resolution = 2.0;
    map_meta_data.width      = 2;
    map_meta_data.height     = 2;
    // add origin to mapMetaData
    map_meta_data.origin = origin;

    /* OccupancyGridAdapter */
    OccupancyGridAdapter service =
            OccupancyGridAdapter(map_meta_data);

    geometry_msgs::Point point;
    point.x = -999;
    point.y = -999;

    AStar::GridPoint grid_point = service.convertFromMapToGridPoint(point);

    EXPECT_EQ(grid_point.col, 0);
    EXPECT_EQ(grid_point.row, 0);
}

TEST(OccupancyGridAdapter, TestIndexOfPointOutsideOfGridInPositiveXPositiveY) {
    /* origin of OccupancyGrid */
    // initialize origin of occupancy grid
    geometry_msgs::Pose origin =
            PathFinderTestUtils::constructPose(3.0, 3.0, 0.0);

    /* mapMetaData of OccupancyGrid */
    // initialize mapMetaData
    nav_msgs::MapMetaData map_meta_data;
    map_meta_data.resolution = 2.0;
    map_meta_data.width      = 2;
    map_meta_data.height     = 2;
    // add origin to mapMetaData
    map_meta_data.origin = origin;

    /* OccupancyGridAdapter */
    OccupancyGridAdapter service =
            OccupancyGridAdapter(map_meta_data);

    geometry_msgs::Point point;
    point.x = 999;
    point.y = 999;

    AStar::GridPoint grid_point = service.convertFromMapToGridPoint(point);

    EXPECT_EQ(grid_point.col, map_meta_data.width - 1);
    EXPECT_EQ(grid_point.row, map_meta_data.height - 1);
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
