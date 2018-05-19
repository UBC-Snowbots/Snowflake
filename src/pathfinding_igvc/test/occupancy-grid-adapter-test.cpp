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

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
