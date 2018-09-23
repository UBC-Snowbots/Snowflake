/*
 * Created By: Min Gyo Kim
 * Created On: May 14th 2018
 * Description: Unit tests for path constructor
 */

#include "PathFinderTestUtils.h"
#include <PathConstructor.h>
#include <gtest/gtest.h>

// TODO: Uncomment and fix, see issue #339
TEST(PathConstructor, TestConstructPath) {
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
    std::shared_ptr<OccupancyGridAdapter> occupancy_grid_adapter_ptr(
    new OccupancyGridAdapter(map_meta_data));

    /* first point in path*/
    AStar::GridPoint point1(-99 - sqrt(3), -99 - 1);

    /* second point in path*/
    AStar::GridPoint point2(-99, -99);

    /* add points to path */
    std::stack<AStar::GridPoint> points;
    points.push(point1);
    points.push(point2);

    PathConstructor path_constructor(occupancy_grid_adapter_ptr);
    nav_msgs::Path path = path_constructor.constructPath(points);

    tf::Quaternion q1;
    tf::quaternionMsgToTF(path.poses[0].pose.orientation, q1);

    /* we lose resolution by converting a point into a grid, so allow more
    error
     */
    EXPECT_NEAR(q1.getAngle(), M_PI + M_PI / 6, 0.5);
    EXPECT_FLOAT_EQ(
    path.poses[0].pose.position.x,
    occupancy_grid_adapter_ptr->convertFromGridToMapPoint(point2).x);
    EXPECT_FLOAT_EQ(
    path.poses[0].pose.position.y,
    occupancy_grid_adapter_ptr->convertFromGridToMapPoint(point2).y);
    EXPECT_FLOAT_EQ(
    path.poses[1].pose.position.x,
    occupancy_grid_adapter_ptr->convertFromGridToMapPoint(point1).x);
    EXPECT_FLOAT_EQ(
    path.poses[1].pose.position.y,
    occupancy_grid_adapter_ptr->convertFromGridToMapPoint(point1).y);
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
