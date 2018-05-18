/*
 * Created By: Min Gyo Kim
 * Created On: May 14th 2018
 * Description: Unit tests for frame transformation service
 */

#include <FrameTransformer.h>
#include "PathFinderTestUtils.h"
#include <PathFinderUtils.h>
#include <gtest/gtest.h>

TEST(FrameTransformer, TestGetAngleBetweenPoints) {
    /* origin of OccupancyGrid */
    // initialize origin of occupancy grid
    geometry_msgs::Pose origin =
    PathFinderTestUtils::constructPose(5.0, 5.0, 0.0);

    tf::Quaternion rotation;
    tf::Vector3 position;

    // setup transformation matrix from map to grid frame
    // convert geometry_msgs::Point to tf::Vector3
    position = PathFinderUtils::pointToVector(origin.position);
    // convert geometry_msgs::Quaternion to tf::Quaternion
    tf::quaternionMsgToTF(origin.orientation, rotation);

    geometry_msgs::Point point;
    point.x = 5.0;
    point.y = 6.0;
    point.z = 0.0;

    geometry_msgs::Point point_on_grid =
    FrameTransformer(rotation, position)
    .transformFromMapToGridFrame(point);

    EXPECT_FLOAT_EQ(point_on_grid.x, point.x - origin.position.x);
    EXPECT_FLOAT_EQ(point_on_grid.y, point.y - origin.position.y);
}

TEST(FrameTransformer, TestChangeOfFrameWith90Rotation) {
    /* origin of OccupancyGrid */
    // initialize origin of occupancy grid
    geometry_msgs::Pose origin =
    PathFinderTestUtils::constructPose(5.0, 5.0, M_PI / 2);

    tf::Quaternion rotation;
    tf::Vector3 position;

    // setup transformation matrix from map to grid frame
    // convert geometry_msgs::Point to tf::Vector3
    position = PathFinderUtils::pointToVector(origin.position);
    // convert geometry_msgs::Quaternion to tf::Quaternion
    tf::quaternionMsgToTF(origin.orientation, rotation);

    geometry_msgs::Point point;
    point.x = 2.0;
    point.y = 6.0;
    point.z = 0.0;

    geometry_msgs::Point point_on_grid =
    FrameTransformer(rotation, position)
    .transformFromMapToGridFrame(point);

    EXPECT_FLOAT_EQ(point_on_grid.x, 1.0);
    EXPECT_FLOAT_EQ(point_on_grid.y, 3.0);
}

TEST(FrameTransformer, TestChangeOfFrameWith30Rotation) {
    /* origin of OccupancyGrid */
    // initialize origin of occupancy grid
    geometry_msgs::Pose origin =
    PathFinderTestUtils::constructPose(3.0, 3.0, M_PI / 6);

    tf::Quaternion rotation;
    tf::Vector3 position;

    // setup transformation matrix from map to grid frame
    // convert geometry_msgs::Point to tf::Vector3
    position = PathFinderUtils::pointToVector(origin.position);
    // convert geometry_msgs::Quaternion to tf::Quaternion
    tf::quaternionMsgToTF(origin.orientation, rotation);

    geometry_msgs::Point point;
    point.x = 3.0 + sqrt(3.0);
    point.y = 3.0 + 1.0;
    point.z = 0.0;

    geometry_msgs::Point point_on_grid =
    FrameTransformer(rotation, position)
    .transformFromMapToGridFrame(point);

    EXPECT_NEAR(point_on_grid.x, 2.0, 0.01);
    EXPECT_NEAR(point_on_grid.y, 0.0, 0.01);
}

TEST(FrameTransformer, TestChangeOfFrameWith30RotationToMap) {
    /* origin of OccupancyGrid */
    // initialize origin of occupancy grid
    geometry_msgs::Pose origin =
    PathFinderTestUtils::constructPose(3.0, 3.0, M_PI / 6);

    tf::Quaternion rotation;
    tf::Vector3 position;

    // setup transformation matrix from map to grid frame
    // convert geometry_msgs::Point to tf::Vector3
    position = PathFinderUtils::pointToVector(origin.position);
    // convert geometry_msgs::Quaternion to tf::Quaternion
    tf::quaternionMsgToTF(origin.orientation, rotation);

    geometry_msgs::Point point_on_grid;
    point_on_grid.x = 2.0;
    point_on_grid.y = 0.0;

    geometry_msgs::Point point_on_map =
    FrameTransformer(rotation, position)
    .transformFromGridToMapFrame(point_on_grid);

    EXPECT_NEAR(point_on_map.x, 3.0 + sqrt(3.0), 0.01);
    EXPECT_NEAR(point_on_map.y, 3.0 + 1.0, 0.01);
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
