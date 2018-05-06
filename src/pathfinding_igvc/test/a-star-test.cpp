//
// Created by min on 05/05/18.
//

#include <AStar.h>
#include <gtest/gtest.h>
#include <tf/transform_datatypes.h>

TEST(AStar, TestChangeOfFrame) {
    AStar a_star = AStar();

    /* origin of OccupancyGrid */
    // initialize origin of occupancy grid
    geometry_msgs::Pose origin;

    // set position of the origin
    geometry_msgs::Point position;
    position.x = 5.0;
    position.y = 5.0;
    position.z = 0.0;
    origin.position = position;

    // set orientation of the origin
    tf::Quaternion q;
    tf::Matrix3x3 rotationMatrix = tf::Matrix3x3();
    rotationMatrix.setEulerYPR(0.0, 0.0, 0.0); // only set Z rotation since it's 2D
    rotationMatrix.getRotation(q);
    tf::quaternionTFToMsg(q, origin.orientation);

    /* mapMetaData of OccupancyGrid */
    // initialize mapMetaData
    nav_msgs::MapMetaData mapMetaData;
    // add origin to mapMetaData
    mapMetaData.origin = origin;

    /* OccupancyGrid */
    // initialize occupancy grid
    nav_msgs::OccupancyGrid grid;
    // set mapMetaData
    grid.info = mapMetaData;

    a_star.setOccupancyGrid(grid);

    geometry_msgs::Point point;
    point.x = 5.0;
    point.y = 6.0;
    point.z = 0.0;

    geometry_msgs::Point point_on_grid = a_star.transformToGridFrame(point);

    EXPECT_FLOAT_EQ(point_on_grid.x, point.x - origin.position.x);
    EXPECT_FLOAT_EQ(point_on_grid.y, point.y - origin.position.y);
}

TEST(AStar, TestChangeOfFrameWithRotation) {
    AStar a_star = AStar();

    /* origin of OccupancyGrid */
    // initialize origin of occupancy grid
    geometry_msgs::Pose origin;

    // set position of the origin
    geometry_msgs::Point position;
    position.x = 5.0;
    position.y = 5.0;
    position.z = 0.0;
    origin.position = position;

    // set orientation of the origin
    tf::Quaternion q;
    tf::Matrix3x3 rotationMatrix = tf::Matrix3x3();
    rotationMatrix.setEulerYPR(M_PI/2, 0.0, 0.0); // only set Z rotation since it's 2D
    rotationMatrix.getRotation(q);
    tf::quaternionTFToMsg(q, origin.orientation);

    /* mapMetaData of OccupancyGrid */
    // initialize mapMetaData
    nav_msgs::MapMetaData mapMetaData;
    // add origin to mapMetaData
    mapMetaData.origin = origin;

    /* OccupancyGrid */
    // initialize occupancy grid
    nav_msgs::OccupancyGrid grid;
    // set mapMetaData
    grid.info = mapMetaData;

    a_star.setOccupancyGrid(grid);

    geometry_msgs::Point point;
    point.x = 2.0;
    point.y = 6.0;
    point.z = 0.0;

    geometry_msgs::Point point_on_grid = a_star.transformToGridFrame(point);

    EXPECT_FLOAT_EQ(point_on_grid.x, 1.0);
    EXPECT_FLOAT_EQ(point_on_grid.y, 3.0);
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}