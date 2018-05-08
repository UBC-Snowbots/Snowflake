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

TEST(AStar, TestChangeOfFrameWith90Rotation) {
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

TEST(AStar, TestChangeOfFrameWith30Rotation) {
    AStar a_star = AStar();

    /* origin of OccupancyGrid */
    // initialize origin of occupancy grid
    geometry_msgs::Pose origin;

    // set position of the origin
    geometry_msgs::Point position;
    position.x = 3.0;
    position.y = 3.0;
    position.z = 0.0;
    origin.position = position;

    // set orientation of the origin
    tf::Quaternion q;
    tf::Matrix3x3 rotationMatrix = tf::Matrix3x3();
    rotationMatrix.setEulerYPR(M_PI/6, 0.0, 0.0); // only set Z rotation since it's 2D
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
    point.x = 3.0 + sqrt(3.0);
    point.y = 3.0 + 1.0;
    point.z = 0.0;

    geometry_msgs::Point point_on_grid = a_star.transformToGridFrame(point);

    EXPECT_NEAR(point_on_grid.x, 2.0, 0.01);
    EXPECT_NEAR(point_on_grid.y, 0.0, 0.01);
}

TEST(AStar, TestIndexOfPointInGrid) {
    AStar a_star = AStar();

    /* origin of OccupancyGrid */
    // initialize origin of occupancy grid
    geometry_msgs::Pose origin;

    // set position of the origin
    geometry_msgs::Point position;
    position.x = 3.0;
    position.y = 3.0;
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
    mapMetaData.resolution = 2.0;
    mapMetaData.width = 2;
    mapMetaData.height = 2;
    // add origin to mapMetaData
    mapMetaData.origin = origin;

    /* OccupancyGrid */
    // initialize occupancy grid
    nav_msgs::OccupancyGrid grid;
    // set mapMetaData
    grid.info = mapMetaData;

    a_star.setOccupancyGrid(grid);

    geometry_msgs::Point point;
    point.x = 6.2;
    point.y = 3.1;
    point.z = 0.0;

    AStar::GridPoint grid_point = a_star.convertToGridPoint(point);

    EXPECT_EQ(grid_point.col, 1);
    EXPECT_EQ(grid_point.row, 0);
}

TEST(AStar, TestResizeMapExpandRight) {
    AStar a_star = AStar();

    /* origin of OccupancyGrid */
    // initialize origin of occupancy grid
    geometry_msgs::Pose origin;

    // set position of the origin
    geometry_msgs::Point position;
    position.x = 3.0;
    position.y = 3.0;
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
    mapMetaData.resolution = 2.0;
    mapMetaData.width = 2;
    mapMetaData.height = 3;
    // add origin to mapMetaData
    mapMetaData.origin = origin;

    /* OccupancyGrid */
    // initialize occupancy grid
    nav_msgs::OccupancyGrid grid;
    // set mapMetaData
    grid.info = mapMetaData;
    grid.data = std::vector<int8_t>(6, OCC_GRID_OCCUPIED);

    a_star.setOccupancyGrid(grid);

    geometry_msgs::Point point;
    point.x = 8.0;
    point.y = 3.0;
    point.z = 0.0;

    AStar::GridPoint grid_point = a_star.convertToGridPoint(point);

    EXPECT_NEAR(grid_point.col, 2.0, 0.01);
    EXPECT_NEAR(grid_point.row, 0.0, 0.01);

    a_star.resizeMapToFitGoal(grid_point);

    EXPECT_EQ(a_star._occupancy_grid.data.size(), 9);
    EXPECT_EQ(a_star._occupancy_grid.info.width, 3);
    EXPECT_EQ(a_star._occupancy_grid.info.height, 3);
    EXPECT_EQ(a_star._occupancy_grid.info.resolution, 2.0);
    EXPECT_EQ(a_star._occupancy_grid.info.origin.position.x, position.x);
    EXPECT_EQ(a_star._occupancy_grid.info.origin.position.y, position.y);

    std::vector<int8_t> expected_data = {
            OCC_GRID_OCCUPIED, OCC_GRID_OCCUPIED, OCC_GRID_FREE,
            OCC_GRID_OCCUPIED, OCC_GRID_OCCUPIED, OCC_GRID_FREE,
            OCC_GRID_OCCUPIED, OCC_GRID_OCCUPIED, OCC_GRID_FREE
    };
    EXPECT_EQ(a_star._occupancy_grid.data, expected_data);
}

TEST(AStar, TestResizeMapExpandLeft) {
    AStar a_star = AStar();

    /* origin of OccupancyGrid */
    // initialize origin of occupancy grid
    geometry_msgs::Pose origin;

    // set position of the origin
    geometry_msgs::Point position;
    position.x = 3.0;
    position.y = 3.0;
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
    mapMetaData.resolution = 2.0;
    mapMetaData.width = 2;
    mapMetaData.height = 3;
    // add origin to mapMetaData
    mapMetaData.origin = origin;

    /* OccupancyGrid */
    // initialize occupancy grid
    nav_msgs::OccupancyGrid grid;
    // set mapMetaData
    grid.info = mapMetaData;
    grid.data = std::vector<int8_t>(6, OCC_GRID_OCCUPIED);

    a_star.setOccupancyGrid(grid);

    geometry_msgs::Point point;
    point.x = 0.0;
    point.y = 3.0;
    point.z = 0.0;

    AStar::GridPoint grid_point = a_star.convertToGridPoint(point);

    ASSERT_NEAR(grid_point.col, -2.0, 0.01);
    ASSERT_NEAR(grid_point.row, 0.0, 0.01);

    a_star.resizeMapToFitGoal(grid_point);

    EXPECT_EQ(a_star._occupancy_grid.data.size(), 12);
    EXPECT_EQ(a_star._occupancy_grid.info.width, 4);
    EXPECT_EQ(a_star._occupancy_grid.info.height, 3);
    EXPECT_EQ(a_star._occupancy_grid.info.resolution, 2.0);
    EXPECT_EQ(a_star._occupancy_grid.info.origin.position.x, -1.0);
    EXPECT_EQ(a_star._occupancy_grid.info.origin.position.y, 3.0);

    std::vector<int8_t> expected_data = {
            OCC_GRID_FREE, OCC_GRID_FREE, OCC_GRID_OCCUPIED, OCC_GRID_OCCUPIED,
            OCC_GRID_FREE, OCC_GRID_FREE, OCC_GRID_OCCUPIED, OCC_GRID_OCCUPIED,
            OCC_GRID_FREE, OCC_GRID_FREE, OCC_GRID_OCCUPIED, OCC_GRID_OCCUPIED
    };
    EXPECT_EQ(a_star._occupancy_grid.data, expected_data);
}

TEST(AStar, TestResizeMapExpandUp) {
    AStar a_star = AStar();

    /* origin of OccupancyGrid */
    // initialize origin of occupancy grid
    geometry_msgs::Pose origin;

    // set position of the origin
    geometry_msgs::Point position;
    position.x = 3.0;
    position.y = 3.0;
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
    mapMetaData.resolution = 2.0;
    mapMetaData.width = 2;
    mapMetaData.height = 3;
    // add origin to mapMetaData
    mapMetaData.origin = origin;

    /* OccupancyGrid */
    // initialize occupancy grid
    nav_msgs::OccupancyGrid grid;
    // set mapMetaData
    grid.info = mapMetaData;
    grid.data = std::vector<int8_t>(6, OCC_GRID_OCCUPIED);

    a_star.setOccupancyGrid(grid);

    geometry_msgs::Point point;
    point.x = 3.0;
    point.y = 12.0;
    point.z = 0.0;

    AStar::GridPoint grid_point = a_star.convertToGridPoint(point);

    EXPECT_NEAR(grid_point.col, 0.0, 0.01);
    EXPECT_NEAR(grid_point.row, 4.0, 0.01);

    a_star.resizeMapToFitGoal(grid_point);

    EXPECT_EQ(a_star._occupancy_grid.data.size(), 10);
    EXPECT_EQ(a_star._occupancy_grid.info.width, 2);
    EXPECT_EQ(a_star._occupancy_grid.info.height, 5);
    EXPECT_EQ(a_star._occupancy_grid.info.resolution, 2.0);
    EXPECT_EQ(a_star._occupancy_grid.info.origin.position.x, position.x);
    EXPECT_EQ(a_star._occupancy_grid.info.origin.position.y, position.y);

    std::vector<int8_t> expected_data = {
            OCC_GRID_OCCUPIED, OCC_GRID_OCCUPIED,
            OCC_GRID_OCCUPIED, OCC_GRID_OCCUPIED,
            OCC_GRID_OCCUPIED, OCC_GRID_OCCUPIED,
            OCC_GRID_FREE, OCC_GRID_FREE,
            OCC_GRID_FREE, OCC_GRID_FREE,
    };

    EXPECT_EQ(a_star._occupancy_grid.data, expected_data);
}

TEST(AStar, TestResizeMapExpandDown) {
    AStar a_star = AStar();

    /* origin of OccupancyGrid */
    // initialize origin of occupancy grid
    geometry_msgs::Pose origin;

    // set position of the origin
    geometry_msgs::Point position;
    position.x = 3.0;
    position.y = 3.0;
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
    mapMetaData.resolution = 2.0;
    mapMetaData.width = 2;
    mapMetaData.height = 3;
    // add origin to mapMetaData
    mapMetaData.origin = origin;

    /* OccupancyGrid */
    // initialize occupancy grid
    nav_msgs::OccupancyGrid grid;
    // set mapMetaData
    grid.info = mapMetaData;
    grid.data = std::vector<int8_t>(6, OCC_GRID_OCCUPIED);

    a_star.setOccupancyGrid(grid);

    geometry_msgs::Point point;
    point.x = 3.0;
    point.y = 0.0;
    point.z = 0.0;

    AStar::GridPoint grid_point = a_star.convertToGridPoint(point);

    EXPECT_NEAR(grid_point.col, 0.0, 0.01);
    EXPECT_NEAR(grid_point.row, -2.0, 0.01);

    a_star.resizeMapToFitGoal(grid_point);

    EXPECT_EQ(a_star._occupancy_grid.data.size(), 10);
    EXPECT_EQ(a_star._occupancy_grid.info.width, 2);
    EXPECT_EQ(a_star._occupancy_grid.info.height, 5);
    EXPECT_EQ(a_star._occupancy_grid.info.resolution, 2.0);
    EXPECT_EQ(a_star._occupancy_grid.info.origin.position.x, 3.0);
    EXPECT_EQ(a_star._occupancy_grid.info.origin.position.y, -1.0);

    std::vector<int8_t> expected_data = {
            OCC_GRID_FREE, OCC_GRID_FREE,
            OCC_GRID_FREE, OCC_GRID_FREE,
            OCC_GRID_OCCUPIED, OCC_GRID_OCCUPIED,
            OCC_GRID_OCCUPIED, OCC_GRID_OCCUPIED,
            OCC_GRID_OCCUPIED, OCC_GRID_OCCUPIED,
    };

    EXPECT_EQ(a_star._occupancy_grid.data, expected_data);
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}