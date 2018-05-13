/*
 * Created By: Min Gyo Kim
 * Created On: May 5th 2018
 * Description: Test class that tests various functions of PathFinder
 */

#include <gtest/gtest.h>
#include <tf/transform_datatypes.h>
#include <PathFinder.h>
#include <PathFinderTestUtils.h>

TEST(PathFinder, TestGetAngleBetweenPoints) {
    geometry_msgs::Point from;
    from.x = -99.0;
    from.y = -99.0;
    from.z = 0.0;

    geometry_msgs::Point to;
    to.x = from.x - sqrt(3);
    to.y = from.y - 1.0;
    to.z = 0.0;

    double angle = PathFinder::getAngleBetweenPoints(from, to);
    tf::Quaternion q = PathFinder::getQuaternionFromAngle(angle);
    EXPECT_FLOAT_EQ(M_PI + M_PI/6, q.getAngle());
}

TEST(PathFinder, TestConstructPath) {
    PathFinder path_finder = PathFinder();

    /* origin of OccupancyGrid */
    // initialize origin of occupancy grid
    geometry_msgs::Pose origin = PathFinderTestUtils::constructPose(3.0, 3.0, 0.0);

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

    path_finder.setOccupancyGrid(grid);

    /* first point in path*/
    AStar::GridPoint point1(-99 - sqrt(3), -99 - 1);

    /* second point in path*/
    AStar::GridPoint point2(-99, -99);

    /* add points to path */
    std::stack<AStar::GridPoint> points;
    points.push(point1);
    points.push(point2);

    nav_msgs::Path path = path_finder.constructPath(points);

    tf::Quaternion q1;
    tf::quaternionMsgToTF(path.poses[0].pose.orientation, q1);

    /* we lose resolution by converting a point into a grid, so allow more error */
    EXPECT_NEAR(q1.getAngle(), M_PI + M_PI/6, 0.5);
}

TEST(PathFinder, TestChangeOfFrame) {
    PathFinder path_finder = PathFinder();

    /* origin of OccupancyGrid */
    // initialize origin of occupancy grid
    geometry_msgs::Pose origin = PathFinderTestUtils::constructPose(5.0, 5.0, 0.0);

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

    path_finder.setOccupancyGrid(grid);

    geometry_msgs::Point point;
    point.x = 5.0;
    point.y = 6.0;
    point.z = 0.0;

    geometry_msgs::Point point_on_grid = path_finder.transformToGridFrame(point);

    EXPECT_FLOAT_EQ(point_on_grid.x, point.x - origin.position.x);
    EXPECT_FLOAT_EQ(point_on_grid.y, point.y - origin.position.y);
}

TEST(PathFinder, TestChangeOfFrameWith90Rotation) {
    PathFinder path_finder = PathFinder();

    /* origin of OccupancyGrid */
    // initialize origin of occupancy grid
    geometry_msgs::Pose origin = PathFinderTestUtils::constructPose(5.0, 5.0, M_PI/2);

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

    path_finder.setOccupancyGrid(grid);

    geometry_msgs::Point point;
    point.x = 2.0;
    point.y = 6.0;
    point.z = 0.0;

    geometry_msgs::Point point_on_grid = path_finder.transformToGridFrame(point);

    EXPECT_FLOAT_EQ(point_on_grid.x, 1.0);
    EXPECT_FLOAT_EQ(point_on_grid.y, 3.0);
}

TEST(PathFinder, TestChangeOfFrameWith30Rotation) {
    PathFinder path_finder = PathFinder();

    /* origin of OccupancyGrid */
    // initialize origin of occupancy grid
    geometry_msgs::Pose origin = PathFinderTestUtils::constructPose(3.0, 3.0, M_PI/6);

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

    path_finder.setOccupancyGrid(grid);

    geometry_msgs::Point point;
    point.x = 3.0 + sqrt(3.0);
    point.y = 3.0 + 1.0;
    point.z = 0.0;

    geometry_msgs::Point point_on_grid = path_finder.transformToGridFrame(point);

    EXPECT_NEAR(point_on_grid.x, 2.0, 0.01);
    EXPECT_NEAR(point_on_grid.y, 0.0, 0.01);
}

TEST(PathFinder, TestChangeOfFrameWith30RotationToMap) {
    PathFinder path_finder = PathFinder();

    /* origin of OccupancyGrid */
    // initialize origin of occupancy grid
    geometry_msgs::Pose origin = PathFinderTestUtils::constructPose(3.0, 3.0, M_PI/6);

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

    path_finder.setOccupancyGrid(grid);

    geometry_msgs::Point point_on_grid;
    point_on_grid.x = 2.0;
    point_on_grid.y = 0.0;

    geometry_msgs::Point point_on_map = path_finder.transformToMapFrame(point_on_grid);

    EXPECT_NEAR(point_on_map.x, 3.0 + sqrt(3.0), 0.01);
    EXPECT_NEAR(point_on_map.y, 3.0 + 1.0, 0.01);
}


TEST(PathFinder, TestIndexOfPointInGrid) {
    PathFinder path_finder = PathFinder();

    /* origin of OccupancyGrid */
    // initialize origin of occupancy grid
    geometry_msgs::Pose origin = PathFinderTestUtils::constructPose(3.0, 3.0, 0.0);

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

    path_finder.setOccupancyGrid(grid);

    geometry_msgs::Point point;
    point.x = 6.2;
    point.y = 3.1;
    point.z = 0.0;

    AStar::GridPoint grid_point = path_finder.convertToGridPoint(point);

    EXPECT_EQ(grid_point.col, 1);
    EXPECT_EQ(grid_point.row, 0);
}

TEST(PathFinder, TestResizeMapExpandRight) {
    PathFinder path_finder = PathFinder();

    /* origin of OccupancyGrid */
    // initialize origin of occupancy grid
    geometry_msgs::Pose origin = PathFinderTestUtils::constructPose(3.0, 3.0, 0.0);

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
    grid.data = std::vector<int8_t>(6, GRID_OCCUPIED);

    path_finder.setOccupancyGrid(grid);

    geometry_msgs::Point point;
    point.x = 8.0;
    point.y = 3.0;
    point.z = 0.0;

    AStar::GridPoint grid_point = path_finder.convertToGridPoint(point);

    EXPECT_NEAR(grid_point.col, 2.0, 0.01);
    EXPECT_NEAR(grid_point.row, 0.0, 0.01);

    path_finder.resizeMapToFitGoal(grid_point);

    EXPECT_EQ(path_finder._occupancy_grid.data.size(), 9);
    EXPECT_EQ(path_finder._occupancy_grid.info.width, 3);
    EXPECT_EQ(path_finder._occupancy_grid.info.height, 3);
    EXPECT_EQ(path_finder._occupancy_grid.info.resolution, 2.0);
    EXPECT_EQ(path_finder._occupancy_grid.info.origin.position.x, origin.position.x);
    EXPECT_EQ(path_finder._occupancy_grid.info.origin.position.y, origin.position.y);

    std::vector<int8_t> expected_data = {
            GRID_OCCUPIED, GRID_OCCUPIED, GRID_FREE,
            GRID_OCCUPIED, GRID_OCCUPIED, GRID_FREE,
            GRID_OCCUPIED, GRID_OCCUPIED, GRID_FREE
    };
    EXPECT_EQ(path_finder._occupancy_grid.data, expected_data);
}

TEST(PathFinder, TestResizeMapExpandLeft) {
    PathFinder path_finder = PathFinder();

    /* origin of OccupancyGrid */
    // initialize origin of occupancy grid
    geometry_msgs::Pose origin = PathFinderTestUtils::constructPose(3.0, 3.0, 0.0);

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
    grid.data = std::vector<int8_t>(6, GRID_OCCUPIED);

    path_finder.setOccupancyGrid(grid);

    geometry_msgs::Point point;
    point.x = 0.0;
    point.y = 3.0;
    point.z = 0.0;

    AStar::GridPoint grid_point = path_finder.convertToGridPoint(point);

    ASSERT_NEAR(grid_point.col, -2.0, 0.01);
    ASSERT_NEAR(grid_point.row, 0.0, 0.01);

    path_finder.resizeMapToFitGoal(grid_point);

    EXPECT_EQ(path_finder._occupancy_grid.data.size(), 12);
    EXPECT_EQ(path_finder._occupancy_grid.info.width, 4);
    EXPECT_EQ(path_finder._occupancy_grid.info.height, 3);
    EXPECT_EQ(path_finder._occupancy_grid.info.resolution, 2.0);
    EXPECT_EQ(path_finder._occupancy_grid.info.origin.position.x, -1.0);
    EXPECT_EQ(path_finder._occupancy_grid.info.origin.position.y, 3.0);

    std::vector<int8_t> expected_data = {
            GRID_FREE, GRID_FREE, GRID_OCCUPIED, GRID_OCCUPIED,
            GRID_FREE, GRID_FREE, GRID_OCCUPIED, GRID_OCCUPIED,
            GRID_FREE, GRID_FREE, GRID_OCCUPIED, GRID_OCCUPIED
    };
    EXPECT_EQ(path_finder._occupancy_grid.data, expected_data);
}

TEST(PathFinder, TestResizeMapExpandUp) {
    PathFinder path_finder = PathFinder();

    /* origin of OccupancyGrid */
    // initialize origin of occupancy grid
    geometry_msgs::Pose origin = PathFinderTestUtils::constructPose(3.0, 3.0, 0.0);

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
    grid.data = std::vector<int8_t>(6, GRID_OCCUPIED);

    path_finder.setOccupancyGrid(grid);

    geometry_msgs::Point point;
    point.x = 3.0;
    point.y = 12.0;
    point.z = 0.0;

    AStar::GridPoint grid_point = path_finder.convertToGridPoint(point);

    EXPECT_NEAR(grid_point.col, 0.0, 0.01);
    EXPECT_NEAR(grid_point.row, 4.0, 0.01);

    path_finder.resizeMapToFitGoal(grid_point);

    EXPECT_EQ(path_finder._occupancy_grid.data.size(), 10);
    EXPECT_EQ(path_finder._occupancy_grid.info.width, 2);
    EXPECT_EQ(path_finder._occupancy_grid.info.height, 5);
    EXPECT_EQ(path_finder._occupancy_grid.info.resolution, 2.0);
    EXPECT_EQ(path_finder._occupancy_grid.info.origin.position.x, origin.position.x);
    EXPECT_EQ(path_finder._occupancy_grid.info.origin.position.y, origin.position.y);

    std::vector<int8_t> expected_data = {
            GRID_OCCUPIED, GRID_OCCUPIED,
            GRID_OCCUPIED, GRID_OCCUPIED,
            GRID_OCCUPIED, GRID_OCCUPIED,
            GRID_FREE, GRID_FREE,
            GRID_FREE, GRID_FREE,
    };

    EXPECT_EQ(path_finder._occupancy_grid.data, expected_data);
}

TEST(PathFinder, TestResizeMapExpandDown) {
    PathFinder path_finder = PathFinder();

    /* origin of OccupancyGrid */
    // initialize origin of occupancy grid
    geometry_msgs::Pose origin = PathFinderTestUtils::constructPose(3.0, 3.0, 0.0);

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
    grid.data = std::vector<int8_t>(6, GRID_OCCUPIED);

    path_finder.setOccupancyGrid(grid);

    geometry_msgs::Point point;
    point.x = 3.0;
    point.y = 0.0;
    point.z = 0.0;

    AStar::GridPoint grid_point = path_finder.convertToGridPoint(point);

    EXPECT_NEAR(grid_point.col, 0.0, 0.01);
    EXPECT_NEAR(grid_point.row, -2.0, 0.01);

    path_finder.resizeMapToFitGoal(grid_point);

    EXPECT_EQ(path_finder._occupancy_grid.data.size(), 10);
    EXPECT_EQ(path_finder._occupancy_grid.info.width, 2);
    EXPECT_EQ(path_finder._occupancy_grid.info.height, 5);
    EXPECT_EQ(path_finder._occupancy_grid.info.resolution, 2.0);
    EXPECT_EQ(path_finder._occupancy_grid.info.origin.position.x, 3.0);
    EXPECT_EQ(path_finder._occupancy_grid.info.origin.position.y, -1.0);

    std::vector<int8_t> expected_data = {
            GRID_FREE, GRID_FREE,
            GRID_FREE, GRID_FREE,
            GRID_OCCUPIED, GRID_OCCUPIED,
            GRID_OCCUPIED, GRID_OCCUPIED,
            GRID_OCCUPIED, GRID_OCCUPIED,
    };

    EXPECT_EQ(path_finder._occupancy_grid.data, expected_data);
}

TEST(PathFinder, TestResizeMapExpandLeftAndDown) {
    PathFinder path_finder = PathFinder();

    /* origin of OccupancyGrid */
    // initialize origin of occupancy grid
    geometry_msgs::Pose origin = PathFinderTestUtils::constructPose(3.0, 3.0, 0.0);

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
    grid.data = std::vector<int8_t>(6, GRID_OCCUPIED);

    path_finder.setOccupancyGrid(grid);

    geometry_msgs::Point point;
    point.x = 0.0;
    point.y = 0.0;
    point.z = 0.0;

    AStar::GridPoint grid_point = path_finder.convertToGridPoint(point);

    EXPECT_NEAR(grid_point.col, -2.0, 0.01);
    EXPECT_NEAR(grid_point.row, -2.0, 0.01);

    path_finder.resizeMapToFitGoal(grid_point);

    EXPECT_EQ(path_finder._occupancy_grid.data.size(), 20);
    EXPECT_EQ(path_finder._occupancy_grid.info.width, 4);
    EXPECT_EQ(path_finder._occupancy_grid.info.height, 5);
    EXPECT_EQ(path_finder._occupancy_grid.info.resolution, 2.0);
    EXPECT_EQ(path_finder._occupancy_grid.info.origin.position.x, -1.0);
    EXPECT_EQ(path_finder._occupancy_grid.info.origin.position.y, -1.0);

    std::vector<int8_t> expected_data = {
            GRID_FREE, GRID_FREE, GRID_FREE, GRID_FREE,
            GRID_FREE, GRID_FREE, GRID_FREE, GRID_FREE,
            GRID_FREE, GRID_FREE, GRID_OCCUPIED, GRID_OCCUPIED,
            GRID_FREE, GRID_FREE, GRID_OCCUPIED, GRID_OCCUPIED,
            GRID_FREE, GRID_FREE, GRID_OCCUPIED, GRID_OCCUPIED,
    };

    EXPECT_EQ(path_finder._occupancy_grid.data, expected_data);
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}