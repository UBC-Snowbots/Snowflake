/*
 * Created By: Min Gyo Kim
 * Created On: May 24th 2018
 * Description: Ros tests for Path Finder Node
 */

#include "PathFinderTestUtils.h"
#include <AStar.h>
#include <geometry_msgs/Point.h>
#include <gtest/gtest.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

/**
 * This is the helper class which will publish and subscribe messages which will
 * test the node being instantiated
 * It contains at the minimum:
 *      publisher - publishes the input to the node
 *      subscriber - publishes the output of the node
 *      callback function - the callback function which corresponds to the
 * subscriber
 *      getter function - to provide a way for gtest to check for equality of
 * the message recieved
 */
class PathFinderRosTest : public testing::Test {
  protected:
    virtual void SetUp() {
        test_goal_publisher = nh_.advertise<geometry_msgs::Point>("/goal", 1);
        test_grid_publisher =
        nh_.advertise<nav_msgs::OccupancyGrid>("/occupancy_grid", 1);
        test_subscriber = nh_.subscribe(
        "/path_finder/path", 1, &PathFinderRosTest::callback, this);

        // Let the publishers and subscribers set itself up timely
        ros::Rate loop_rate(1);
        loop_rate.sleep();
    }

    ros::NodeHandle nh_;
    nav_msgs::Path path;
    ros::Publisher test_goal_publisher;
    ros::Publisher test_grid_publisher;
    ros::Subscriber test_subscriber;

  public:
    void callback(const nav_msgs::Path& incoming_path) { path = incoming_path; }
};

signed char _ = AStar::GRID_FREE;
signed char X = AStar::GRID_OCCUPIED;

// TODO: Uncomment and fix, see issue #339
// TEST_F(PathFinderRosTest, TestPathFinder) {
//    /* origin of OccupancyGrid */
//    // initialize origin of occupancy grid
//    geometry_msgs::Pose origin =
//    PathFinderTestUtils::constructPose(3.0, 7.0, 0.0);
//
//    /* mapMetaData of OccupancyGrid */
//    // initialize mapMetaData
//    nav_msgs::MapMetaData mapMetaData;
//    mapMetaData.resolution = 1.0;
//    mapMetaData.width      = 10;
//    mapMetaData.height     = 9;
//    // add origin to mapMetaData
//    mapMetaData.origin = origin;
//
//    /* OccupancyGrid */
//    // initialize occupancy grid
//    nav_msgs::OccupancyGrid grid;
//
//    // set mapMetaData
//    grid.info = mapMetaData;
//    grid.data = {_, _, _, X, X, X, _, X, X, _, _, X, _, _, _, _, X, _,
//                 _, _, _, X, X, X, X, _, X, X, X, _, _, X, _, _, _, _,
//                 X, _, X, X, _, _, _, X, _, _, _, X, _, X, X, X, _, X,
//                 _, X, X, X, X, _, _, _, _, X, _, _, X, _, X, _, _, _,
//                 _, X, _, _, _, X, _, _, _, X, _, _, _, _, X, _, _, _};
//
//    test_grid_publisher.publish(grid);
//
//    geometry_msgs::Point goal;
//    goal.x = 3.0;
//    goal.y = 15.0;
//
//    test_goal_publisher.publish(goal);
//
//    ros::Rate loop_rate(1);
//
//    // Wait for the message to get passed around
//    loop_rate.sleep();
//
//    // spinOnce allows ros to actually process your callbacks
//    // for the curious:
//    // http://answers.ros.org/question/11887/significance-of-rosspinonce/
//    ros::spinOnce();
//
//    ASSERT_EQ(path.poses.size(), 9);
//
//    std::vector<float> expected_x_in_base_link = {
//    0.0, 0.0, 0.0, 0.0, 1.0, 2.0, 1.0, 0.0, 0.0};
//    std::vector<float> expected_y_in_base_link = {
//    0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0};
//
//    for (int i = 0; i < path.poses.size(); i++) {
//        EXPECT_FLOAT_EQ(expected_x_in_base_link[i] + origin.position.x,
//                        path.poses[i].pose.position.x);
//        EXPECT_FLOAT_EQ(expected_y_in_base_link[i] + origin.position.y,
//                        path.poses[i].pose.position.y);
//    }
//}

// TODO: Uncomment and fix, see issue #339
// TEST_F(PathFinderRosTest, PathFindingWhenGoalNotInGrid) {
//    /* origin of OccupancyGrid */
//    // initialize origin of occupancy grid
//    geometry_msgs::Pose origin =
//    PathFinderTestUtils::constructPose(3.0, 3.0, 0.0);
//
//    /* map_meta_data of OccupancyGrid */
//    // initialize map_meta_data
//    nav_msgs::MapMetaData map_meta_data;
//    map_meta_data.resolution = 2.0;
//    map_meta_data.width      = 2;
//    map_meta_data.height     = 3;
//    // add origin to map_meta_data
//    map_meta_data.origin = origin;
//
//    /* OccupancyGrid */
//    // initialize occupancy grid
//    nav_msgs::OccupancyGrid grid;
//    // set map_meta_data
//    grid.info = map_meta_data;
//    grid.data = {
//    X, X, X, X, _, _,
//    };
//
//    test_grid_publisher.publish(grid);
//
//    /* Goal point in map frame */
//    geometry_msgs::Point goal;
//    goal.x = 3.0;
//    goal.y = -9999;
//
//    test_goal_publisher.publish(goal);
//
//    ros::Rate loop_rate(1);
//
//    // Wait for the message to get passed around
//    loop_rate.sleep();
//
//    // spinOnce allows ros to actually process your callbacks
//    // for the curious:
//    // http://answers.ros.org/question/11887/significance-of-rosspinonce/
//    ros::spinOnce();
//
//    EXPECT_EQ(path.poses.size(), 4);
//
//    std::vector<float> expected_x = {3.0, 1.0, 1.0, 3.0};
//    std::vector<float> expected_y = {7.0, 5.0, 3.0, 1.0};
//    for (int i = 0; i < path.poses.size(); i++) {
//        EXPECT_FLOAT_EQ(expected_x[i], path.poses[i].pose.position.x);
//        EXPECT_FLOAT_EQ(expected_y[i], path.poses[i].pose.position.y);
//    }
//}

int main(int argc, char** argv) {
    // !! Don't forget to initialize ROS, since this is a test within the ros
    // framework !!
    ros::init(argc, argv, "path_finder_rostest");
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
