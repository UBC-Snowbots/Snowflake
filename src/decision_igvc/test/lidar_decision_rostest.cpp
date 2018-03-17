/*
 * Created By: Gareth Ellis
 * Created On: February 4th, 2016
 * Description: Integration testing for the lidar_decision node
 */

#include <LidarDecision.h>
#include <gtest/gtest.h>
#include <ros/ros.h>

class LidarDecisionTest : public testing::Test {
  protected:
    virtual void SetUp() {
        laser_scan_publisher =
        nh_.advertise<sensor_msgs::LaserScan>("/robot/laser/scan", 1);
        twist_subscriber = nh_.subscribe(
        "/lidar_decision/command", 1, &LidarDecisionTest::callback, this);

        // Create a fake laserscan
        ulong num_rays      = 360;
        test_scan.angle_min = (float) (-M_PI / 2);
        test_scan.angle_max = (float) (M_PI / 2);
        test_scan.angle_increment =
        (test_scan.angle_max - test_scan.angle_min) / num_rays;
        // Set all the ranges to 0 initially
        test_scan.ranges    = std::vector<float>(num_rays, 0);
        test_scan.range_min = 2;
        test_scan.range_max = 40;

        ros::spinOnce();
        ros::Rate loop_rate(1);
        loop_rate.sleep();
    }

    ros::NodeHandle nh_;
    ros::Publisher laser_scan_publisher;
    ros::Subscriber twist_subscriber;

    geometry_msgs::Twist command;

    sensor_msgs::LaserScan test_scan;

  public:
    void callback(const geometry_msgs::Twist::ConstPtr msg) { command = *msg; }
};

TEST_F(LidarDecisionTest, oneObstacleStraightAheadTest) {
    // Add a large obstacle directly in front
    std::fill(
    test_scan.ranges.begin() + 140, test_scan.ranges.begin() + 220, 3);

    laser_scan_publisher.publish(test_scan);

    ros::Rate loop_rate(1);
    loop_rate.sleep();
    ros::spinOnce();

    // With the given laserscan, we would want to be turning
    EXPECT_GE(abs(command.angular.z), 0.00001);

    // We would also want to slow down
    EXPECT_NEAR(1.7320508, command.linear.x, 0.000001);

    // Everything else should always be 0
    EXPECT_EQ(0, command.linear.y);
    EXPECT_EQ(0, command.linear.z);
    EXPECT_EQ(0, command.angular.x);
    EXPECT_EQ(0, command.angular.y);
}

TEST_F(LidarDecisionTest, noObstacles) {
    laser_scan_publisher.publish(test_scan);

    ros::Rate loop_rate(1);
    loop_rate.sleep();
    ros::spinOnce();

    // With the given laserscan, we would expect no command,
    // as there are no obstacles to base a command off of
    EXPECT_EQ(0, command.linear.x);
    EXPECT_EQ(0, command.linear.y);
    EXPECT_EQ(0, command.linear.z);
    EXPECT_EQ(0, command.angular.x);
    EXPECT_EQ(0, command.angular.y);
    EXPECT_EQ(0, command.angular.z);
}

TEST_F(LidarDecisionTest, obstacleToRight) {
    // Add a large obstacle to the right of the robot
    std::fill(
    test_scan.ranges.begin() + 100, test_scan.ranges.begin() + 140, 3);

    laser_scan_publisher.publish(test_scan);

    ros::Rate loop_rate(1);
    loop_rate.sleep();
    ros::spinOnce();

    // With the given laserscan, we would want to be turning left
    EXPECT_GE(command.angular.z, 0.0001);

    // We would also want to slow down
    EXPECT_NEAR(1.7320508, command.linear.x, 0.000001);

    // Everything else should always be 0
    EXPECT_EQ(0, command.linear.y);
    EXPECT_EQ(0, command.linear.z);
    EXPECT_EQ(0, command.angular.x);
    EXPECT_EQ(0, command.angular.y);
}

TEST_F(LidarDecisionTest, obstacleToLeft) {
    // Add a large obstacle to the left of the robot
    std::fill(
    test_scan.ranges.begin() + 220, test_scan.ranges.begin() + 260, 3);

    laser_scan_publisher.publish(test_scan);

    ros::Rate loop_rate(1);
    loop_rate.sleep();
    ros::spinOnce();

    // With the given laserscan, we would want to be turning right
    EXPECT_LE(command.angular.z, -0.0001);

    // We would also want to slow down
    EXPECT_NEAR(1.7320508, command.linear.x, 0.000001);

    // Everything else should always be 0
    EXPECT_EQ(0, command.linear.y);
    EXPECT_EQ(0, command.linear.z);
    EXPECT_EQ(0, command.angular.x);
    EXPECT_EQ(0, command.angular.y);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "lidar_decision_rostest");
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
