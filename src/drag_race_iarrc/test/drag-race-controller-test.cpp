#include <DragRaceNode.h>
#include <LidarObstacleManager.h>
#include <geometry_msgs/Twist.h>
#include <gtest/gtest.h>

/*
 * Created By: Robyn Castro
 * Created On: July 8, 2017
 * Description: Tests for Drag Race Controller
 */

double more_than_target_distance = 0.0;
double less_than_target_distance = 0.65;

// Velocity limits
double angular_vel_cap = 1.0;
double linear_vel_cap  = 1.0;

// Scaling
double theta_scaling_multiplier = 1.0 / (M_PI * 4);
double angular_speed_multiplier = 1.0;
double linear_speed_multiplier  = 1.0;

// Line values
// Slopes
double right_angle_slope = -3.0;
double left_angle_slope  = 3.0;

// X-Intercepts
double line_to_the_right = -1.0;
double line_to_the_left  = 1.0;

double correlation = 1.0;

// TEST(Position of Line, Heading of the line, Robots Relative position to
// target distance)
TEST(LeftLineTest, angleRightMoreThanTargetDistance) {
    LineOfBestFit testLine =
    LineOfBestFit(right_angle_slope, line_to_the_left, correlation);
    DragRaceController dragRaceController =
    DragRaceController(more_than_target_distance,
                       false,
                       theta_scaling_multiplier,
                       angular_speed_multiplier,
                       linear_speed_multiplier,
                       angular_vel_cap,
                       linear_vel_cap);
    geometry_msgs::Twist testCommand =
    dragRaceController.determineDesiredMotion(testLine, false);
    // Turn Left
    EXPECT_GE(testCommand.angular.z, 0);
}

TEST(LeftLineTest, angleLeftMoreThanTargetDistance) {
    LineOfBestFit testLine =
    LineOfBestFit(left_angle_slope, line_to_the_left, correlation);
    DragRaceController dragRaceController =
    DragRaceController(more_than_target_distance,
                       false,
                       theta_scaling_multiplier,
                       angular_speed_multiplier,
                       linear_speed_multiplier,
                       angular_vel_cap,
                       linear_vel_cap);
    geometry_msgs::Twist testCommand =
    dragRaceController.determineDesiredMotion(testLine, false);
    // Turn Left
    EXPECT_GE(testCommand.angular.z, 0);
}

TEST(LeftLineTest, angleRightLessThanTargetDistance) {
    LineOfBestFit testLine =
    LineOfBestFit(right_angle_slope, line_to_the_left, correlation);
    DragRaceController dragRaceController =
    DragRaceController(less_than_target_distance,
                       false,
                       theta_scaling_multiplier,
                       angular_speed_multiplier,
                       linear_speed_multiplier,
                       angular_vel_cap,
                       linear_vel_cap);
    geometry_msgs::Twist testCommand =
    dragRaceController.determineDesiredMotion(testLine, false);
    // Turn Right
    EXPECT_GE(0, testCommand.angular.z);
}

TEST(LeftLineTest, angleLeftLessThanTargetDistance) {
    LineOfBestFit testLine =
    LineOfBestFit(left_angle_slope, line_to_the_left, correlation);
    DragRaceController dragRaceController =
    DragRaceController(less_than_target_distance,
                       false,
                       theta_scaling_multiplier,
                       angular_speed_multiplier,
                       linear_speed_multiplier,
                       angular_vel_cap,
                       linear_vel_cap);
    geometry_msgs::Twist testCommand =
    dragRaceController.determineDesiredMotion(testLine, false);
    // Turn Right
    EXPECT_GE(0, testCommand.angular.z);
}

TEST(RightLineTest, angleRightMoreThanTargetDistance) {
    LineOfBestFit testLine =
    LineOfBestFit(right_angle_slope, line_to_the_right, correlation);
    DragRaceController dragRaceController =
    DragRaceController(more_than_target_distance,
                       true,
                       theta_scaling_multiplier,
                       angular_speed_multiplier,
                       linear_speed_multiplier,
                       angular_vel_cap,
                       linear_vel_cap);
    geometry_msgs::Twist testCommand =
    dragRaceController.determineDesiredMotion(testLine, false);
    // Turn Right
    EXPECT_GE(0, testCommand.angular.z);
}

TEST(RightLineTest, angleLeftMoreThanTargetDistance) {
    LineOfBestFit testLine =
    LineOfBestFit(left_angle_slope, line_to_the_right, correlation);
    DragRaceController dragRaceController =
    DragRaceController(more_than_target_distance,
                       true,
                       theta_scaling_multiplier,
                       angular_speed_multiplier,
                       linear_speed_multiplier,
                       angular_vel_cap,
                       linear_vel_cap);
    geometry_msgs::Twist testCommand =
    dragRaceController.determineDesiredMotion(testLine, false);
    // Turn Right
    EXPECT_GE(0, testCommand.angular.z);
}

TEST(RightLineTest, angleRightLessThanTargetDistance) {
    LineOfBestFit testLine =
    LineOfBestFit(right_angle_slope, line_to_the_right, correlation);
    DragRaceController dragRaceController =
    DragRaceController(less_than_target_distance,
                       true,
                       theta_scaling_multiplier,
                       angular_speed_multiplier,
                       linear_speed_multiplier,
                       angular_vel_cap,
                       linear_vel_cap);
    geometry_msgs::Twist testCommand =
    dragRaceController.determineDesiredMotion(testLine, false);
    // Turn Left
    EXPECT_GE(testCommand.angular.z, 0);
}

TEST(RightLineTest, angleLeftLessThanTargetDistance) {
    LineOfBestFit testLine =
    LineOfBestFit(left_angle_slope, line_to_the_right, correlation);
    DragRaceController dragRaceController =
    DragRaceController(less_than_target_distance,
                       true,
                       theta_scaling_multiplier,
                       angular_speed_multiplier,
                       linear_speed_multiplier,
                       angular_vel_cap,
                       linear_vel_cap);
    geometry_msgs::Twist testCommand =
    dragRaceController.determineDesiredMotion(testLine, false);
    // Turn Left
    EXPECT_GE(testCommand.angular.z, 0);
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}