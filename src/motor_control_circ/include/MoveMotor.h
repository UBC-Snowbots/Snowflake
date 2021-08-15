/*
 * Created By: Ihsan Olawale, Kevin Lin
 * Created On: August 15th, 2021
 * Description: Header file for MoveMotor.cpp and motor_control node
 *              Sets up Phidgets objects, arrays containing the motors,
 *              the left and right motors, ROS topics and function prototypes
 *              See master documentation for more information.
 */
#ifndef _MOVE_MOTOR
#define _MOVE_MOTOR

#include "geometry_msgs/Twist.h"
#include "ros/ros.h"
#include <phidget22.h>
#include <string>
#include <vector>

class MoveMotor {
  public:
    /**
     * Main initiation function for motor control node
     *
     * @param node_name the name of the node
     */
    MoveMotor(int argc, char** argv, std::string node_name);
    /**
     * Function to close all Phidgets motors
     */
    void close();

  private:
    /**
     * Callback function for when a message intended for the left motors is
     * received
     *
     * @param msg the Twist message received in the callback
     */
    void left_callback(const geometry_msgs::Twist::ConstPtr& msg);
    /**
     * Callback function for when a message intended for the right motors is
     * received
     *
     * @param msg the Twist message received in the callback
     */
    void right_callback(const geometry_msgs::Twist::ConstPtr& msg);

    /**
     * Helper functions to run multiple motors on the same side at once with
     * the same velocity
     *
     * @param velocity the desired speed, which will be passed from a callback
     */
    void run_motors(float velocity);

    ros::Subscriber left_subscriber;
    ros::Subscriber right_subscriber;
    ros::Publisher my_publisher;
    const int static NUM_MOTORS = 6;
    PhidgetBLDCMotorHandle bldcMotor0;
    PhidgetBLDCMotorHandle bldcMotor1;
    PhidgetBLDCMotorHandle bldcMotor2;
    PhidgetBLDCMotorHandle bldcMotor3;
    PhidgetBLDCMotorHandle bldcMotor4;
    PhidgetBLDCMotorHandle bldcMotor5;
    std::vector<PhidgetBLDCMotorHandle> motors{
    bldcMotor0, bldcMotor1, bldcMotor2, bldcMotor3, bldcMotor4, bldcMotor5};
    std::vector<int> right_motors{0, 1, 2};
    std::vector<int> left_motors{3, 4, 5};
    std::vector<int> current_motors = right_motors;
    PhidgetReturnCode ret;
    PhidgetReturnCode errorCode;
    PhidgetReturnCode res;
    const char* errorString;
    char errorDetail[100];
    size_t errorDetailLen = 100;
};
#endif // _MOVE_MOTOR
