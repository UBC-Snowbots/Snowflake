#ifndef _MOVE_MOTOR
#define _MOVE_MOTOR

#include "geometry_msgs/Twist.h"
#include "ros/ros.h"
#include <phidget22.h>
#include <string>
#include <vector>

class MoveMotor {
  public:
    MoveMotor(int argc, char** argv, std::string node_name);
    void close();

  private:
    void left_callback(const geometry_msgs::Twist::ConstPtr& msg);
    void right_callback(const geometry_msgs::Twist::ConstPtr& msg);
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
