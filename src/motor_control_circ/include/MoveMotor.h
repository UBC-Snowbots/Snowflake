#ifndef _MOVE_MOTOR
#define _MOVE_MOTOR

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <phidget22.h>
#include <string>
#include<vector>

class MoveMotor {
    public:
	MoveMotor(int argc, char **argv, std::string node_name);
    void close();
    private:
	void left_callback(const geometry_msgs::Twist::ConstPtr& msg);
    void right_callback(const geometry_msgs::Twist::ConstPtr& msg);

	ros::Subscriber my_subscriber;
	ros::Publisher my_publisher;
    std::vector<float> weights(number_of_students);
    const int NUM_MOTORS = 6;
    PhidgetBLDCMotorHandle bldcMotor0;
    PhidgetBLDCMotorHandle bldcMotor1;
    PhidgetBLDCMotorHandle bldcMotor2;
    PhidgetBLDCMotorHandle bldcMotor3;
    PhidgetBLDCMotorHandle bldcMotor4;
    PhidgetBLDCMotorHandle bldcMotor5;
    PhidgetBLDCMotorHandle motors [NUM_MOTORS] = { bldcMotor0, bldcMotor1, bldcMotor2,
                                          bldcMotor3, bldcMotor4, bldcMotor5 };
    PhidgetReturnCode ret;
    PhidgetReturnCode errorCode;
    PhidgetReturnCode res;
    const char * errorString;
    char errorDetail[100];
    size_t errorDetailLen = 100;

};
#endif // _MOVE_MOTOR
