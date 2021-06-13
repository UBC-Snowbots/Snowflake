#ifndef _MOVE_MOTOR
#define _MOVE_MOTOR

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <phidget22.h>
#include <string>

class MoveMotor {
    public:
	MoveMotor(int argc, char **argv, std::string node_name);
    private:
	void callback(const geometry_msgs::Twist::ConstPtr& msg);

	ros::Subscriber my_subscriber;
	ros::Publisher my_publisher;
    PhidgetBLDCMotorHandle bldcMotor0;
    PhidgetReturnCode ret;
    PhidgetReturnCode errorCode;
    PhidgetReturnCode res;
    const char * errorString;
    char errorDetail[100];
    size_t errorDetailLen = 100;
    void close();
};
#endif // _MOVE_MOTOR
