/*
 * Created By: Gareth Ellis
 * Created On: April 16, 2017
 * Description: This node is responsible for passing twist messages received
 *              over serial to the arduino controlling the robot
 */

#ifndef DRIVERS_STEERING_DRIVER_H
#define DRIVERS_STEERING_DRIVER_H

// STD
#include <iostream>

// Snowbots
#include <sb_utils.h>

// ROS
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <std_msgs/String.h>

// Other
#include <SerialStream.h>

class SteeringDriver {
public:
    SteeringDriver(int argc, char **argv, std::string node_name);

private:
    void twistCallback(geometry_msgs::Twist::ConstPtr twist_msg);

    ros::Subscriber twist_subscriber;

    // The SerialStream to/from the arduino
    LibSerial::SerialStream arduino;

    // The Port the arduino is connected to
    std::string port;
    // The max ABSOLUTE linear and angular speeds this driver will receive
    // These define the mapping between the twist messages received
    // (in m/s and rad/s), and the speed values sent to the motors by the
    // arduino (integer values between 0 and 255)
    double max_abs_linear_speed, max_abs_angular_speed;
};
#endif //DRIVERS_STEERING_DRIVER_H
