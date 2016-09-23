/*
 * Created By: YOUR NAME HERE
 * Created On: September 22, 2016
 * Description: This node is responsible for passing received twist messages over
 *              serial to the arduino controlling the robot
 *
 */

#ifndef DRIVERS_STEERING_DRIVER_H
#define DRIVERS_STEERING_DRIVER_H

#include <iostream>
#include <std_msgs/String.h>
#include <ros/ros.h>

class SteeringDriver {
public:
    SteeringDriver(int argc, char **argv, std::string node_name);
private:
    void commandCallBack(const std_msgs::String::ConstPtr& msg);

    ros::Subscriber command_subscriber;
};
#endif //DRIVERS_STEERING_DRIVER_H
