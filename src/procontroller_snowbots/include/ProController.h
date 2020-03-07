/*
 * Created By: Kevin Lin
 * Created On: December 21st, 2019
 * Description: Simple header file for switch controller-->ROS Twist message cpp
 */


#ifndef PROCONTROLLER_SNOWBOTS_CONTROLLER_H
#define PROCONTROLLER_SNOWBOTS_CONTROLLER_H

#include <cstdio>
#include "libevdev.h"
#include <sys/fcntl.h>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>

using namespace std;

class ProController {
public:
    ProController(int argc, char **argv, std::string node_name);

private:
    void setup();

    void readInputs();

    //run "evtest" in terminal to figure out which path leads to your bluetooth connected controller
    const char *EVTEST_PATH = "/dev/input/event16";
//This publishes to the turtlesim ROS demo for testing, change this to publish to a different topic
    const char *ROS_TOPIC = "turtle1/cmd_vel";
//Change this value (default 1.0) to change the x (forward and backward) sensitivity
    const int X_SENSITIVITY = 1.0;
//Change this value (default 1.0) to change the z (turning speed) sensitivty
    const int Z_SENSITIVITY = 1.0;
    struct libevdev *dev = NULL;
    ros::Publisher pub;
};

#endif //PROCONTROLLER_SNOWBOTS_CONTROLLER_H
