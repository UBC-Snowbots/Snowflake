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
//    Supported events (output from running evtest in terminal):
//    Event type 0 (EV_SYN)
//    Event type 1 (EV_KEY)
//    Event code 304 (BTN_SOUTH) B button
//    Event code 305 (BTN_EAST) A button
//    Event code 307 (BTN_NORTH) Y button, not X for some reason
//    Event code 308 (BTN_WEST) X button, not Y for some reason
//    Event code 310 (BTN_TL) Left top trigger
//    Event code 311 (BTN_TR) Right top trigger
//    Event code 314 (BTN_SELECT) - button
//    Event code 315 (BTN_START) + button
//    Event code 316 (BTN_MODE) Home button
//    Event code 317 (BTN_THUMBL) Left joystick pressed down
//    Event code 318 (BTN_THUMBR) Right joystick pressed down
//    Event type 3 (EV_ABS)
//    Event code 0 (ABS_X) Left joystick
//    Value    135
//    Min        0
//    Max      255
//    Event code 1 (ABS_Y) Left joystick
//    Value    126
//    Min        0
//    Max      255
//    Event code 2 (ABS_Z) Left bottom trigger
//    Value      0
//    Min        0
//    Max      255
//    Event code 3 (ABS_RX) Right joystick
//    Value    129
//    Min        0
//    Max      255
//    Event code 4 (ABS_RY) Right joystick
//    Value    129
//    Min        0
//    Max      255
//    Event code 5 (ABS_RZ) Right bottom trigger
//    Value      0
//    Min        0
//    Max      255
//    Event code 16 (ABS_HAT0X) Arrow keys left and right
//    Value      0
//    Min       -1
//    Max        1
//    Event code 17 (ABS_HAT0Y) Arrow keys up and down
//    Value      0
//    Min       -1
//    Max        1

    //run "evtest" in terminal to figure out which path leads to your connected controller
    const char *EVTEST_PATH = "/dev/input/event17";
//Change this value (default 1.0) to change the x (forward and backward) sensitivity
    const int X_SENSITIVITY = 1.0;
//Change this value (default 1.0) to change the z (turning speed) sensitivty
    const int Z_SENSITIVITY = 1.0;
    struct libevdev *dev = NULL;
    ros::Publisher pubmove;
    ros::Publisher pubarm;
};

#endif //PROCONTROLLER_SNOWBOTS_CONTROLLER_H
