/*
 * Created By: Kevin Lin
 * Created On: December 21st, 2019
 * Description: Simple header file for switch controller-->ROS Twist message cpp
 */

#ifndef PROCONTROLLER_SNOWBOTS_CONTROLLER_H
#define PROCONTROLLER_SNOWBOTS_CONTROLLER_H

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <geometry_msgs/Twist.h>
#include <iostream>
#include <libevdev-1.0/libevdev/libevdev.h>
#include <ros/ros.h>
#include <sys/fcntl.h>
#include <tuple>
using namespace std;

class ProController {
  public:
    ProController(int argc, char** argv, std::string node_name);

  private:
    void setup();
    void readInputs();
    void leftJoystickX(int value);      // ABS_X
    void leftJoystickY(int value);      // ABS_Y
    void rightJoystickX(int value);     // ABS_RX
    void rightJoystickY(int value);     // ABS_RY
    void A(int value);                  // BTN_EAST
    void B(int value);                  // BTN_SOUTH
    void X(int value);                  // BTN_WEST
    void Y(int value);                  // BTN_NORTH
    void leftBumper(int value);         // BTN_TL
    void rightBumper(int value);        // BTN_TR
    void select(int value);             // BTN_SELECT
    void start(int value);              // BTN_START
    void home(int value);               // BTN_MODE
    void leftTrigger(int value);        // ABS_Z
    void rightTrigger(int value);       // ABS_RZ
    void arrowsRorL(int value);         // ABS_HAT0X
    void arrowsUorD(int value);         // ABS_HAT0Y
    void leftJoystickPress(int value);  // BTN_THUMBL
    void rightJoystickPress(int value); // BTN_THUMBR
    tuple<double, double>
    publishMoveXZ(double x_new, double z_new, double x_old, double z_old);
    void publishArmXZ(double x_new, double z_new, double x_old, double z_old);
    void printState();
    void printControllerDebug(int type, int code, int value);
    // see documentation to changes sensitivities at runtime
    double X_SENSITIVITY = 1.0;
    double Z_SENSITIVITY = 1.0;
    double x;
    double z;
    struct libevdev* dev = NULL;
    enum Mode { wheels = 0, arm_joint_space = 1, arm_cartesian = 2, drilling = 3 };
    Mode state;
    bool debug = false;
    ros::Publisher pubmove;
    ros::Publisher pubarm;
};

#endif // PROCONTROLLER_SNOWBOTS_CONTROLLER_H
