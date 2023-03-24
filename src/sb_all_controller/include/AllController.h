/*
 * Created By: Kevin Lin
 * Created On: December 21st, 2019
 * Description: Simple header file for switch controller-->ROS Twist message cpp
 */

#ifndef ALLCONTROLLER_SNOWBOTS_CONTROLLER_H
#define ALLCONTROLLER_SNOWBOTS_CONTROLLER_H
#define NUM_BUTTONS 12

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <iostream>
//#include <libevdev-1.0/libevdev/libevdev.h>
#include <ros/ros.h>
#include <sys/fcntl.h>
#include <tuple>
#include <unistd.h>

using namespace std;

class AllController {
  public:
    AllController(int argc, char** argv, std::string node_name);

  private: //TODO: make vel relative to analog values
    void setup();
    void processInputs();
    void readJoyInputs(const sensor_msgs::Joy::ConstPtr& msg);
    bool inDeadzone(int value);
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
    void leftPaddle(int value);        // Same as trigger, but discreet not analog
    void rightPaddle(int value);       // ABS_RZ
    void arrowsRorL(int value);         // ABS_HAT0X
    void arrowsUorD(int value);         // ABS_HAT0Y
    void leftJoystickPress(int value);  // BTN_THUMBL
    void rightJoystickPress(int value); // BTN_THUMBR
    tuple<double, double> publishMoveXZ(double x_new, double z_new, double x_old, double z_old);
    void publishArmMessage(std::string outMsg);
    void printState();
    void printControllerDebug(int type, int code, int value);
    void publishCmds();

    // see documentation to changes sensitivities at runtime
    double X_SENSITIVITY = 1.0;
    double Z_SENSITIVITY = 1.0;
    double x;
    double z;
    std::string armOutMsg, armOutVal;
// character representations of buttons for arm communication
    const char leftJSU = 'A';
    const char leftJSD = 'B';
    const char rightJSU = 'C';
    const char rightJSD = 'D';
    const char buttonA = 'E';
    const char buttonB = 'F';
    const char buttonX = 'G';
    const char buttonY = 'H';
    const char triggerL = 'I';
    const char triggerR = 'J';
    const char bumperL = 'K';
    const char bumperR = 'L';
    const char buttonARel = 'M';
    const char buttonBRel = 'N';
    const char buttonXRel = 'O';
    const char buttonYRel = 'P';
    const char triggerLRel = 'Q';
    const char triggerRRel = 'R';
    const char bumperLRel = 'S';
    const char bumperRRel = 'T';
    const char arrowL = 'U';
    const char arrowR = 'V';
    const char arrowU = 'W';
    const char arrowD = 'X';
    const char arrowRLRel = '0';
    const char arrowUDRel = '5';
    const char leftJSRel = 'Y';
    const char rightJSRel = 'Z';
    const char rightJSPress = '7';
    const char rightJSPressRel = '8';
    const char homeVal = '4';
    const char homeValEE = '6';
    const char J1 = '1';
    const char J2 = '2';
    const char J3 = '3';
    const char J4 = '4';
    // arm modes
    const char jointMode = '1';
    const char IKMode = '2';
    const char drillMode = '3';

    struct libevdev* dev = NULL;
    enum Mode { wheels = 0, arm_joint_space = 1, arm_cartesian = 2, drilling = 3, num_modes = 2 };
    Mode state;
    bool debug = false;
    ros::Publisher pubmove;
    ros::Publisher pubarm;
    ros::Publisher pubmode;
    ros::Publisher pubmovegrp;
    ros::Subscriber joyinput;

    std_msgs::Bool true_message;
    std_msgs::Bool false_message;
    int speed = 50;
    int max_speed = 100;
    int increment = 10;

};

#endif // ALLCONTROLLER_SNOWBOTS_CONTROLLER_H
