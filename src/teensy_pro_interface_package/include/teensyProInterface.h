/*
 * Created By: Tate Kolton and Ihsan Olawale
 * Created On: July 4, 2022
 * Description: Header file for recieving messages from pro controller and relaying them to arm hardware driver module
 */

#ifndef TEENSY_PRO_INTERFACE_MYNODE_H
#define TEENSY_PRO_INTERFACE_MYNODE_H

// STD Includes
#include <iostream>

// ROS Includes
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <vector>
#include <math.h>

// Snowbots Includes
#include <sb_utils.h>

// Hardware Driver Functions
#include "../../arm_hardware_driver/include/armHardwareDriver.h"

class TeensyProInterface {
  public:
    TeensyProInterface(int argc, char** argv, std::string node_name);
    void armCommandCallback(std::string inMsg);
    void jointSpaceMove(int joystick, char dir);
    void changeSpeed(char dir);
    void changeAxis(char joystick);
    void releaseAxis(char joystick);
    void prepareDrilling();
    void collectSample();
    void depositSample();
    void manualDrill(char dir);
    void endEffector(char dir);
    void endEffectorRel();
    void homeArm();
    void sendMsg(std::string outMsg);
    void recieveMsg(std::string inMsg);
    void initCommunication();


    // character representations of buttons for arm communication
    const char leftJSL = 'A';
    const char leftJSR = 'B';
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
    const char buttonRel = "M";
    const char triggerLRel = 'N';
    const char triggerRRel = 'O';
    const char bumperLRel = 'P';
    const char bumperRRel = 'Q';
    const char arrowL = 'R';
    const char arrowR = 'S';
    const char arrowU = 'T';
    const char arrowD = 'U';
    const char leftJSRel = "V";
    const char rightJSRel = "W";
    const char homeVal = "X";
    // arm modes
    const char jointMode = "1";
    const char IKMode = "2";
    const char drillMode = "3";

    //joystick direction characters
    const char left = "L";
    const char right = "R";
    const char up = "U";
    const char down = "D";
    const char wrist = "W";

  private:
    ros::Subscriber subPro;
};
#endif // TEENSY_PRO_INTERFACE_MYNODE_H
