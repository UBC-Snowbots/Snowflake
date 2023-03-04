/*
 * Created By: Tate Kolton and Ihsan Olawale
 * Created On: July 4, 2022
 * Description: Header file for recieving messages from pro controller and
 * relaying them to arm hardware driver module
 */

#ifndef ARM_HARDWARE_DRIVER_MYNODE_H
#define ARM_HARDWARE_DRIVER_MYNODE_H

// STD Includes
#include <iostream>
#include <string>
#include <cstdio>
#include <unistd.h>

// ROS Includes
#include <math.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <vector>

// Snowbots Includes
#include <sb_msgs/ArmPosition.h>
#include <sb_utils.h>

// Other
#include <serial/serial.h>


using std::string;
using std::exception;
using std::cout;
using std::cerr;
using std::endl;
using std::vector;

class ArmHardwareDriver {
  public:
    ArmHardwareDriver(ros::NodeHandle& nh);
    void allControllerCallback(const std_msgs::String::ConstPtr& inMsg);
    void parseInput(std::string inMsg);
    void joint_space_motion(std::string inMsg);
    void endEffector(const char dir);
    void endEffectorRel();
    void homeArm();
    void cartesian_motion(std::string inMsg);
    void cartesian_moveit_move(std::vector<double>& pos_commands,
                               std::vector<double>& joint_positions);
    void updateEncoderSteps(std::string msg);
    void encStepsToJointPos(std::vector<int>& enc_steps,
                            std::vector<double>& joint_positions);
    void jointPosToEncSteps(std::vector<double>& joint_positions,
                            std::vector<int>& enc_steps);
    void sendMsg(std::string outMsg);
    void recieveMsg();
    void requestArmPosition();
    void updateHWInterface();
    void homeEE();
    void axisRelease(const char axis);
    void axisMove(const char axis, const char dir);

    //new serial
    unsigned long baud = 9600;
    string port = "/dev/ttyACM0";



    // character representations of buttons for arm communication
    const char leftJSU     = 'A';
    const char leftJSD     = 'B';
    const char rightJSU    = 'C';
    const char rightJSD    = 'D';
    const char buttonA     = 'E';
    const char buttonB     = 'F';
    const char buttonX     = 'G';
    const char buttonY     = 'H';
    const char triggerL    = 'I';
    const char triggerR    = 'J';
    const char bumperL     = 'K';
    const char bumperR     = 'L';
    const char buttonARel  = 'M';
    const char buttonBRel  = 'N';
    const char buttonXRel  = 'O';
    const char buttonYRel  = 'P';
    const char triggerLRel = 'Q';
    const char triggerRRel = 'R';
    const char bumperLRel  = 'S';
    const char bumperRRel  = 'T';
    const char arrowL      = 'U';
    const char arrowR      = 'V';
    const char arrowU      = 'W';
    const char arrowD      = 'X';
    const char arrowRLRel  = '0';
    const char leftJSRel   = 'Y';
    const char rightJSRel  = 'Z';
    const char rightJSPress = '7';
    const char rightJSPressRel = '8';
    const char homeVal     = '4';
    const char homeValEE = '6';

    const char J1 = '1';
    const char J2 = '2';
    const char J3 = '3';
    const char J4 = '4';
    const char J5 = '5';
    const char J6 = '6';

    // arm modes
    const char jointMode = '1';
    const char IKMode    = '2';

    // joystick direction characters
    const char left    = 'L';
    const char right   = 'R';
    const char up      = 'U';
    const char down    = 'D';
    const char wrist   = 'W';
    const char garbage = 'G';
    const char open = 'O';
    const char close = 'C';

    int num_joints_ = 6;
    double ppr      = 400.0;
    double encppr   = 512.0;

    bool homeFlag = false;
    char mode = jointMode;

    // hardware interface communication variables
    std::vector<int> encPos, encCmd;
    std::vector<double> armCmd, armPos, poseCmd, encStepsPerDeg;
    std::vector<double> reductions{50.0, 160.0, 92.3077, 43.936, 57, 14};

  private:
    ros::NodeHandle nh;
    void armPositionCallBack(const sb_msgs::ArmPosition::ConstPtr& cmd_msg);
    void poseSelectCallback(const sb_msgs::ArmPosition::ConstPtr& poseAngles);

    ros::Subscriber subPro;
    ros::Subscriber subPose;
    ros::Subscriber subCmdPos;
    ros::Publisher pubObservedPos;

    serial::Serial teensy;
};
#endif // ARM_HARDWARE_DRIVER_MYNODE_H
