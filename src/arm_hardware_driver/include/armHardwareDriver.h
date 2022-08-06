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
#include <sstream>

// ROS Includes
#include <math.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <vector>

// Snowbots Includes
#include <sb_msgs/ArmPosition.h>
#include <sb_utils.h>

// Other
#include <SerialStream.h>

// #include <ArmDriver.h>

class ArmHardwareDriver {
  public:
    ArmHardwareDriver(ros::NodeHandle& nh);
    void teensySerialCallback(const std_msgs::String::ConstPtr& inMsg);
    void parseInput(std::string inMsg);
    void joint_space_motion(std::string inMsg);
    void drill_motion(std::string inMsg);
    void jointSpaceMove(const char joystick, const char dir);
    void changeSpeed(const char dir);
    void changeAxis(const char joystick);
    void releaseAxis(const char joystick, const char dir);
    void endEffector(const char dir);
    void endEffectorRel();
    void prepareDrilling();
    void collectSample();
    void depositSample();
    void manualDrill(const char dir);
    void releaseDrill();
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

    // character representations of buttons for arm communication
    const char leftJSL     = 'A';
    const char leftJSR     = 'B';
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
    const char homeVal     = '4';
    // arm modes
    const char jointMode = '1';
    const char IKMode    = '2';
    const char drillMode = '3';

    // joystick direction characters
    const char left    = 'L';
    const char right   = 'R';
    const char up      = 'U';
    const char down    = 'D';
    const char wrist   = 'W';
    const char garbage = 'G';

    int num_joints_ = 6;
    double ppr      = 400.0;
    double encppr   = 512.0;

    // hardware interface communication variables
    std::vector<int> encPos, encCmd;
    std::vector<double> armCmd, armPos, encStepsPerDeg;
    std::vector<double> reductions{50, 161, 93.07, 44.8, 57.34, 57.34};

    // timer variables
    double refresh_rate_hz = 10.0;
    ros::Timer arm_pos_timer;

  private:
    ros::NodeHandle nh;
    void armPositionCallBack(const sb_msgs::ArmPosition::ConstPtr& cmd_msg);

    ros::Subscriber subPro;
    ros::Subscriber sub_command_pos;
    ros::Publisher pub_observed_pos;

    // The SerialStream to/from the teensy
    LibSerial::SerialStream teensy;

    // The Port the teensy is connected to
    std::string port;
};
#endif // ARM_HARDWARE_DRIVER_MYNODE_H
