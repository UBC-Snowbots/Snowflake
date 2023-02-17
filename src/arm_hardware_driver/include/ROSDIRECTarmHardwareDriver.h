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
#include <sensor_msgs/Joy.h>

// Snowbots Includes
#include <sb_msgs/ArmPosition.h>
#include <sb_msgs/armFirmware.h>
#include <sb_utils.h>

// Other
#define debug 1

class ArmHardwareDriver {
  public:
    ArmHardwareDriver(ros::NodeHandle& nh);
    void teensyPosCallback(const sb_msgs::ArmPosition inMsg);
    //void obsPosCallback(const sb_);
    void parseInput(std::string inMsg);
    void joint_space_motion(std::string inMsg);
    void drill_motion(std::string inMsg);
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
    void requestEEFeedback();
    void requestJPFeedback();
    void homeEE();
    void axisRelease(const char axis);
    void axisMove(const char axis, const char dir);
    void newInput();
    //void joyparse(std::int32_t buttons[], std::float_t axes[]); //maybe return a time? so we know how long since the last input

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
    const char drillMode = '3';

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
    //char mode = jointMode

    // hardware interface communication variables
    std::vector<int> encPos, encCmd;
    std::vector<double> armCmd, armPos, encStepsPerDeg;
    std::vector<double> reductions{50, 161, 93.07, 44.8, 28, 14};

    

    // timer variables
    double refresh_rate_hz = 10.0;
    ros::Timer arm_pos_timer;

  private:
    ros::NodeHandle nh;
    sb_msgs::armFirmware Arm; //most recent arm information


    void armCMDPositionCallBack(const sb_msgs::ArmPosition::ConstPtr& cmd_msg);
    void microComCallBack(const sb_msgs::armFirmware::ConstPtr& micro_msg);
    void joyCallBack(const sensor_msgs::Joy::ConstPtr& joy_msg);


    //void teensyFeedback(const ros::TimerEvent& e);

    ros::Subscriber sub_command_pos;
    ros::Subscriber sub_obs_pos; //observed position
    ros::Subscriber sub_joy;
    ros::Publisher pub_cmd;
       ros::Publisher pub_joint_cmd;
    

    //user input
    sensor_msgs::Joy joy; //controller
    //input flags
    //bool mode

 //   ros::Timer feedbackLoop;
 
};
#endif // ARM_HARDWARE_DRIVER_MYNODE_H
