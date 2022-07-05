/*
 * Created By: Tate Kolton
 * Created On: July 4, 2022
 * Description: This package receives info from /cmd_arm topic and publishes serial data via callback to be recieved by the arm MCU (Teensy 4.1)
 */

#include "../include/teensyProInterface.h"

TeensyProInterface::TeensyProInterface(int argc, char** argv) {

    // Setup NodeHandles
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    // Setup Subscriber
    subPro = nh.subscribe("/cmd_arm", 1, &TeensyProInterface::teensySerialCallback, this);
}


// Callback function to relay pro controller messages to teensy MCU on arm via rosserial
void TeensyProInterface::teensySerialCallback(std::string inMsg) {
    parseInput(inMsg);
}

void TeensyProInterface::parseInput(std::string inMsg) {

    char mode = inMsg.at(0);

    switch(mode) {
        case jointMode: joint_space_motion(inMsg); break;
        case IKMode: cartesian_motion(inMsg); break;
        case drillMode: drill_motion(inMsg); break;
    }
}

// Sends joint space motion related commands to teensy
void TeensyProInterface::joint_space_motion(std::string inMsg) {

    ROS_INFO("Joint space command recieved");
    char action = inMsg.at(1);
    
    switch(action) {
        case leftJSL: jointSpaceMove(left, left); break;
        case leftJSR: jointSpaceMove(left, right); break;
        case rightJSU: jointSpaceMove(right, up); break;
        case rightJSD: jointSpaceMove(right, down); break;
        case buttonA: jointSpaceMove(wrist, down); break;
        case buttonB: jointSpaceMove(wrist, right); break;
        case buttonX: jointSpaceMove(wrist, left); break;
        case buttonY: jointSpaceMove(wrist, up); break;
        case triggerL: changeAxis(horizantal_axes); break;
        case triggerR: changeAxis(vertical_axes); break;
        case leftJSRel: releaseAxis(left); break;
        case rightJSRel: releaseAxis(right); break;
        case buttonRel: releaseAxis(wrist); break;
        case arrowU: changeSpeed(up); break;
        case arrowD: changeSpeed(down); break;
        case arrowL: endEffector(left); break;
        case arrowR: endEffector(right); break;
        case arrowRLRel: endEffectorRel(); break;
        case homeVal: homeArm(); break;
    }
}

// Sends cartesian motion related commands to teensy
void TeensyProInterface::cartesian_motion(std::string inMsg) {
    // to fill with arm_hardware_interface_implementation
}

// Sends drilling mode related commands to teensy
void TeensyProInterface::drill_motion(std::string inMsg) {

    ROS_INFO("Drilling command recieved");
    char action = inMsg.at(1);

    switch(action) {
        case buttonA: prepareDrilling(); break;
        case buttonB: collectSample(); break;
        case buttonX: depositSample(); break;
        case triggerL: manualDrill(left); break;
        case triggerR: manualDrill(right); break;
        // below two lines to be implemented once cartesian mode is sorted
        // case rightJSU: moveDrillUp(); break;
        // case rightJSD: moveDrillDown(); break;
    }
}

void TeensyProInterface::jointSpaceMove(char joystick, char dir)
{
    std::string outMsg = "JM";
    outMsg += "M";
    outMsg += dir;
    outMsg += joystick;
    outMsg += "\n";
    sendMsg(outMsg);
}

void TeensyProInterface::changeSpeed(char dir)
{
    std::string outMsg = "JM";
    outMsg = "S";
    outMsg += dir;
    outMsg += "\n";
    sendMsg(outMsg);
}

void TeensyProInterface::changeAxis(char joystick)
{
    std::string outMsg = "JM";
    outMsg += "A";
    outMsg += joystick;
    outMsg += "\n";
    sendMsg(outMsg);
}

void TeensyProInterface::releaseAxis(char joystick)
{
    std::string outMsg = "JM";
    outMsg += "R";
    outMsg += joystick;
    outMsg += "\n";
    sendMsg(outMsg);
}

// End Effector Hardware Driver Functions
void TeensyProInterface::endEffector(char dir)
{
    std::string outMsg = "EE";
    outMsg += dir;
    outMsg += "\n";
    sendMsg(outMsg);
}

void TeensyProInterface:: endEffectorRel(char dir)
{
    std::string outmsg = "EEX\n";
    sendMsg(outMsg);
}

// Drilling Mode Hardware Driver Functions
void TeensyProInterface::prepareDrilling()
{
    std::string outMsg = "DMP\n";
    sendMsg(outMsg);
}

void TeensyProInterface::collectSample()
{
    std::string outMsg = "DMC\n";
    sendMsg(outMsg);
}

void TeensyProInterface::depositSample()
{
    std::string outMsg = "DMD\n";
    sendMsg(outMsg);
}

void TeensyProInterface::manualDrill(char dir)
{
    std::string outMsg = "DM";
    outMsg += dir;
    outMsg += "\n";
    sendMsg(outMsg);
}

void TeensyProInterface::homeArm() {
    std::string outMsg = "Z\n";
    sendMsg(outMsg);
}


// Serial Communication Protocols 
// IHSAN TO FILL OUT ROSSERIAL IMPLEMENTATION

void sendMsg(std::string outMsg)
{

}

void recieveMsg(std::string inMsg)
{
    
}

void initCommunication()
{

}

