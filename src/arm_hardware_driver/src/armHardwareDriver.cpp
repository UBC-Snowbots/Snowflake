/*
 * Created By: Tate Kolton
 * Created On: July 4, 2022
 * Description: This package receives info from /cmd_arm topic and publishes serial data via callback to be recieved by the arm MCU (Teensy 4.1)
 */


#include "../include/armHardwareDriver.h"

ArmHardwareDriver::ArmHardwareDriver(int argc, char** argv) {

    // Setup NodeHandles
    std::string node_name = "TeensyDriver";
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    // Setup Subscriber
    subPro = nh.subscribe("/cmd_arm", 1, ArmHardwareDriver::teensySerialCallback, this);

    // Get Params
    SB_getParam(private_nh, "port", port, (std::string) "/dev/ttyACM0");
    // Open the given serial port
    teensy.Open(port);
    teensy.SetBaudRate(LibSerial::SerialStreamBuf::BAUD_9600);
    teensy.SetCharSize(LibSerial::SerialStreamBuf::CHAR_SIZE_8);
}


// Callback function to relay pro controller messages to teensy MCU on arm via rosserial
void ArmHardwareDriver::teensySerialCallback(std_msgs::String& inMsg) {
    parseInput(inMsg.data);
}

void ArmHardwareDriver::parseInput(std::string inMsg) {

    char mode = inMsg[0];

    switch(mode) {
        case jointMode: joint_space_motion(inMsg); break;
        case IKMode: cartesian_motion(inMsg); break;
        case drillMode: drill_motion(inMsg); break;
    }
}

// Sends joint space motion related commands to teensy
void ArmHardwareDriver::joint_space_motion(std::string inMsg) {

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
        case triggerL: changeAxis(down); break;
        case triggerR: changeAxis(up); break;
        case leftJSRel: releaseAxis(left, garbage); break;
        case rightJSRel: releaseAxis(right, garbage); break;
        case buttonARel: releaseAxis(wrist, right); break;
        case buttonBRel: releaseAxis(wrist, down); break;
        case buttonXRel: releaseAxis(wrist, up); break;
        case buttonYRel: releaseAxis(wrist, left); break;
        case arrowU: changeSpeed(up); break;
        case arrowD: changeSpeed(down); break;
        case arrowL: endEffector(left); break;
        case arrowR: endEffector(right); break;
        case arrowRLRel: endEffectorRel(); break;
        case homeVal: homeArm(); break;
    }
}

// Sends drilling mode related commands to teensy
void ArmHardwareDriver::drill_motion(std::string inMsg) {

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

void ArmHardwareDriver::jointSpaceMove(const char joystick, const char dir)
{
    std::string outMsg = "JM";
    outMsg += "M";
    outMsg += dir;
    outMsg += joystick;
    outMsg += "\n";
    sendMsg(outMsg);
}

void ArmHardwareDriver::changeSpeed(const char dir)
{
    std::string outMsg = "JM";
    outMsg = "S";
    outMsg += dir;
    outMsg += "\n";
    sendMsg(outMsg);
}

void ArmHardwareDriver::changeAxis(const char joystick)
{
    std::string outMsg = "JM";
    outMsg += "A";
    outMsg += dir;
    outMsg += "\n";
    sendMsg(outMsg);
}

void ArmHardwareDriver::releaseAxis(const char joystick)
{
    std::string outMsg = "JM";
    outMsg += "R";
    outMsg += joystick;
    outMsg += dir;
    outMsg += "\n";
    sendMsg(outMsg);
}

// End Effector Hardware Driver Functions
void ArmHardwareDriver::endEffector(const char dir)
{
    std::string outMsg = "EE";
    outMsg += dir;
    outMsg += "\n";
    sendMsg(outMsg);
}

void ArmHardwareDriver:: endEffectorRel(const char dir)
{
    std::string outmsg = "EEX\n";
    sendMsg(outMsg);
}

// Drilling Mode Hardware Driver Functions
void ArmHardwareDriver::prepareDrilling()
{
    std::string outMsg = "DMP\n";
    sendMsg(outMsg);
}

void ArmHardwareDriver::collectSample()
{
    std::string outMsg = "DMC\n";
    sendMsg(outMsg);
}

void ArmHardwareDriver::depositSample()
{
    std::string outMsg = "DMD\n";
    sendMsg(outMsg);
}

void ArmHardwareDriver::manualDrill(const char dir)
{
    std::string outMsg = "DM";
    outMsg += dir;
    outMsg += "\n";
    sendMsg(outMsg);
}

void ArmHardwareDriver::homeArm() {
    std::string outMsg = "HM\n";
    sendMsg(outMsg);
}


// ROS SERIAL COMMUNICATION APIs //

void TeensyDriver::cartesian_motion(std::vector<double>& pos_commands, std::vector<double>& joint_positions)
{
    std::string inMsg = "";
    // convert angles to encoder steps for teensy
    jointPosToEncSteps(pos_commands, enc_commands_);

    // construct update message
    std::string outMsg = "MT";
    for (int i = 0; i < num_joints_; ++i)
    {
        outMsg += 'A' + i;
        outMsg += std::to_string(enc_commands_[i]);
    }
    outMsg += "\n";

    // run the communication with board
    sendMsg(outMsg);

    // get feedback from arm
    recieveMsg(inMsg);
    ROS_INFO("Recieved Arm Current Position");
    updateEncoderSteps(inMsg);

    // convert from encoder steps to angles
    encStepsToJointPos(enc_steps_ , joint_positions);
}


void TeensyDriver::updateEncoderSteps(std::string msg)
{
    size_t idx1 = msg.find("A", 2) + 1;
    size_t idx2 = msg.find("B", 2) + 1;
    size_t idx3 = msg.find("C", 2) + 1;
    size_t idx4 = msg.find("D", 2) + 1;
    size_t idx5 = msg.find("E", 2) + 1;
    size_t idx6 = msg.find("F", 2) + 1;
    enc_steps_[0] = std::stoi(msg.substr(idx1, idx2 - idx1));
    enc_steps_[1] = std::stoi(msg.substr(idx2, idx3 - idx2));
    enc_steps_[2] = std::stoi(msg.substr(idx3, idx4 - idx3));
    enc_steps_[3] = std::stoi(msg.substr(idx4, idx5 - idx4));
    enc_steps_[4] = std::stoi(msg.substr(idx5, idx6 - idx5));
    enc_steps_[5] = std::stoi(msg.substr(idx6));
}

void TeensyDriver::encStepsToJointPos(std::vector<int>& enc_steps, std::vector<double>& joint_positions)
{
    for (int i = 0; i < enc_steps.size(); ++i)
    {
        // convert enc steps to joint deg
        joint_positions[i] = static_cast<double>(enc_steps[i]) / enc_steps_per_deg_[i];
    }
}

void TeensyDriver::jointPosToEncSteps(std::vector<double>& joint_positions, std::vector<int>& enc_steps)
{
    for (int i = 0; i < joint_positions.size(); ++i)
    {
        // convert joint deg to enc steps
        enc_steps[i] = static_cast<int>(joint_positions[i] * enc_steps_per_deg_[i]);
    }
}


// Libserial Implementation

sendMsg(std::string outMsg)
{
    // Send everything in outMsg through serial port
    teensy << outMsg;
}

recieveMsg(std::string& inMsg)
{
    // fill inMsg string with whatever comes through serial port until \n
    std::stringstream buffer;
    char next_char;
    do {
	teensy >> next_char;
	buffer << next_char;
    } while (next_char != '\n');
    inMsg = buffer.str();
}

