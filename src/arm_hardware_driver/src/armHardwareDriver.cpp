/*
 * Created By: Tate Kolton
 * Created On: July 4, 2022
 * Description: This package receives info from /cmd_arm topic and publishes serial data via callback to be recieved by the arm MCU (Teensy 4.1)
 */


#include "../include/armHardwareDriver.h"

ArmHardwareDriver::ArmHardwareDriver(int argc, char** argv, std::string node_name) {

    // Setup NodeHandles
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    // Setup Subscribers
    int queue_size                    = 10;

    subPro                     = nh.subscribe(
    "/cmd_arm", queue_size, &ArmHardwareDriver::teensySerialCallback, this);
    sub_command_pos = nh.subscribe("/cmd_pos_arm", queue_size, &ArmHardwareDriver::armPositionCallBack, this);
    pub_observed_pos = private_nh.advertise<sb_msgs::ArmPosition>("/observed_pos_arm", 1);

    // Get Params
    SB_getParam(private_nh, "port", port, (std::string) "/dev/ttyACM0");
    // Open the given serial port
    teensy.Open(port);
    teensy.SetBaudRate(LibSerial::SerialStreamBuf::BAUD_9600);
    teensy.SetCharSize(LibSerial::SerialStreamBuf::CHAR_SIZE_8);

    encCmd.resize(num_joints_);
    armCmd.resize(num_joints_);
    encStepsPerDeg.resize(num_joints_);
    armPos.resize(num_joints_);
    armCmd.resize(num_joints_);

    for(int i=0; i<num_joints_; i++)
    {
        encStepsPerDeg[i] = reductions[i]*ppr/(360.0*encppr*4.0);
    }
}

// Callback function to relay pro controller messages to teensy MCU on arm via rosserial
void ArmHardwareDriver::teensySerialCallback(std_msgs::String& inMsg) {
    parseInput(inMsg.data);
}

void ArmHardwareDriver::parseInput(std::string inMsg) {

    char mode = inMsg[0];

    if (mode == jointMode) {
        joint_space_motion(inMsg);
    } else if (mode == IKMode) {
        cartesian_motion(inMsg);
    } else if (mode == drillMode) {
        drill_motion(inMsg);
    }
}

// Sends joint space motion related commands to teensy
void ArmHardwareDriver::joint_space_motion(std::string inMsg) {

    ROS_INFO("Joint space command recieved");
    char action = inMsg[1];
    
    if (action == leftJSL) {
        jointSpaceMove(left, left);
    } else if (action == leftJSR) {
        jointSpaceMove(left, right);
    } else if (action == rightJSU) {
        jointSpaceMove(right, up);
    } else if (action == rightJSD) {
        jointSpaceMove(right, down);
    } else if (action == buttonA) {
        jointSpaceMove(wrist, down);
    } else if (action == buttonB) {
        jointSpaceMove(wrist, right);
    } else if (action == buttonX) {
        jointSpaceMove(wrist, left);
    } else if (action == buttonY) {
        jointSpaceMove(wrist, up);
    } else if (action == triggerL) {
        changeAxis(down);
    } else if (action == triggerR) {
        changeAxis(up);
    } else if (action == leftJSRel) {
        releaseAxis(right, garbage);
    } else if (action == rightJSRel) {
        releaseAxis(wrist, garbage);
    } else if (action == buttonARel) {
        releaseAxis(wrist, right);
    } else if (action == buttonBRel) {
        releaseAxis(wrist, down);
    } else if (action == buttonXRel) {
        releaseAxis(wrist, up);
    } else if (action == buttonYRel) {
        releaseAxis(wrist, left);
    } else if (action == arrowU) {
        changeSpeed(up);
    } else if (action == arrowD) {
        changeSpeed(down);
    } else if (action == arrowL) {
        endEffector(left);
    } else if (action == arrowR) {
        endEffector(right);
    } else if (action == arrowRLRel) {
        endEffectorRel();
    } else if (action == homeVal) {
        homeArm();
    }
}

void ArmHardwareDriver::cartesian_motion(std::string inMsg)
{
    char action = inMsg[1];

    if (arrowL) {
        endEffector(left);
    } else if (arrowRLRel) {
        endEffector(right);
    } else if (arrowRLRel) {
        endEffectorRel();
    }
}

// Sends drilling mode related commands to teensy
void ArmHardwareDriver::drill_motion(std::string inMsg) {

    ROS_INFO("Drilling command recieved");
    char action = inMsg.at(1);

    if (buttonA) {
        prepareDrilling();
    } else if (buttonB) {
        collectSample();
    } else if (buttonX) {
        depositSample();
    } else if (triggerL) {
        manualDrill(left);
    } else if (triggerR) {
        manualDrill(right);
    } else if (triggerLRel || triggerRRel) {
        releaseDrill();
    }
    // below two lines to be implemented once cartesian mode is sorted
    // case rightJSU: moveDrillUp(); break;
    // case rightJSD: moveDrillDown(); break;
}

void ArmHardwareDriver::jointSpaceMove(const char joystick, const char dir)
{
    std::string outMsg = "JM";
    outMsg += "M";
    outMsg += joystick;
    outMsg += dir;
    outMsg += "\n";
    sendMsg(outMsg);
    recieveMsg();
}

void ArmHardwareDriver::changeSpeed(const char dir)
{
    ROS_INFO("Changing speed");
    ROS_INFO(dir == up ? "Speed increasing" : (dir == down ? "Speed decreasing" : "Incorrect direction provided"));
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
    outMsg += joystick;
    outMsg += "\n";
    sendMsg(outMsg);
}

void ArmHardwareDriver::releaseAxis(const char joystick, const char dir)
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
    // sending command to end effector results in force feedback being requested
    recieveMsg();
}

void ArmHardwareDriver::endEffectorRel()
{
    std::string outMsg = "EER\n";
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
    std::string outMsg = "DMM";
    outMsg += dir;
    outMsg += "\n";
    sendMsg(outMsg);
}

void ArmHardwareDriver::releaseDrill()
{
    std::string outMsg = "DMMX";
    outMsg += "\n";
    sendMsg(outMsg);
}

void ArmHardwareDriver::homeArm() {
    std::string outMsg = "HM\n";
    sendMsg(outMsg);
}


void ArmHardwareDriver::armPositionCallBack(const sb_msgs::ArmPosition::ConstPtr& observed_msg)
{
    // TODO: ihsan fill std::vector<double> type with sb_msgs values
    armCmd.assign(observed_msg.positions.begin(), observed_msg.positions.end());
    jointPosToEncSteps(armCmd, encCmd);

    std::string outMsg = "MT";
    for (int i = 0; i < num_joints_; ++i)
    {
        outMsg += 'A' + i;
        outMsg += std::to_string(encCmd[i]);
    }
    outMsg += "\n";
    sendMsg(outMsg);
    recieveMsg();
}

/* deprecated, for reference only

void ArmHardwareDriver::cartesian_moveit_move(std::vector<double>& pos_commands, std::vector<double>& joint_positions)
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
*/


void ArmHardwareDriver::updateEncoderSteps(std::string msg)
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

void ArmHardwareDriver::encStepsToJointPos(std::vector<int>& enc_steps, std::vector<double>& joint_positions)
{
    for (int i = 0; i < enc_steps.size(); ++i)
    {
        // convert enc steps to joint deg
        joint_positions[i] = static_cast<double>(enc_steps[i]) / enc_steps_per_deg_[i];
    }
}

void ArmHardwareDriver::jointPosToEncSteps(std::vector<double>& joint_positions, std::vector<int>& enc_steps)
{
    for (int i = 0; i < joint_positions.size(); ++i)
    {
        // convert joint deg to enc steps
        enc_steps[i] = static_cast<int>(joint_positions[i] * enc_steps_per_deg_[i]);
    }
}

// Libserial Implementation

void ArmHardwareDriver::sendMsg(std::string outMsg)
{
    // Send everything in outMsg through serial port
    teensy << outMsg;
}

void ArmHardwareDriver::recieveMsg(std::string& inMsg)
{
    // fill inMsg string with whatever comes through serial port until \n
    std::stringstream buffer;
    char next_char;
    do {
	teensy >> next_char;
	buffer << next_char;
    } while (next_char != '\n');
    inMsg = buffer.str();

    if(inMsg.substr(0, 2) == "JP")
    {
        ROS_INFO("Sending Arm Position to HW Interface")
        updateEncoderSteps(inMsg);
        encStepsToJointPos(encPos , armPos);
        updateHWInterface();
    }

    else if (inMsg.substr(0, 2) == "EE")
        ROS_INFO(inMsg);

}

void ArmHardwareDriver::updateHWInterface()
{
    // TODO: Ihsan fill in correct message implementation
    sb_msgs::ArmPosition outMsg;
    outMsg.positions.assign(armPos.begin(), armPos.end());
    pub_observed_pos.publish(outMsg);
}

