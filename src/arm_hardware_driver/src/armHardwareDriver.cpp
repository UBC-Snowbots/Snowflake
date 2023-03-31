/*
 * Created By: Tate Kolton
 * Created On: July 4, 2022
 * Description: This package receives info from /cmd_arm topic and publishes
 * serial data via callback to be recieved by the arm MCU (Teensy 4.1)
 */

#include "../include/armHardwareDriver.h"

ArmHardwareDriver::ArmHardwareDriver(ros::NodeHandle& nh) : nh(nh) {
    // Setup NodeHandles

    ros::NodeHandle private_nh("~");

    // Setup Subscribers
    int queue_size = 55;

    subPro = nh.subscribe( 
        "/cmd_arm", queue_size, &ArmHardwareDriver::allControllerCallback, this);

    subCmdPos = nh.subscribe(
        "/cmd_pos_arm", queue_size, &ArmHardwareDriver::armPositionCallBack, this);

    subPose = private_nh.subscribe("/cmd_pose", 1, &ArmHardwareDriver::poseSelectCallback, this);

    pubObservedPos = private_nh.advertise<sb_msgs::ArmPosition>("/observed_pos_arm", 1);


    teensy.setBaudrate(115200);
    teensy.setPort("/dev/ttyACM1");
    teensy.open();
    teensy.setDTR(false);
    teensy.setRTS(false);


    encCmd.resize(num_joints_);
    armCmd.resize(num_joints_);
    encStepsPerDeg.resize(num_joints_);
    armPos.resize(num_joints_);
    encPos.resize(num_joints_);
    armCmd.resize(num_joints_);
    poseCmd.resize(num_joints_);

    for (int i = 0; i < num_joints_; i++) {
        encStepsPerDeg[i] = reductions[i] * ppr * 5.12 / 360.0;
    }
    
}

// Callback function to relay pro controller messages to teensy MCU on arm via
void ArmHardwareDriver::allControllerCallback(const std_msgs::String::ConstPtr& inMsg) {
    std::string tmp = inMsg->data;
    //ROS_INFO("%s", tmp.c_str());
    parseInput(tmp); 
}

void ArmHardwareDriver::parseInput(std::string inMsg) {
    mode = inMsg[0];

    if (mode == jointMode) {
        joint_space_motion(inMsg);
    } else if (mode == IKMode) {
        cartesian_motion(inMsg);
    } 
}

// Sends joint space motion related commands to teensy
void ArmHardwareDriver::joint_space_motion(std::string inMsg) {
    char action = inMsg[1];

    if(action == homeVal) {
        homeArm();
        recieveMsg();
    } else if(action == leftJSU) {
        axisMove(J3,up);
        recieveMsg();
    } else if (action == leftJSD) {
        axisMove(J3, down);
        recieveMsg();
    } else if (action == rightJSU) {
        axisMove(J2, up);
        recieveMsg();
    } else if (action == rightJSD) {
        axisMove(J2, down);
        recieveMsg();
    } else if (action == leftJSRel) {
        axisRelease(J3);
        recieveMsg();
    } else if (action == rightJSRel) {
        axisRelease(J2);
        recieveMsg();
    } else if (action == buttonY) {
        axisMove(J6, left);
        recieveMsg();
    } else if (action == buttonA) {
        axisMove(J6, right);
        recieveMsg();
    } else if (action == buttonB) {
        axisMove(J5, down);
        recieveMsg();
    } else if (action == buttonX) {
        axisMove(J5, up);
        recieveMsg();
    } else if (action == triggerL) {
        axisMove(J1, left);
        recieveMsg();
    } else if (action == triggerR) {
        axisMove(J1, right);
        recieveMsg();
    } else if ((action == triggerLRel) || (action == triggerRRel)) {
        axisRelease(J1);
        recieveMsg();
    } else if (action == buttonARel) {
        axisRelease(J6);
        recieveMsg();
    } else if (action == buttonBRel) {
        axisRelease(J5);
        recieveMsg();
    } else if (action == buttonXRel) {
        axisRelease(J5);
        recieveMsg();
    } else if (action == buttonYRel) {
        axisRelease(J6);
        recieveMsg();
    } else if(action == bumperL) {
        axisMove(J4, left);
        recieveMsg();
    } else if(action == bumperR) {
        axisMove(J4, right); 
        recieveMsg();
    } else if((action == bumperLRel) || (action == bumperRRel)) {
        axisRelease(J4);
        recieveMsg();
    } else if (action == arrowL) {
        endEffector(open);
        recieveMsg();
    } else if (action == arrowR) {
        endEffector(close);
        recieveMsg();
    } else if (action == arrowRLRel) {
        endEffectorRel();
        recieveMsg();
    } else if(action == homeValEE) {
        homeEE();
        recieveMsg();
        
    }
}

void ArmHardwareDriver::cartesian_motion(std::string inMsg) {
    char action = inMsg[1];

    if (action == arrowL) {
        endEffector(open);
    } else if (action == arrowR) {
        endEffector(close);
    } else if (action == arrowRLRel) {
        endEffectorRel();
    } else if(action == homeValEE) {
        homeEE();
    }

    recieveMsg();
}

void ArmHardwareDriver::axisMove(const char axis, const char dir)
{
    std::string outMsg = "JMM";
    outMsg += axis;
    outMsg += dir;
    outMsg += "\n";
    sendMsg(outMsg);
}

void ArmHardwareDriver::axisRelease(const char axis)
{
    std::string outMsg = "JMR";
    outMsg += axis;
    outMsg += "\n";
    sendMsg(outMsg);
}

// End Effector Hardware Driver Functions
void ArmHardwareDriver::endEffector(const char dir) {
    std::string outMsg = "EE";
    outMsg += dir;
    outMsg += "\n";
    sendMsg(outMsg);
}

void ArmHardwareDriver::endEffectorRel() {
    std::string outMsg = "EER\n";
    sendMsg(outMsg);
}

void ArmHardwareDriver::homeArm() {
    std::string outMsg = "HM\n";
    sendMsg(outMsg);

}

void ArmHardwareDriver::homeEE() {
    std::string outMsg = "EEH\n";
    sendMsg(outMsg);
}

void ArmHardwareDriver::poseSelectCallback(
const sb_msgs::ArmPosition::ConstPtr& poseAngles) {
    poseCmd.assign(poseAngles->positions.begin(), poseAngles->positions.end());
    jointPosToEncSteps(poseCmd, encCmd);
    
    std::string outMsg = "PM";
    for(int i=0; i < num_joints_; i++) {
        outMsg += 'A' + i;
        outMsg += std::to_string(encCmd[i]);
    }

    outMsg += "/n";
    sendMsg(outMsg);
    recieveMsg();
}

void ArmHardwareDriver::armPositionCallBack(
const sb_msgs::ArmPosition::ConstPtr& commanded_msg) {
    
    armCmd.assign(commanded_msg->positions.begin(),
                  commanded_msg->positions.end());
    jointPosToEncSteps(armCmd, encCmd);

    std::string outMsg = "MT";
    for (int i = 0; i < num_joints_; ++i) {
        outMsg += 'A' + i;
        outMsg += std::to_string(encCmd[i]);
    }
    outMsg += "\n";
    sendMsg(outMsg);
    recieveMsg();
}

void ArmHardwareDriver::updateEncoderSteps(std::string msg) {
    size_t idx1 = msg.find("A", 2) + 1;
    size_t idx2 = msg.find("B", 2) + 1;
    size_t idx3 = msg.find("C", 2) + 1;
    size_t idx4 = msg.find("D", 2) + 1;
    size_t idx5 = msg.find("E", 2) + 1;
    size_t idx6 = msg.find("F", 2) + 1;
    size_t idx7 = msg.find("Z", 2) + 1;
    encPos[0]   = std::stoi(msg.substr(idx1, idx2 - idx1));
    encPos[1]   = std::stoi(msg.substr(idx2, idx3 - idx2));
    encPos[2]   = std::stoi(msg.substr(idx3, idx4 - idx3));
    encPos[3]   = std::stoi(msg.substr(idx4, idx5 - idx4));
    encPos[4]   = std::stoi(msg.substr(idx5, idx6 - idx5));
    encPos[5]   = std::stoi(msg.substr(idx6, idx7 - idx6));
}

void ArmHardwareDriver::encStepsToJointPos(
std::vector<int>& enc_steps, std::vector<double>& joint_positions) {
    for (int i = 0; i < enc_steps.size(); ++i) {
        // convert enc steps to joint deg
        joint_positions[i] =
        static_cast<double>(enc_steps[i]) / encStepsPerDeg[i];
    }
}

void ArmHardwareDriver::jointPosToEncSteps(std::vector<double>& joint_positions,
                                           std::vector<int>& enc_steps) {
    for (int i = 0; i < joint_positions.size(); ++i) {
        // convert joint deg to enc steps
        enc_steps[i] = static_cast<int>(joint_positions[i] * encStepsPerDeg[i]);
    }
}

// Libserial Implementation

void ArmHardwareDriver::sendMsg(std::string outMsg) {
    // Send everything in outMsg through serial port
    //ROS_INFO("attempting send");
    teensy.write(outMsg);
    ROS_INFO("Sent via serial: %s", outMsg.c_str());
}

void ArmHardwareDriver::recieveMsg() {

    std::string next_char = "";
    std::string buffer = "";
    int timeoutCounter = 0;
    do {
        timeoutCounter ++;
        next_char = teensy.read();
        buffer += next_char;
    //    if(timeoutCounter > 50){
    //     ROS_INFO("timed out");
    //     next_char = "Z";
    //    }
    } while (next_char != "Z");

     ROS_INFO("buffer: %s", buffer.c_str());


    // // Update parameters based on feedback
    updateEncoderSteps(buffer);
    encStepsToJointPos(encPos, armPos);
    updateHWInterface();

}

void ArmHardwareDriver::updateHWInterface() {
    sb_msgs::ArmPosition outMsg;
    outMsg.positions.assign(armPos.begin(), armPos.end());
    pubObservedPos.publish(outMsg);
}

