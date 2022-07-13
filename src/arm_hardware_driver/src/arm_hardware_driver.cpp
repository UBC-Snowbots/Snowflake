/*
 * Created By: Tate Kolton
 * Created On: July 4, 2022
 * Description: Node for recieving messages from pro controller and relaying them to arm hardware driver module
 */

#include "../include/armHardwareDriver.h"

int main(int argc, char** argv) {
    // Setup your ROS node
    std::string node_name = "arm_hardware_driver";

    // Create an instance of your class
    ArmHardwareDriver teensyComm(argc, argv, node_name);

    // Start up ros. This will continue to run until the node is killed
    ros::spin();

    // Once the node stops, return 0
    return 0;
}
