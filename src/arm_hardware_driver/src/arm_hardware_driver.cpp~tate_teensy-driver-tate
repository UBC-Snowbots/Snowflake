/*
 * Created By: Tate Kolton
 * Created On: July 4, 2022
 * Description: Node for recieving messages from pro controller and relaying them to arm hardware driver module
 */

#include "../include/armHardwareDriver.h"
#include <ros/callback_queue.h>

int main(int argc, char** argv) {


    // Setup your ROS node
    std::string node_name = "arm_hardware_driver";
    ros::CallbackQueue ros_queue;
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;
    nh.setCallbackQueue(&ros_queue);

    // Create an instance of your class
    ArmHardwareDriver teensyComm(nh);

    // Start up ros. This will continue to run until the node is killed
    ros::MultiThreadedSpinner spinner(0);
    spinner.spin(&ros_queue);

    // Once the node stops, return 0
    return 0;
}
