/*
 * Created By: Ihsan Olawale, Tate Kolton
 * Created On: June 26th, 2021
 * Description: An example node that subscribes to a topic publishing strings,
 *              and re-publishes everything it receives to another topic with
 *              a "!" at the end
 */

#include <ArmDriver.h>

ArmDriver::ArmDriver(int argc, char **argv, std::string node_name) {
    // Setup NodeHandles
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    // Get Params
    SB_getParam(private_nh, "port", port, (std::string) "/dev/ttyACM0");
    // Open the given serial port
    teensy.Open(port);
    teensy.SetBaudRate(LibSerial::SerialStreamBuf::BAUD_9600); // adjust
    teensy.SetCharSize(LibSerial::SerialStreamBuf::CHAR_SIZE_8); // adjust

    // Setup Subscriber(s)
    int queue_size                    = 10;
    my_subscriber                     = nh.subscribe(
    "xbox_mode", queue_size, &ArmDriver::controllerModeCallBack, this);
}

void ArmDriver::controllerModeCallBack(const std_msgs::Bool::ConstPtr& msg) {
    xbox_mode = msg->data;
    if (xbox_mode)
        ROS_INFO("Enabling Xbox Mode");
    else
        ROS_INFO("Enabling Pro Controller Mode");
}
