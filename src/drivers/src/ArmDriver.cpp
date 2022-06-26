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

    // Obtains character from the parameter server (or launch file), sets '!' as default
    std::string parameter_name    = "character";
    std::string default_character = "!";
    SB_getParam(private_nh, parameter_name, suffix, default_character);

    // Get Params
    SB_getParam(private_nh, "port", port, (std::string) "/dev/ttyACM0");
    // Open the given serial port
    teensy.Open(port);
    teensy.SetBaudRate(LibSerial::SerialStreamBuf::BAUD_9600); // adjust
    teensy.SetCharSize(LibSerial::SerialStreamBuf::CHAR_SIZE_8); // adjust

    // Setup Subscriber(s)
    std::string topic_to_subscribe_to = "subscribe_topic";
    int queue_size                    = 10;
    my_subscriber                     = nh.subscribe(
    topic_to_subscribe_to, queue_size, &ArmDriver::subscriberCallBack, this);

    int queue_size                    = 10;
    my_subscriber                     = nh.subscribe(
    "xbox_mode", queue_size, &ArmDriver::controllerModeCallBack, this);

    // Setup Publisher(s)
    std::string topic = private_nh.resolveName("publish_topic");
    queue_size        = 1;
    my_publisher = private_nh.advertise<std_msgs::String>(topic, queue_size);
}

void ArmDriver::subscriberCallBack(const std_msgs::String::ConstPtr& msg) {
    ROS_INFO("Received message");
    std::string input_string = msg->data.c_str();
    std::string new_msg = addCharacterToString(input_string, suffix);
    republishMsg(new_msg);
}

void ArmDriver::controllerModeCallBack(const std_msgs::Bool::ConstPtr& msg) {
    xbox_mode = msg->data;
    if (xbox_mode)
        ROS_INFO("Enabling Xbox Mode");
    else
        ROS_INFO("Enabling Pro Controller Mode");
}

std::string ArmDriver::addCharacterToString(std::string input_string, std::string suffix) {
    return input_string.append(suffix);
}

void ArmDriver::republishMsg(std::string msg_to_publish) {
    std_msgs::String string_to_publish;
    string_to_publish.data = msg_to_publish;
    my_publisher.publish(string_to_publish);
    ROS_INFO("Published message");
}
