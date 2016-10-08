/*
 * Created By: YOUR NAME HERE
 * Created On: September 22, 2016
 * Description: This node is responsible for passing received twist messages over
 *              serial to the arduino controlling the robot
 *
 */

#include <SteeringDriver.h>

// The constructor for MyClass
SteeringDriver::SteeringDriver(int argc, char **argv, std::string node_name) {
    ros::init(argc, argv, node_name);
    // Setup NodeHandles
    ros::NodeHandle nh;
    ros::NodeHandle public_nh("~");

    // Setup Subscriber(s)
    std::string topic_to_subscribe_to = "/final_decision/command";
    int refresh_rate = 10;
    command_subscriber = nh.subscribe(topic_to_subscribe_to, refresh_rate, &SteeringDriver::commandCallBack, this);

}

// The callback function for the subscriber (my_subscriber).
// This is called whenever a new message is received
void SteeringDriver::commandCallBack(const std_msgs::String::ConstPtr& msg) {
    // Deal with the command here
}

