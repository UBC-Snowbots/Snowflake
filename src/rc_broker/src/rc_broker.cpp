/**
 * Created by William Gu on Jun 1 2019
 * Runs the remote communications broker node, which
 * forwards String messages between client and server nodes.
 */

#include <RCBrokerNode.h>

int main(int argc, char** argv) {
    // Set up ROS node
    std::string node_name = "rc_broker";

    // Create an instance of the class
    RCBrokerNode node(argc, argv, node_name);

    // Start up
    ros::spin();

    // Once node stops, return 0
    return 0;
}
