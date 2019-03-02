/**
 * Created by William Gu on Mar 2 2019
 * Runs the state machine node
 */

#include "StateMachineNode.h"


int main(int argc, char** argv) {
    // Set up ROS node
    std::string node_name = "state_machine_node";

    // Create an instance of the class
    StateMachineNode node(argc, argv, node_name);

    // Start up
    ros::spin();

    // Once node stops, return 0
    return 0;
}
