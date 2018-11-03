/**
 * Created by William Gu on Sept 29 2018
 * Runs the reactive system node
 */

#include <ReactiveSystemNode.h>

int main(int argc, char** argv) {
    // Set up ROS node
    std::string node_name = "reactive_system_node";

    // Create an instance of the class
    ReactiveSystemNode node(argc, argv, node_name);

    // Start up
    ros::spin();

    // Once node stops, return 0
    return 0;
}