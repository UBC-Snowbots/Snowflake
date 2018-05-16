/*
 * Created By: Min Gyo Kim
 * Created On: May 5th 2018
 * Description: Spawns path finding node
 */

#include "ros/ros.h" // TODO: include this is Node class instead
#include <PathFinder.h>

int main(int argc, char** argv) {
    // Setup your ROS node
    std::string node_name = "a_star";

    // Create an instance of your class
    // TODO: Part 3: make a node

    // Start up ros. This will continue to run until the node is killed
    ros::spin();

    // Once the node stops, return 0
    return 0;
}
