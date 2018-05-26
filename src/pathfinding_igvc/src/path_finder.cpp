/*
 * Created By: Min Gyo Kim
 * Created On: May 5th 2018
 * Description: Spawns path finding node
 */

#include "ros/ros.h"
#include <PathFinderNode.h>

int main(int argc, char** argv) {
    // Setup your ROS node
    std::string node_name = "path_finder";

    // Create an instance of your class
    PathFinderNode node(argc, argv, node_name);

    // Start up ros. This will continue to run until the node is killed
    ros::spin();

    // Once the node stops, return 0
    return 0;
}
