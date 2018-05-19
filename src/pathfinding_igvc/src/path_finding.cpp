/*
 * Created By: William Gu
 * Created On: Jan 20, 2018
 * Description: A path finding node which converts a given path into a Twist
 * message to send to the robot
 */

#include <PathFinding.h>

int main(int argc, char** argv) {
    // Setup your ROS node
    std::string node_name = "path_finding";

    // Create an instance of your class
    PathFinding pf_node(argc, argv, node_name);

    // Start up ros. This will continue to run until the node is killed
    ros::spin();

    // Once the node stops, return 0
    return 0;
}