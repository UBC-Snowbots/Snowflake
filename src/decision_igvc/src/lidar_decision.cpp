/*
 * Created By: Gareth Ellis
 * Created On: September 22, 2016
 * Description: The Lidar decision node, takes in a raw Lidar scan
 *              and broadcasts a recommended Twist message
 */

#include <LidarDecision.h>

int main(int argc, char** argv) {
    // Setup your ROS node
    std::string node_name = "lidar_decision";

    // Create an instance of your class
    LidarDecision lidar_decision(argc, argv, node_name);

    // Start up ros. This will continue to run until the node is killed
    ros::spin();

    // Once the node stops, return 0
    return 0;
}
