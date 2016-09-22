/*
 * Created By: Gareth Ellis
 * Created On: July 16th, 2016
 * Description: The Lidar decision Node
 *
 */

#include <LidarDecision.h>


int main(int argc, char **argv){
    // Setup your ROS node
    std::string node_name = "lidar_decision";

    ros::init(argc, argv, node_name);

    // Create an instance of your class
    LidarDecision lidar_decision();

    // Start up ros. This will continue to run until the node is killed
    ros::spin();

    // Once the node stops, return 0
    return 0;
}
