/*
 * Created By: Gareth Ellis
 * Created On: July 16th, 2016
 * Description: The Vision decision Node
 *
 */

#include <VisionDecision.h>

int main(int argc, char **argv){
    // Setup your ROS node
    std::string node_name = "vision_decision";

    //ros::init(argc, argv, node_name);

    // Create an instance of your class
    VisionDecision vision_decision(argc, argv, node_name);

    // Start up ros. This will continue to run until the node is killed
    ros::spin();

    // Once the node stops, return 0
    return 0;
}
