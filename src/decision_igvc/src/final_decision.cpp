/*
 * Created By: YOUR NAME HERE
 * Created On: September 22, 2016
 * Description: The Decision Node for GPS, takes in a point relative to
 *              the robots location and heading and broadcasts a
 *              recommended twist message
 */

#include <FinalDecision.h>

int main(int argc, char** argv) {
    // Setup your ROS node
    std::string node_name = "final_decision";

    // Create an instance of your class
    FinalDecision gps_decision(argc, argv, node_name);

    // Start up ros. This will continue to run until the node is killed
    ros::spin();

    // Once the node stops, return 0
    return 0;
}
