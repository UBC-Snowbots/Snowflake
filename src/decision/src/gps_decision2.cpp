/*
 * Created By: Gareth Ellis
 * Created On: May 18, 2017
 * Description: The Decision Node for GPS, takes in a point relative to
 *              the robots location and heading and broadcasts a
 *              recommended twist message. Revised to use TF's
 */

#include <GpsDecision2.h>


int main(int argc, char **argv){
    // Setup your ROS node
    std::string node_name = "gps_decision";

    // Create an instance of your class
    GpsDecision2 gps_decision(argc, argv, node_name);

    // Start up ros. This will continue to run until the node is killed
    ros::spin();

    // Once the node stops, return 0
    return 0;
}
