/*
 * Created By: Robyn Castro
 * Created On: October 14, 2018
 * Description: Spawns Risk Analysis Node
 */

#include "RiskAnalysisNode.h"

#include <ros/ros.h>

int main(int argc, char** argv) {
    // Setup your ROS node
    std::string node_name = "risk_analysis";
    // Create an instance of your class
    RiskAnalysisNode risk_analysis(argc, argv, node_name);
    // Start up ROS, this will continue to run until the node is killed
    ros::spin();
    // Once the node stops, return 0
    return 0;
}
