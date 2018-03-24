/*
 * Created by William Gu on 24/03/18
 * Creates a cone extractor node (for identifying cones from a laser message)
 */

#include <ConeExtractorNode.h>

int main(int argc, char** argv) {
    // Set up ROS node
    std::string node_name = "laserscan_cone_manager";

    // Create an instance of the class
    LaserscanConeManager node(argc, argv, node_name);

    // Start up
    ros::spin();

    // Once node stops, return 0
    return 0;
}