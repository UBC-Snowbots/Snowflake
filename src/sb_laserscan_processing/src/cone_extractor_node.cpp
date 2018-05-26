/*
 * Created by William Gu on 24/03/18
 * Creates a cone extractor node (for identifying cones from a laser message)
 */

#include <ConeExtractorNode.h>

int main(int argc, char** argv) {
    // Set up ROS node
    std::string node_name = "cone_extractor_node";

    // Create an instance of the class
    ConeExtractorNode node(argc, argv, node_name);

    // Start up
    ros::spin();

    // Once node stops, return 0
    return 0;
}