/*
 * Created By: Min Gyo Kim
 * Created On: January 20, 2018
 * Description: Spawns Line Extractor Node
 */

#include <LineExtractorNode.h>

int main(int argc, char** argv) {
    // Set up ROS node
    std::string node_name = "line_extractor_node";

    // Create an instance of the class
    LineExtractorNode node(argc, argv, node_name);

    // Start up
    ros::spin();

    // Once node stops, return 0
    return 0;
}