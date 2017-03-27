/*
 * Created By: Valerian Ratu
 * Created On: December 23, 2016
 * Description: A node which manages and organizes mapping information from various sources
 */

#include <Map.h>

int main(int argc, char **argv){
    // Setup your ROS node
    std::string node_name = "map_node";

    // Create an instance of your class
    Map map(argc, argv, node_name);

    // Start up ros. This will continue to run until the node is killed
    ros::spin();

    // Once the node stops, return 0
    return 0;
}