/*
 * Created By: Gareth Ellis
 * Created On: April 3, 2018
 * Description: The node wrapper for the `ObstacleManager` that provides us
 *              with a discrete map of our environment and can generate
 *              Occupancy grids for navigation
 */

#include "ObstacleManagerNode.h"


int main(int argc, char **argv){
    // Setup your ROS node
    std::string node_name = "obstacle_manager_node";

    // Create an instance of your class
    ObstacleManagerNode obstacle_manager_node(argc, argv, node_name);

    // Start up ros. This will continue to run until the node is killed
    ros::spin();

    // Once the node stops, return 0
    return 0;
}