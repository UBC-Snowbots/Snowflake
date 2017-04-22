/*
 * Created By: Valerian Ratu
 * Created On: April 21, 2017
 * Description: A node which analyzes 3-d colour pointcloud images and filters out
 *              objects of interest.
 */

#include <ZedFilter.h>


int main(int argc, char **argv){
    // Setup your ROS node
    std::string node_name = "my_node";

    // Create an instance of your class
    ZedFilter zed_filter(argc, argv, node_name);

    // Start up ros. This will continue to run until the node is killed
    ros::spin();

    // Once the node stops, return 0
    return 0;
}