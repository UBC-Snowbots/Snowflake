/*
 * Created By: Robyn Castro
 * Created On: June 15, 2017
 * Description: Filters out all non-green from the image
 *              then publishes the new filtered image.
 */

#include <IPMFilterNode.h>
#include <ros/ros.h>

int main(int argc, char** argv) {
    // Setup your ROS node
    std::string node_name = "ipm_filter";
    // Create an instance of your class
    IPMFilterNode ipm_filter(argc, argv, node_name);
    // Start up ROS, this will continue to run until the node is killed
    ros::spin();
    // Once the node stops, return 0
    return 0;
}
