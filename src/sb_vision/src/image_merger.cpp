/*
 * Created By: Robyn Castro
 * Created On: July 12, 2018
 * Description: Combines two images (Bitwise or) into one then publishes it
 *
 */

#include <ImageMergerNode.h>
#include <ros/ros.h>

int main(int argc, char** argv) {
    // Setup your ROS node
    std::string node_name = "image_merger";
    // Create an instance of your class
    ImageMergerNode image_merger(argc, argv, node_name);
    // Start up ROS, this will continue to run until the node is killed
    ros::spin();
    // Once the node stops, return 0
    return 0;
}
