/*
 * Created By: Robyn Castro
 * Created On: June 15, 2017
 * Description: Decides when the robot starts moving
 *              based on whether green is being seen.
 */

#include <CircleDetection.h>

int main(int argc, char** argv) {
    // Setup your ROS node
    std::string node_name = "circle_detection";
    // Create an instance of your class
    CircleDetection green_recognition(argc, argv, node_name);
    // Start up ROS, this will continue to run until the node is killed
    ros::spin();
    // Once the node stops, return 0
    return 0;
}