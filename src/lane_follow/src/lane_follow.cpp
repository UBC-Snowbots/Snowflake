/*
 * Created By: Raad Khan
 * Created On: April 23, 2017
 * Description: Gets angle of lane lines point of intersection
 *              and broadcasts a recommended Twist message.
 */

#include <LaneFollow.h>
// #include <ros/ros.h>

using namespace std;

int main(int argc, char** argv) {
    // Setup your ROS node
    std::string node_name = "lane_follow";
    // Create an instance of your class
    LaneFollow lane_follow(argc, argv, node_name);
    // Start up ROS, this will continue to run until the node is killed
    ros::spin();
    // Once the node stops, return 0
    return 0;
}

