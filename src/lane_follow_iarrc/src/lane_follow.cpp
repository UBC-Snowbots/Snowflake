/*
 * Created By: Raad Khan
 * Created On: April 23, 2017
 * Description: Takes in an image feed and uses LineDetect to generate
 *              lane lines and a destination point, then broadcasts a
 *              Twist message to stay within the lanes.
 */

#include "LaneFollow.h"

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
