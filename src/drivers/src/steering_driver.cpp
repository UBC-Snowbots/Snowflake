/*
 * Created By: Gareth Ellis
 * Created On: April 16, 2017
 * Description: This node is responsible for passing twist messages received
 *              over serial to the Arduino controlling the robot
 */

#include <SteeringDriver.h>
#include <ros/ros.h>

int main(int argc, char **argv){
    // Setup your ROS node
    std::string node_name = "steering_driver";

    // Create an instance of your class
    SteeringDriver steering_driver(argc, argv, node_name);

    // Start up ros. This will continue to run until the node is killed
    ros::spin();

    // Once the node stops, return 0
    return 0;
}
