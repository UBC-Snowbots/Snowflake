/*
 * Created By: Gareth Ellis
 * Created On: March 29th, 2018
 * Description: A node the that translates the encoder readings (published in a
 *              JointState message as # of ticks) to a ROS Odometry message
 *              Originally designed for use with the Phidget 1047 encoder
 */

#include <EncoderOdometryNode.h>

int main(int argc, char** argv) {
    std::string node_name = "encoder_to_odometry";

    // Setup our Encoder -> Odometry conversion node
    EncoderOdometryNode encoder_odometry_node(argc, argv, node_name);

    // Start up ros. This will continue to run until the node is killed
    ros::spin();

    // Once the node stops, return 0
    return 0;
}