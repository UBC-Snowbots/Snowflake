/*
 * Created By: Gareth Ellis
 * Created On: March 29th, 2018
 * Description: A node the that translates the encoder readings (published in a
 *              JointState message as # of ticks) to a ROS Odometry message
 *              Originally designed for use with the Phidget 1047 encoder
 */

#include <EncoderOdometryNode.h>


int main(int argc, char **argv){
    std::string node_name = "encoder_to_odometry";

    // Setup our Encoder -> Odometry conversion node
    EncoderOdometryNode encoder_odometry_node(argc, argv, node_name);

    // The rate at which to publish estimated Odometry messages (in hz).
    // We use this because we want to make velocity predictions based off
    // several sets of encoder readings (to allow for averaging and reduce
    // estimation error)
    int odometry_estimate_refresh_rate;

    // Get the desired refresh rate of the Odometry message estimation
    ros::NodeHandle private_nh("~");
    SB_getParam(private_nh, "odom_msg_refresh_rate", odometry_estimate_refresh_rate, 10);

    // Set the refresh rate to the desired odom estimation rate
    ros::Rate loop_rate(odometry_estimate_refresh_rate);

    // Keep making estimates until we're told to stop
    while (ros::ok()){
        encoder_odometry_node.publishEstimatedOdomMsg();
        ros::spinOnce();
        loop_rate.sleep();
    }

    // Start up ros. This will continue to run until the node is killed
    ros::spin();

    // Once the node stops, return 0
    return 0;
}