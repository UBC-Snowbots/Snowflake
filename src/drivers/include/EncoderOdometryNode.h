/*
 * Created By: Gareth Ellis
 * Created On: March 29th, 2018
 * Description: A node the that translates the encoder readings (published in a
 *              JointState message as # of ticks) to a ROS Odometry message
 *              Originally designed for use with the Phidget 1047 encoder
 */

#ifndef DRIVERS_ENCODER_ODOMETRY_NODE_H
#define DRIVERS_ENCODER_ODOMETRY_NODE_H

// STD Includes
#include <iostream>

// ROS Includes
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

// Snowbots Includes
#include <sb_utils.h>

class EncoderOdometryNode {
public:
    EncoderOdometryNode(int argc, char **argv, std::string node_name);

    /**
     * Publishes an estimate of the current state of the robot as an Odometry msg
     *
     * Estimates are based on current encoder wheel count. Math used may be
     * found here:
     * https://www.cs.cmu.edu/afs/cs.cmu.edu/academic/class/16311/www/s07/labs/NXTLabs/Lab%203.html
     */
    void publishEstimatedOdomMsg();

private:

    /**
     * The callback function for the JointState messages from the Encoders
     *
     * Updates the current encoder counts for the left and/or right encoders
     *
     * @param joint_state_ptr
     */
    void encoderJointStateCallback(sensor_msgs::JointState::ConstPtr joint_state_ptr);

    // The subscriber to the JointState messages from the Encoders containing
    // the # of ticks they have counted
    ros::Subscriber joint_state_subscriber;

    // The publisher that publishes our Odometry estimates
    ros::Publisher odom_estimate_publisher;

    // The joint names representing the left and right encoders
    std::string left_encoder_joint_name;
    std::string right_encoder_joint_name;

    // The current number of encoder ticks for the left and right encoders
    // (-1 if we haven't gotten any ticks yet)
    int left_encoder_num_ticks_curr;
    int right_encoder_num_ticks_curr;

    // The number of encoder ticks for the left and right encoders at the last
    // time we made an Odometry estimate (-1 if we've never made an estimate)
    int left_encoder_num_ticks_prev;
    int right_encoder_num_ticks_prev;

    // The last odometry estimate we made
    nav_msgs::Odometry last_estimate;

    // The radius of the robots wheels
    double wheel_radius;

    // The length of the wheelbase of the robot (the distance between the
    // two wheels)
    double wheelbase_length;
};
#endif //DRIVERS_ENCODER_ODOMETRY_NODE_H
