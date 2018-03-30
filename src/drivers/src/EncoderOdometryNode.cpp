/*
 * Created By: Gareth Ellis
 * Created On: March 29th, 2018
 * Description: A node the that translates the encoder readings (published in a
 *              JointState message as # of ticks) to a ROS Odometry message
 *              Originally designed for use with the Phidget 1047 encoder
 */

// ROS Includes
#include <tf/transform_datatypes.h>

// Snowbots Includes
#include "EncoderOdometryNode.h"
#include <sb_utils.h>


EncoderOdometryNode::EncoderOdometryNode(int argc, char **argv, std::string node_name):
    // Set current encoder ticks to "empty" as we haven't received any yet
    left_encoder_num_ticks_curr({}),
    right_encoder_num_ticks_curr({}),
    left_encoder_num_ticks_prev({}),
    right_encoder_num_ticks_prev({})
{
    // Setup NodeHandles
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    // Get Params
    SB_getParam(private_nh, "left_encoder_joint_name", left_encoder_joint_name, std::string("left_encoder"));
    SB_getParam(private_nh, "right_encoder_joint_name", right_encoder_joint_name, std::string("right_encoder"));
    SB_getParam(private_nh, "wheel_radius", wheel_radius, 0.1);
    SB_getParam(private_nh, "wheelbase_length", wheelbase_length, 0.7);
    SB_getParam(private_nh, "ticks_per_rotation", ticks_per_rotation, 1024);


    // Setup Subscriber(s)
    joint_state_subscriber = nh.subscribe(std::string("/encoders/joint_states"), 1, &EncoderOdometryNode::encoderJointStateCallback, this);
    reset_subscriber = private_nh.subscribe(std::string("reset"), 1, &EncoderOdometryNode::resetCallback, this);

    // Setup Publisher(s)
    std::string odom_estimate_topic_name = nh.resolveName("/encoders/odom");
    odom_estimate_publisher = nh.advertise<nav_msgs::Odometry>(odom_estimate_topic_name, 10);

    // Initialise our estimate
    last_estimate.pose.pose.position.x = 0;
    last_estimate.pose.pose.position.y = 0;
    last_estimate.pose.pose.orientation = tf::createQuaternionMsgFromYaw(0);
}

void EncoderOdometryNode::encoderJointStateCallback(sensor_msgs::JointState::ConstPtr joint_state_ptr) {
    sensor_msgs::JointState joint_state = *joint_state_ptr;

    // Try and find the number of ticks at the joints specified for the left
    // and right encoders
    // NOTE: We expect both encoder readings to come in under a single
    // JointState message
    for (int i = 0; i < joint_state.name.size(); i++){
        if (joint_state.name[i] == left_encoder_joint_name){
            // We expect that there will be a value in the `position` array
            if (joint_state.position.size() < (i+1)){
                // Something is wrong and we can't get an encoder reading
                ROS_WARN("No \"position\" field for left encoder joint name found, so can't get number of ticks");
            } else {
                left_encoder_num_ticks_curr = (int)joint_state.position[i];
            }
        } else if (joint_state.name[i] == right_encoder_joint_name){
            // We expect that there will be a value in the `position` array
            if (joint_state.position.size() < (i+1)){
                // Something is wrong and we can't get an encoder reading
                ROS_WARN("No \"position\" field for right encoder joint name found, so can't get number of ticks");
            } else {
                right_encoder_num_ticks_curr = (int)joint_state.position[i];
            }
        }
    }

}

void EncoderOdometryNode::resetCallback(std_msgs::Empty::ConstPtr empty_msg) {
    // Reset encoder tick counts to "empty"
    left_encoder_num_ticks_curr = {};
    right_encoder_num_ticks_curr = {};
    left_encoder_num_ticks_prev = {};
    right_encoder_num_ticks_prev = {};

    // Reset our last estimate
    last_estimate.pose.pose.position.x = 0;
    last_estimate.pose.pose.position.y = 0;
    last_estimate.pose.pose.orientation = tf::createQuaternionMsgFromYaw(0);
}

void EncoderOdometryNode::publishEstimatedOdomMsg() {
    // TODO: Delete me
    if (!left_encoder_num_ticks_curr || !right_encoder_num_ticks_curr){
        ROS_INFO_STREAM("Updating Estimate: No ticks received");
    } else {
        ROS_INFO_STREAM("Updating Estimate from: " << *left_encoder_num_ticks_curr << ", " << *right_encoder_num_ticks_curr);
    }

    // Check to make sure that we have some counts for both the left and right
    // encoders
    if (!left_encoder_num_ticks_curr || !right_encoder_num_ticks_curr){
        // If we don't have either the left or right encoder ticks, just return
        return;
    }

    // Checks if we've made an estimate before
    if (!left_encoder_num_ticks_prev || !right_encoder_num_ticks_prev){
        ROS_INFO("Got first ticks from encoders");
        // If we haven't, save the current state as the previous state
        // and just return. The first actual estimate will be made at the
        // next call to this function
        left_encoder_num_ticks_prev = left_encoder_num_ticks_curr;
        right_encoder_num_ticks_prev = right_encoder_num_ticks_curr;

        return;
    }

    // All math taken from:
    // https://www.cs.cmu.edu/afs/cs.cmu.edu/academic/class/16311/www/s07/labs/NXTLabs/Lab%203.html

    double dt = ros::Time::now().toSec() - last_estimate.header.stamp.toSec();

    // left and right wheel velocities (in radians/s)
    double v_l = (*left_encoder_num_ticks_curr - *left_encoder_num_ticks_prev)/dt * (2*M_PI)/ticks_per_rotation * wheel_radius;
    double v_r = (*right_encoder_num_ticks_curr - *right_encoder_num_ticks_prev)/dt * (2*M_PI)/ticks_per_rotation * wheel_radius;

    ROS_INFO_STREAM("v_l: " << v_l);
    ROS_INFO_STREAM("v_r: " << v_r);

    // linear and angular velocity
    double v = (v_r + v_l) / 2;
    double w = (v_r - v_l) / wheelbase_length;

    // position
    double prev_x = last_estimate.pose.pose.position.x;
    double prev_y = last_estimate.pose.pose.position.y;
    double prev_yaw = tf::getYaw(last_estimate.pose.pose.orientation);

    double k_00 = v*cos(prev_yaw);
    double k_01 = v*sin(prev_yaw);
    double k_02 = w;
    double k_10 = v*cos(prev_yaw + dt/2*k_02);
    double k_11 = v*sin(prev_yaw + dt/2*k_02);
    double k_12 = w;
    double k_20 = v*cos(prev_yaw + dt/2*k_12);
    double k_21 = v*sin(prev_yaw + dt/2*k_12);
    double k_22 = w;
    double k_30 = v*cos(prev_yaw + dt*k_22);
    double k_31 = v*sin(prev_yaw + dt*k_22);
    double k_32 = w;

    double x = prev_x + (dt/6) * (k_00 + 2*(k_10 + k_20) + k_30);
    double y = prev_y + (dt/6) * (k_01 + 2*(k_11 + k_21) + k_31);
    double yaw = prev_yaw + (dt/6) * (k_02 + 2*(k_12 + k_22) + k_32);

    // Update our estimate
    last_estimate.pose.pose.position.x = x;
    last_estimate.pose.pose.position.y = y;
    tf::quaternionTFToMsg(tf::createQuaternionFromYaw(yaw), last_estimate.pose.pose.orientation);
    last_estimate.header.stamp = ros::Time::now();

    // Save the current number of ticks as the "previous" number for the
    // next time we estimate
    left_encoder_num_ticks_prev = left_encoder_num_ticks_curr;
    right_encoder_num_ticks_prev = right_encoder_num_ticks_curr;

    // Publish our estimate
    odom_estimate_publisher.publish(last_estimate);

    // TODO: Delete me
    ROS_INFO_STREAM(last_estimate.pose.pose);
}

