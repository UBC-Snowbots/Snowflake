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
    left_encoder_num_ticks_curr(std::experimental::optional<int>{}),
    right_encoder_num_ticks_curr(std::experimental::optional<int>{}),
    left_encoder_num_ticks_prev(std::experimental::optional<int>{}),
    right_encoder_num_ticks_prev(std::experimental::optional<int>{}),
    last_estimate(generateAllZeroOdometryMessage())
{
    // Setup NodeHandles
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    // Get Params
    SB_getParam(private_nh, "left_encoder_joint_name", left_encoder_joint_name, std::string("left_encoder"));
    SB_getParam(private_nh, "right_encoder_joint_name", right_encoder_joint_name, std::string("right_encoder"));
    SB_getParam(private_nh, "reverse_left_encoder", reverse_left_encoder, false);
    SB_getParam(private_nh, "reverse_right_encoder", reverse_right_encoder, false);
    SB_getParam(private_nh, "wheel_radius", wheel_radius, 0.1);
    SB_getParam(private_nh, "wheelbase_length", wheelbase_length, 0.7);
    SB_getParam(private_nh, "ticks_per_rotation", ticks_per_rotation, 1024);
    SB_getParam(private_nh, "odom_frame", odom_frame_id, std::string("encoder"));
    // TODO: Figure out actually reasonable variances
    SB_getParam(private_nh, "left_encoder_variance", left_encoder_variance, 0.1);
    SB_getParam(private_nh, "right_encoder_variance", right_encoder_variance, 0.1);
    double odometry_estimate_refresh_rate;
    SB_getParam(private_nh, "odom_msg_refresh_rate", odometry_estimate_refresh_rate, 10.0);

    // Setup Subscriber(s)
    joint_state_subscriber = nh.subscribe(std::string("/encoders/joint_states"), 10, &EncoderOdometryNode::encoderJointStateCallback, this);
    reset_subscriber = private_nh.subscribe(std::string("reset"), 1, &EncoderOdometryNode::resetCallback, this);

    // Setup Publisher(s)
    // Note: We publish the encoder odometry on the global namespace under the
    // `/encoders` namespace so the odometry is neatly associated with
    // the encoders
    std::string odom_estimate_topic_name = nh.resolveName("/encoders/odom");
    odom_estimate_publisher = nh.advertise<nav_msgs::Odometry>(odom_estimate_topic_name, 10);

    // Setup the timer that will publish our Odometry estimates at the set frequency
    // NOTE: This acts similairly to a callback, expect that instead of being
    // triggered when we receive a message, it is triggered after a set period
    // of time
    ros::Duration odom_estimate_period(1 / odometry_estimate_refresh_rate);
    odom_estimate_timer = private_nh.createTimer(odom_estimate_period,
                                                 &EncoderOdometryNode::publishEstimatedOdomMsg, this);
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
                if (reverse_left_encoder){
                    *left_encoder_num_ticks_curr *= -1;
                }
            }
        } else if (joint_state.name[i] == right_encoder_joint_name){
            // We expect that there will be a value in the `position` array
            if (joint_state.position.size() < (i+1)){
                // Something is wrong and we can't get an encoder reading
                ROS_WARN("No \"position\" field for right encoder joint name found, so can't get number of ticks");
            } else {
                right_encoder_num_ticks_curr = (int)joint_state.position[i];
                if (reverse_right_encoder){
                    *right_encoder_num_ticks_curr *= -1;
                }
            }
        }
    }

}

void EncoderOdometryNode::resetCallback(std_msgs::Empty::ConstPtr empty_msg) {
    // Reset encoder tick counts to "empty"
    left_encoder_num_ticks_curr = std::experimental::optional<int>{};
    right_encoder_num_ticks_curr = std::experimental::optional<int>{};
    left_encoder_num_ticks_prev = std::experimental::optional<int>{};
    right_encoder_num_ticks_prev = std::experimental::optional<int>{};

    // Reset our last estimate
    last_estimate = generateAllZeroOdometryMessage();
}

void
EncoderOdometryNode::publishEstimatedOdomMsg(const ros::TimerEvent &timer_event) {
    // Check to make sure that we have some counts for both the left and right
    // encoders
    if (!left_encoder_num_ticks_curr || !right_encoder_num_ticks_curr){
        // If we don't have either the left or right encoder ticks, just return
        return;
    }

    // Checks if we've made an estimate before
    if (!left_encoder_num_ticks_prev || !right_encoder_num_ticks_prev){
        ROS_DEBUG("Got first ticks from encoders");
        // If we haven't, save the current state as the previous state
        // and just return. The first actual estimate will be made at the
        // next call to this function
        left_encoder_num_ticks_prev = left_encoder_num_ticks_curr;
        right_encoder_num_ticks_prev = right_encoder_num_ticks_curr;

        last_estimate.header.stamp = ros::Time::now();

        return;
    }

    // All math taken from:
    // https://www.cs.cmu.edu/afs/cs.cmu.edu/academic/class/16311/www/s07/labs/NXTLabs/Lab%203.html

    double dt = ros::Time::now().toSec() - last_estimate.header.stamp.toSec();

    // left and right wheel velocities (in meters/s)
    int left_encoder_ticks = *left_encoder_num_ticks_curr - *left_encoder_num_ticks_prev;
    int right_encoder_ticks = *right_encoder_num_ticks_curr - *right_encoder_num_ticks_prev;
    double v_l = (left_encoder_ticks)/dt * (2*M_PI)/ticks_per_rotation * wheel_radius;
    double v_r = (right_encoder_ticks)/dt * (2*M_PI)/ticks_per_rotation * wheel_radius;

    // linear and angular velocity
    double v = (v_r + v_l) / 2;
    double w = (v_r - v_l) / wheelbase_length;

    //x and y velocity
    double x_vel = v*cos(w);
    double y_vel = v*sin(w);

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
    last_estimate.pose.covariance = calculatePoseCovariance(left_encoder_ticks, right_encoder_ticks, prev_yaw);
    last_estimate.twist.twist.linear.x = x_vel;
    last_estimate.twist.twist.linear.y = y_vel;
    last_estimate.twist.twist.angular.z = w;
    last_estimate.header.stamp = ros::Time::now();
    last_estimate.header.frame_id = odom_frame_id;

    // Save the current number of ticks as the "previous" number for the
    // next time we estimate
    left_encoder_num_ticks_prev = left_encoder_num_ticks_curr;
    right_encoder_num_ticks_prev = right_encoder_num_ticks_curr;

    // Publish our estimate
    odom_estimate_publisher.publish(last_estimate);
}

nav_msgs::Odometry EncoderOdometryNode::generateAllZeroOdometryMessage() {
    nav_msgs::Odometry odom_msg;
    odom_msg.pose.pose.position.x = 0;
    odom_msg.pose.pose.position.y = 0;
    odom_msg.pose.pose.orientation = tf::createQuaternionMsgFromYaw(0);
    odom_msg.twist.twist.linear.x = 0;
    odom_msg.twist.twist.linear.y = 0;
    odom_msg.twist.twist.linear.z = 0;
    odom_msg.twist.twist.angular.x = 0;
    odom_msg.twist.twist.angular.y = 0;
    odom_msg.twist.twist.angular.z = 0;

    return odom_msg;
}

geometry_msgs::PoseWithCovariance::_covariance_type EncoderOdometryNode::calculatePoseCovariance(
        int left_wheel_ticks,
        int right_wheel_ticks,
        double prev_yaw) {

    // All math taken from: http://digital.csic.es/bitstream/10261/30337/1/Onto%20computing.pdf
    // namely pages 1342 and 1343

    // Compute displacement of the left and right wheels (relative to last estimate)
    double wheel_circumference = 2 * M_PI * wheel_radius;
    double d_l = left_wheel_ticks * wheel_circumference / ticks_per_rotation;
    double d_r = right_wheel_ticks * wheel_circumference / ticks_per_rotation;

    // TODO: Detailed doc comments here
    // let p_t be the change from the previous state (x,y,theta): p_t = [x_t; y_t; theta_t]

    // Cov[p_t] = E[p_t * transpose(p_t)] - E[p_t] * E[transpose(p_t)]

    // Compute E[p_t * transpose(p_t)] - E[p_t]
    double k_1 = std::pow(d_r, 2) + std::pow(d_l, 2);
    double k_2 = std::pow(d_r, 2) - std::pow(d_l, 2);
    double k_3 = d_r * d_l;
    double k_4 = left_encoder_variance + right_encoder_variance;
    double k_5 = left_encoder_variance - right_encoder_variance;

    // let c_ij be the element at (i,j) in E[p_t * transpose(p_t)] - E[p_t]
    double c_11 = (k_1 + 2*k_3 + k_4)/4 * cos(prev_yaw) * cos(prev_yaw);
    double c_12 = (k_1 + 2*k_3 + k_4)/4 * cos(prev_yaw) * sin(prev_yaw);
    double c_13 = (k_2 + k_5)/(2 * wheelbase_length) * cos(prev_yaw);
    double c_21 = c_12;
    double c_22 = (k_1 + 2*k_3 + k_4)/4 * sin(prev_yaw) * sin(prev_yaw);
    double c_23 = (k_2 + k_5)/(2 * wheelbase_length) * sin(prev_yaw);
    double c_31 = c_13;
    double c_32 = c_23;
    double c_33 = (k_2 + k_5)/(2 * wheelbase_length) * sin(prev_yaw);

    // Compute E[p_t] * E[transpose(p_t)]
    double k_6 = (d_r + d_l)/2 * cos(prev_yaw);
    double k_7 = (d_r + d_l)/2 * sin(prev_yaw);
    double k_8 = (d_r - d_l)/wheelbase_length;

    // let d_ij be the element at (i,j) in E[p_t] * E[transpose(p_t)]
    double d_11 = k_6*k_6;
    double d_12 = k_6*k_7;
    double d_13 = k_6*k_8;
    double d_21 = d_12;
    double d_22 = k_7*k_7;
    double d_23 = k_7*k_8;
    double d_31 = d_13;
    double d_32 = d_23;
    double d_33 = k_8 * k_8;

    // Cov[p_t] = E[p_t * transpose(p_t)] - E[p_t] * E[transpose(p_t)]
    // Let cov_A_B be the covariance between A and B
    double cov_x_x = c_11 - d_11;
    double cov_y_y = c_22 - d_22;
    double cov_theta_theta = c_33 - d_33;
    double cov_x_y = c_12 - d_12;
    double cov_y_x = c_21 - d_21;
    double cov_x_theta = c_13 - d_13;
    double cov_theta_x = c_31 - d_31;
    double cov_y_theta = c_23 - d_23;
    double cov_theta_y = c_32 - d_32;

    // Set the covariances at the correct places in the array
    // (since the array also includes linear z and angular x and y)
    geometry_msgs::PoseWithCovariance::_covariance_type covariances;
    covariances.fill(0);
    covariances[0*6 + 0] = cov_x_x;
    covariances[0*6 + 1] = cov_x_y;
    covariances[0*6 + 5] = cov_x_theta;
    covariances[1*6 + 0] = cov_y_x;
    covariances[1*6 + 1] = cov_y_y;
    covariances[1*6 + 2] = cov_y_theta;
    covariances[5*6 + 0] = cov_theta_x;
    covariances[5*6 + 1] = cov_theta_y;
    covariances[5*6 + 5] = cov_theta_theta;

    return covariances;
}

