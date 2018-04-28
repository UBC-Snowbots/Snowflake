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

EncoderOdometryNode::EncoderOdometryNode(int argc,
                                         char** argv,
                                         std::string node_name)
  : // Set current encoder ticks to "empty" as we haven't received any yet
    left_encoder_num_ticks_curr(std::experimental::optional<int>{}),
    right_encoder_num_ticks_curr(std::experimental::optional<int>{}),
    left_encoder_num_ticks_prev(std::experimental::optional<int>{}),
    right_encoder_num_ticks_prev(std::experimental::optional<int>{}),
    last_estimate(generateAllZeroOdometryMessage()) {
    // Setup NodeHandles
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    // Get Params
    SB_getParam(private_nh,
                "left_encoder_joint_name",
                left_encoder_joint_name,
                std::string("left_encoder"));
    SB_getParam(private_nh,
                "right_encoder_joint_name",
                right_encoder_joint_name,
                std::string("right_encoder"));
    SB_getParam(
    private_nh, "reverse_left_encoder", reverse_left_encoder, false);
    SB_getParam(
    private_nh, "reverse_right_encoder", reverse_right_encoder, false);
    SB_getParam(private_nh, "wheel_radius", wheel_radius, 0.1);
    SB_getParam(private_nh, "wheelbase_length", wheelbase_length, 0.7);
    SB_getParam(private_nh, "ticks_per_rotation", ticks_per_rotation, 1024);
    SB_getParam(
    private_nh, "odom_frame", odom_frame_id, std::string("encoder"));
    // The variance of the x, y, and theta measurements.
    // These values should be determined experimentally
    // TODO: Link to markdown file explaining how to determine these
    double x_variance;
    double y_variance;
    double theta_variance;
    SB_getParam(private_nh, "x_variance", x_variance, 0.1);
    SB_getParam(private_nh, "y_variance", y_variance, 0.1);
    SB_getParam(private_nh, "theta_variance", theta_variance, 0.1);
    double odometry_estimate_refresh_rate;
    SB_getParam(
    private_nh, "odom_msg_refresh_rate", odometry_estimate_refresh_rate, 10.0);

    // Setup Subscriber(s)
    joint_state_subscriber =
    nh.subscribe(std::string("/encoders/joint_states"),
                 10,
                 &EncoderOdometryNode::encoderJointStateCallback,
                 this);
    reset_subscriber = private_nh.subscribe(
    std::string("reset"), 1, &EncoderOdometryNode::resetCallback, this);

    // Setup Publisher(s)
    // Note: We publish the encoder odometry on the global namespace under the
    // `/encoders` namespace so the odometry is neatly associated with
    // the encoders
    std::string odom_estimate_topic_name = nh.resolveName("/encoders/odom");
    odom_estimate_publisher =
    nh.advertise<nav_msgs::Odometry>(odom_estimate_topic_name, 10);

    // Setup the timer that will publish our Odometry estimates at the set
    // frequency
    // NOTE: This acts similairly to a callback, expect that instead of being
    // triggered when we receive a message, it is triggered after a set period
    // of time
    ros::Duration odom_estimate_period(1 / odometry_estimate_refresh_rate);
    odom_estimate_timer = private_nh.createTimer(
    odom_estimate_period, &EncoderOdometryNode::publishEstimatedOdomMsg, this);

    // Set the covariance on our Odometry messages. Since this is static, we
    // only need to set it once
    // We set all values to 0, expect for the diagonal, on which we set the
    // variance for
    // linear.x, linear.y, and angular.z, and set all other values to a very
    // high number (indicating that
    // we don't have a measurement for that part of the message)
    std::fill(last_estimate.pose.covariance.begin(),
              last_estimate.pose.covariance.end(),
              0);
    last_estimate.pose.covariance[6 * 0 + 0] = x_variance;
    last_estimate.pose.covariance[6 * 1 + 1] = y_variance;
    last_estimate.pose.covariance[6 * 2 + 2] =
    std::numeric_limits<double>::max();
    last_estimate.pose.covariance[6 * 3 + 3] =
    std::numeric_limits<double>::max();
    last_estimate.pose.covariance[6 * 4 + 4] =
    std::numeric_limits<double>::max();
    last_estimate.pose.covariance[6 * 5 + 5] = theta_variance;
}

void EncoderOdometryNode::encoderJointStateCallback(
sensor_msgs::JointState::ConstPtr joint_state_ptr) {
    sensor_msgs::JointState joint_state = *joint_state_ptr;

    // Try and find the number of ticks at the joints specified for the left
    // and right encoders
    // NOTE: We expect both encoder readings to come in under a single
    // JointState message
    for (int i = 0; i < joint_state.name.size(); i++) {
        if (joint_state.name[i] == left_encoder_joint_name) {
            // We expect that there will be a value in the `position` array
            if (joint_state.position.size() < (i + 1)) {
                // Something is wrong and we can't get an encoder reading
                ROS_WARN(
                "No \"position\" field for left encoder joint name found, so "
                "can't get number of ticks");
            } else {
                left_encoder_num_ticks_curr = (int) joint_state.position[i];
                if (reverse_left_encoder) {
                    *left_encoder_num_ticks_curr *= -1;
                }
            }
        } else if (joint_state.name[i] == right_encoder_joint_name) {
            // We expect that there will be a value in the `position` array
            if (joint_state.position.size() < (i + 1)) {
                // Something is wrong and we can't get an encoder reading
                ROS_WARN(
                "No \"position\" field for right encoder joint name found, so "
                "can't get number of ticks");
            } else {
                right_encoder_num_ticks_curr = (int) joint_state.position[i];
                if (reverse_right_encoder) {
                    *right_encoder_num_ticks_curr *= -1;
                }
            }
        }
    }
}

void EncoderOdometryNode::resetCallback(std_msgs::Empty::ConstPtr empty_msg) {
    // Reset encoder tick counts to "empty"
    left_encoder_num_ticks_curr  = std::experimental::optional<int>{};
    right_encoder_num_ticks_curr = std::experimental::optional<int>{};
    left_encoder_num_ticks_prev  = std::experimental::optional<int>{};
    right_encoder_num_ticks_prev = std::experimental::optional<int>{};

    // Reset our last estimate
    last_estimate = generateAllZeroOdometryMessage();
}

void EncoderOdometryNode::publishEstimatedOdomMsg(
const ros::TimerEvent& timer_event) {
    // Check to make sure that we have some counts for both the left and right
    // encoders
    if (!left_encoder_num_ticks_curr || !right_encoder_num_ticks_curr) {
        // If we don't have either the left or right encoder ticks, just return
        return;
    }

    // Checks if we've made an estimate before
    if (!left_encoder_num_ticks_prev || !right_encoder_num_ticks_prev) {
        ROS_DEBUG("Got first ticks from encoders");
        // If we haven't, save the current state as the previous state
        // and just return. The first actual estimate will be made at the
        // next call to this function
        left_encoder_num_ticks_prev  = left_encoder_num_ticks_curr;
        right_encoder_num_ticks_prev = right_encoder_num_ticks_curr;

        // Save the time so that next time we get ticks, we can
        // generate a state estimate
        last_estimate.header.stamp = ros::Time::now();

        return;
    }

    // All math taken from:
    // https://www.cs.cmu.edu/afs/cs.cmu.edu/academic/class/16311/www/s07/labs/NXTLabs/Lab%203.html

    double dt = ros::Time::now().toSec() - last_estimate.header.stamp.toSec();

    // left and right wheel velocities (in meters/s)
    int left_encoder_ticks =
    *left_encoder_num_ticks_curr - *left_encoder_num_ticks_prev;
    int right_encoder_ticks =
    *right_encoder_num_ticks_curr - *right_encoder_num_ticks_prev;
    double v_l =
    (left_encoder_ticks) / dt * (2 * M_PI) / ticks_per_rotation * wheel_radius;
    double v_r =
    (right_encoder_ticks) / dt * (2 * M_PI) / ticks_per_rotation * wheel_radius;

    // linear and angular velocity
    double v = (v_r + v_l) / 2;
    double w = (v_r - v_l) / wheelbase_length;

    // x and y velocity
    double x_vel = v * cos(w);
    double y_vel = v * sin(w);

    // position
    double prev_x   = last_estimate.pose.pose.position.x;
    double prev_y   = last_estimate.pose.pose.position.y;
    double prev_yaw = tf::getYaw(last_estimate.pose.pose.orientation);

    double k_00 = v * cos(prev_yaw);
    double k_01 = v * sin(prev_yaw);
    double k_02 = w;
    double k_10 = v * cos(prev_yaw + dt / 2 * k_02);
    double k_11 = v * sin(prev_yaw + dt / 2 * k_02);
    double k_12 = w;
    double k_20 = v * cos(prev_yaw + dt / 2 * k_12);
    double k_21 = v * sin(prev_yaw + dt / 2 * k_12);
    double k_22 = w;
    double k_30 = v * cos(prev_yaw + dt * k_22);
    double k_31 = v * sin(prev_yaw + dt * k_22);
    double k_32 = w;

    double x   = prev_x + (dt / 6) * (k_00 + 2 * (k_10 + k_20) + k_30);
    double y   = prev_y + (dt / 6) * (k_01 + 2 * (k_11 + k_21) + k_31);
    double yaw = prev_yaw + (dt / 6) * (k_02 + 2 * (k_12 + k_22) + k_32);

    // Update our estimate
    last_estimate.pose.pose.position.x = x;
    last_estimate.pose.pose.position.y = y;
    tf::quaternionTFToMsg(tf::createQuaternionFromYaw(yaw),
                          last_estimate.pose.pose.orientation);
    last_estimate.twist.twist.linear.x  = x_vel;
    last_estimate.twist.twist.linear.y  = y_vel;
    last_estimate.twist.twist.angular.z = w;
    last_estimate.header.stamp          = ros::Time::now();
    last_estimate.header.frame_id       = odom_frame_id;

    // Save the current number of ticks as the "previous" number for the
    // next time we estimate
    left_encoder_num_ticks_prev  = left_encoder_num_ticks_curr;
    right_encoder_num_ticks_prev = right_encoder_num_ticks_curr;

    // Publish our estimate
    odom_estimate_publisher.publish(last_estimate);
}

nav_msgs::Odometry EncoderOdometryNode::generateAllZeroOdometryMessage() {
    nav_msgs::Odometry odom_msg;
    odom_msg.pose.pose.position.x  = 0;
    odom_msg.pose.pose.position.y  = 0;
    odom_msg.pose.pose.orientation = tf::createQuaternionMsgFromYaw(0);
    odom_msg.twist.twist.linear.x  = 0;
    odom_msg.twist.twist.linear.y  = 0;
    odom_msg.twist.twist.linear.z  = 0;
    odom_msg.twist.twist.angular.x = 0;
    odom_msg.twist.twist.angular.y = 0;
    odom_msg.twist.twist.angular.z = 0;

    return odom_msg;
}
