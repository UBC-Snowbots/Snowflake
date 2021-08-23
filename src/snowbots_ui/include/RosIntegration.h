/*
 * Created By: Adam Nguyen
 * Created On: August 21st, 2021
 * Snowbots UI
 */

#ifndef RosIntegration_H
#define RosIntegration_H
#include "geometry_msgs/Twist.h"
#include "ros/ros.h"
#include <string>

static geometry_msgs::Twist twist_message_controller;
static geometry_msgs::Twist twist_message_left;
static geometry_msgs::Twist twist_message_right;

class RosIntegration {
  public:
    RosIntegration();
    ~RosIntegration();

    // twist topic

    static void
    twist_controller_callback(const geometry_msgs::Twist::ConstPtr& twist_msg) {
        twist_message_controller = *twist_msg;
    }

    static void
    twist_left_callback(const geometry_msgs::Twist::ConstPtr& twist_msg) {
        twist_message_left = *twist_msg;
    }

    static void
    twist_right_callback(const geometry_msgs::Twist::ConstPtr& twist_msg) {
        twist_message_right = *twist_msg;
    }

    void twist_subscriber() {
        ros::Rate loop_rate(5);
        twist_controller_sub =
        n->subscribe("/cmd_vel", 1000, twist_controller_callback);
        twist_left_sub = n->subscribe(
        "/integration_node/lwheels_pub_topic", 1000, twist_left_callback);
        twist_right_sub = n->subscribe(
        "/integration_node/rwheels_pub_topic", 1000, twist_right_callback);
        loop_rate.sleep();
        ros::spinOnce();
    }

  private:
    ros::NodeHandle* n;

    // twist topics
    ros::Subscriber twist_controller_sub;
    ros::Subscriber twist_left_sub;
    ros::Subscriber twist_right_sub;
};

#endif // RosIntegration_H
