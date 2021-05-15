/*
 * Created By: Vijeeth Vijhaipranith
 * Created On: Oct 30th, 2020
 * Description: This node subscribes to a topic publishing Geometry Twist
 * messages that indicate the direction to move the rover in,
 *              and publishes velocity values to the left and right wheels of
 * the rover.
 *              The velocity of the left wheels is published to the topic
 * "/integration_node/lwheels_pub_topic"
 *              The velocity of the right wheels is published to the topic
 * "/integration_node/rwheels_pub_topic"
 */

#ifndef WHEEL_INTEGRATION_PACKAGE_MYNODE_H
#define WHEEL_INTEGRATION_PACKAGE_MYNODE_H

// STD Includes
#include <iostream>

// ROS Includes
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <std_msgs/String.h>

// Snowbots Includes
#include <sb_utils.h>

class MyClass {
  public:
    MyClass(
    int argc, char** argv, std::string node_name, float dist, float max_speed);
    float distBetweenWheels;
    float maximum_speed;

  private:
    /**
     * Callback function for when a new string is received
     *
     * @param msg the string received in the callback
     */
    void subscriberCallBack(const geometry_msgs::Twist::ConstPtr& msg);

    ros::Subscriber my_subscriber;
    ros::Publisher lwheels_publisher;
    ros::Publisher rwheels_publisher;
};
#endif // WHEEL_INTEGRATION_PACKAGE_MYNODE_H
