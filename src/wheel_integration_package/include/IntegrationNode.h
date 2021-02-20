/*
 * Created By: Gareth Ellis
 * Created On: July 16th, 2016
 * Description: An example node that subscribes to a topic publishing strings,
 *              and re-publishes everything it receives to another topic with
 *              a "!" at the end
 */

#ifndef WHEEL_INTEGRATION_PACKAGE_MYNODE_H
#define WHEEL_INTEGRATION_PACKAGE_MYNODE_H

// STD Includes
#include <iostream>

// ROS Includes
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include <ros/ros.h>

// Snowbots Includes
#include <sb_utils.h>

class MyClass {
public:
    MyClass(int argc, char **argv, std::string node_name, float dist, float max_speed);
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
#endif //WHEEL_INTEGRATION_PACKAGE_MYNODE_H
