/*
 * Created By: Adam Nguyen
 * Created On: August 21st, 2021
 * Snowbots UI Tester. This publishes random twist messages to the "/cmd_vel"
 * topic in place of the procontroller package
 */

#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <stdlib.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "ui_tester");

    ros::NodeHandle n;

    ros::Publisher pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);

    ros::Rate rate(10);

    srand(time(0));

    while (ros::ok()) {
        // Declares the message to be sent
        geometry_msgs::Twist msg;

        // Random x value between -2 and 2
        msg.linear.x = 4 * double(rand()) / double(RAND_MAX) - 2;

        // Random z value between -3 and 3
        msg.angular.z = 6 * double(rand()) / double(RAND_MAX) - 3;

        // Publish the message
        pub.publish(msg);

        // Delays until it is time to send another message
        rate.sleep();
    }

    return 0;
}
