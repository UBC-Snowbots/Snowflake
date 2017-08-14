/*
 * Created By: Gareth Ellis
 * Created On: May 18, 2017
 * Description: The Decision Node for GPS, takes in a point relative to
 *              the robots location and heading and broadcasts a
 *              recommended twist message. Revised to use TF's
 */

#ifndef DECISION_GPS_DECISION_H
#define DECISION_GPS_DECISION_H

// STD
#include <iostream>

// Snowbots
#include <sb_utils.h>
#include <GpsMover.h>

// ROS
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_datatypes.h>
#include <std_msgs/Float32.h>


class GpsDecision2 {
public:
    GpsDecision2(int argc, char **argv, std::string node_name);

private:
    //callback for our destination waypoint
    void waypointCallback(const geometry_msgs::PointStamped::ConstPtr& waypoint);

    ros::Subscriber waypoint_subscriber; // Subscribes to the next waypoint the robot has to go to

    ros::Publisher twist_publisher; // Publishes a twist message telling the robot how to move

    GpsMover mover; // The class that generates our twist messages


    std::string base_frame; // The base frame of the robot ("base_link", "base_footprint", etc.)
    std::string global_frame; // The global frame ("map", "odom", etc.)
};

#endif //DECISION_GPS_DECISION_H
