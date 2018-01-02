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
#include <GpsMover.h>
#include <sb_utils.h>

// ROS
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <tf/transform_datatypes.h>

class GpsDecision {
  public:
    GpsDecision(int argc, char** argv, std::string node_name);

  private:
    // callback for our destination waypoint
    void
    waypointCallback(const geometry_msgs::PointStamped::ConstPtr& waypoint);

    // Subscribes to the next waypoint the robot has to go to
    ros::Subscriber waypoint_subscriber;

    // Publishes a twist message telling the robot how to move
    ros::Publisher twist_publisher;

    // The class that generates our twist messages
    GpsMover mover;

    // The base frame of the robot ("base_link", "base_footprint", etc.)
    std::string base_frame;
    // The global frame ("map", "odom", etc.)
    std::string global_frame;
};

#endif // DECISION_GPS_DECISION_H
