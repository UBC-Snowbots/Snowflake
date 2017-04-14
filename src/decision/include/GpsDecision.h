/*
 * Created By: Gareth Ellis
 * Created On: April 1, 2017
 * Description: The Decision Node for GPS, takes in a point relative to
 *              the robots location and heading and broadcasts a
 *              recommended twist message
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


class GpsDecision {
public:
    GpsDecision(int argc, char **argv, std::string node_name);

private:
    //callback for the current location
    void currentLocationCallback(const geometry_msgs::Point::ConstPtr& current_location);
    //callback for the current heading
    void imuCallback(const std_msgs::Float32::ConstPtr &heading);
    //callback for our destination waypoint
    void waypointCallback(const geometry_msgs::Point::ConstPtr& waypoint);

    ros::Subscriber imu_subscriber;             // Subscribes to the current robot heading
    ros::Subscriber current_location_subscriber;    // Subscribes to the current robot location
    ros::Subscriber waypoint_subscriber;            // Subscribes to the next waypoint the robot has to go to

    ros::Publisher twist_publisher;     // Publishes a twist message telling the robot how to move

    double current_heading;                 // The current robot heading (in radians)
    geometry_msgs::Point current_location; // The current location of the robot

    GpsMover mover;    // The class that generates our twist messages
};

#endif //DECISION_GPS_DECISION_H
