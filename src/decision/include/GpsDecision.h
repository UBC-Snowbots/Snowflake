/*
 * Created By: YOUR NAME HERE
 * Created On: September 22, 2016
 * Description: The Decision Node for GPS, takes in a point relative to
 *              the robots location and heading and broadcasts a
 *              recommended twist message
 */

#ifndef DECISION_GPS_DECISION_H
#define DECISION_GPS_DECISION_H

#include <iostream>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_datatypes.h>


class Mover {
public:
    /**
     * Constructor for a mover object
     *
     * @param distance_factor the weight of the distance to the gps waypoint in the movement command
     * @param heading_factor the weight of the robots heading to the gps waypoint in the movement command
     */
    Mover(double distance_factor, double heading_factor);

    /**
     * An empty constructor will default to factors of 1
     */
    Mover() : Mover(1,1) {};

    /**
     * @param distance_factor the value to set the distance factor to
     * @param heading_factor the value to set the heading factor to
     */
    void setFactors(double distance_factor, double heading_factor);

    /**
     * Sets the distance factor
     *
     * @param distance_factor the value to set the distance factor to
     */
    void setDistanceFactor(double distance_factor);

    /**
     * Sets the heading factor
     *
     * @param heading_factor the value to set the distance factor to
     */
    void setHeadingFactor(double heading_factor);

    /**
     * Creates a recommended twist command, based on the robots current heading, location, and dest. waypoint
     *
     * @param current_location the current location of the robot
     * @param current_heading the current heading of the robot
     * @param waypoint the waypoint we're trying to get to
     *
     * @return a twist message that moves the robot towards the waypoint
     */
    geometry_msgs::Twist createTwistMessage(geometry_msgs::Point current_location,
                                            double current_heading,
                                            geometry_msgs::Point waypoint);
    /**
     * Computes the average of 1/x and sqrt(y), multiplied by their respective scaling factors
     *
     * @param x TODO
     * @param y TODO
     * @param x_scale the value to multiply the x value by
     * @param y_scale the value to multiply the y value by
     *
     * @return the average of x_scale * 1/x and y_scale * sqrt(y)
     */
    double magicFunction(double x, double y, double x_scale, double y_scale);

    /**
     * Finds the minimum angular change required to turn from one heading to another
     *
     * @param from_heading the heading we're going from (in radians)
     * @param to_heading the heading we're going to (in radians)
     *
     * @return the minimum angular change FROM from_heading TO to_heading
     */
    double minAngularChange(double from_heading, double to_heading);

private:
    /**
     * Find the angle between two points, based on the ROS coordinate system (see main README)
     *
     * @param p1
     * @param p2
     *
     * @return the angle FROM startPoint TO endPoint
     */
    double angleBetweenPoints(geometry_msgs::Point startPoint, geometry_msgs::Point endPoint);

    double distance_factor, heading_factor;
};


class GpsDecision {
public:
    GpsDecision(int argc, char **argv, std::string node_name);

private:
    //callback for the current location
    void currentLocationCallback(const geometry_msgs::Point::ConstPtr& current_location);
    //callback for the current heading
    void headingCallback(const sensor_msgs::Imu::ConstPtr& imu_msg);
    //callback for our destination waypoint
    void waypointCallback(const geometry_msgs::Point::ConstPtr& waypoint);

    ros::Subscriber heading_subscriber;             // Subscribes to the current robot heading
    ros::Subscriber current_location_subscriber;    // Subscribes to the current robot location
    ros::Subscriber waypoint_subscriber;            // Subscribes to the next waypoint the robot has to go to

    ros::Publisher twist_publisher;     // Publishes a twist message telling the robot how to move

    double current_heading;                 // The current robot heading (in radians)
    geometry_msgs::Point current_location; // The current location of the robot

    Mover mover;    // The class that generates our twist messages
};

#endif //DECISION_GPS_DECISION_H
