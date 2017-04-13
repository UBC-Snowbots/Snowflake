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
#include <sb_utils.h>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_datatypes.h>
#include <std_msgs/Float32.h>


// TODO: Should this be it's own header file? If so, we should
// probably do the same with the LidarObstacle class
class Mover {
public:
    /**
     * A constructor for a mover object
     *
     * See Mover::setFactors for param descriptions
     *
     * @param linear_distance_factor
     * @param linear_heading_factor
     * @param angular_distance_factor
     * @param angular_heading_factor
     */
    Mover(double linear_distance_factor, double linear_heading_factor,
          double angular_distance_factor, double angular_heading_factor);

    /**
     * An empty constructor will default to factors of 1
     */
    Mover() : Mover(1,1,1,1) {};

    /**
     * Sets relations between distance/angle to object, and linear/angular speeds
     *
     * @param linear_distance_factor how much distance from the obstacle influence
     * linear speed
     * @param linear_heading_factor how much the angle between the current heading and
     * the angle to the obstacle influence the linear speed
     * linear speed
     * @param angular_distance_factor how much distance from the obstacle influence
     * angular speed
     * @param angular_heading_factor how much the angle between the current heading and
     * the angle to the obstacle influence the angular speed
     * angular speed
     */
    void setFactors(double linear_distance_factor, double linear_heading_factor,
                double angular_distance_factor, double angular_heading_factor);

    /**
     * Sets the caps for linear and angular speed
     *
     * @param max_linear_speed the max linear speed for any commaand this mover returns
     * @param max_angular_speed the max angular speed for any command this mover returns
     */
    void setMaxSpeeds(double max_linear_speed, double max_angular_speed);

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

    /**
     * Find the angle between two points, based on the ROS coordinate system (see main README)
     *
     * @param startPoint
     * @param endPoint
     *
     * @return the angle FROM startPoint TO endPoint
     */
    double angleBetweenPoints(geometry_msgs::Point startPoint, geometry_msgs::Point endPoint);

private:

    /**
     * Reduces a value to an specified absolute maximum
     *
     * @param val the value to cap
     * @param cap the absolute cap
     */
     void capValue(double& val, double cap);

    // How much effect the distance to the waypoint has on the linear speed
    double linear_distance_factor;
    // How much effect the angle between our heading and the
    // angle to the waypoint has on the linear speed
    double linear_heading_factor;
    // How much effect the distance to the waypoint has on the angular speed
    double angular_distance_factor;
    // How much effect the heading between our heading and the
    // heading to the waypoint has on the angular speed
    double angular_heading_factor;

    // The max linear speed allowed
    double max_linear_speed;
    // The max angular speed allowed
    double max_angular_speed;
};


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

    Mover mover;    // The class that generates our twist messages
};

#endif //DECISION_GPS_DECISION_H
