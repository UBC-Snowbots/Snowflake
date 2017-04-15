/*
 * Created By: Gareth Ellis
 * Created On: April 1, 2017
 * Description: The declaration for the GpsMover class. This class
 *              contains all the methods required to decide on the
 *              appropriate twist message to make the robot move smoothly
 *              towards a given point from it's current location
 */

#ifndef MOVER_H
#define MOVER_H

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>

class GpsMover {
public:
    /**
     * A constructor for a GpsMover object
     *
     * See GpsMover::setFactors for param descriptions
     *
     * @param linear_distance_factor
     * @param linear_heading_factor
     * @param angular_distance_factor
     * @param angular_heading_factor
     */
    GpsMover(double linear_distance_factor, double linear_heading_factor,
          double angular_distance_factor, double angular_heading_factor);

    /**
     * An empty constructor will default to factors of 1
     */
    GpsMover() : GpsMover(1,1,1,1) {};

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
     * @param max_linear_speed the max linear speed for any command this mover returns
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
     * @param x
     * @param y
     * @param x_scale the value to multiply the x value by
     * @param y_scale the value to multiply the y value by
     *
     * @return the average of: [x_scale * 1/x] and [y_scale * sqrt(y)]
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

#endif //MOVER_H
