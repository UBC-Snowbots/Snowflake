/*
 * Created By: Sophie Chen
 * Created On: March 8, 2023
 * Description: A node that subscribes to gps messages,
 *              and publishes movement vector to MoveMotor 
 */

#ifndef PATHFINDER
#define PATHFINDER

// STD Includes
#include <iostream>

// ROS Includes
#include <std_msgs/String.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>

// Snowbots Includes
#include <sb_utils.h>

class PathFinder {
public:
    PathFinder(int argc, char **argv, std::string node_name);
    /**
     * Returns a vector that provides the shortest possible path between two given coordinates.
     *
     * Given two coordinates, each with an x and y coordinate, with one being the starting point and 
     * the other being the end point, we use vector subtraction to find the shortest path between the points.
     */

private:
    /**
     * Callback function for when a new string is received
     *
     * @param msg the string received in the callback
     */
    void CallBack(const sensor_msgs::NavSatFix::ConstPtr& msg);

    void update_movement_vector(float x_rover, float y_rover, float x_dest, float y_dest);
    //void publish_movement_vector();


    /**
     * Publishes a given string
     *
     * @param msg_to_publish the string to publish
     */

    /**
     * Helper function that sets float x and float y given two pairs of x and y values
     */
    ros::Subscriber my_subscriber;
    ros::Publisher my_publisher;
    float linear_x;
    float angular_y;
    float last_coord_x;
    float last_coord_y;
    float cardinal_direction;
    float angular_threshold;
};
#endif //PATHFINDING
