/*
 * Created By: Gareth Ellis
 * Created On: September 22, 2016
 * Description: The LIDAR decision node, takes in a raw LIDAR scan
 *              and broadcasts a recommended Twist message
 */



#ifndef DECISION_LIDAR_DECISION_H
#define DECISION_LIDAR_DECISION_H

#include <iostream>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <ros/ros.h>
using namespace std;

class LidarDecision {
public:
    LidarDecision(int argc, char **argv, std::string node_name);

/**
 * Determine whether there is obstacle in a certain range
 *
 * The range parameter include starting angle to ending angle (min_angle, max_angle)
 * so the detection area will be from (min_angle to max angle and from 0 to max_obstacle_distance)
 * The sensor messages gives a distance value for each angle,
 * if any of the distance value for the angles from min_angle to max_angle is smaller than max_obstacle_distance,
 * then the function decides that there is an obstacle.
 * The angles (including those from sensor messages) are in radians
 * the distances are in meters
 *
 * @param min_angle The minimum angle in the LIDAR scan to check for obstacles
 * @param max_angle The maximum angle in the LIDAR scan to check for obstacles
 * @param max_obstacle_distance The maximum distance an object in the scan can be at and still considered an obstacle
 * @param raw_scan A reference to a sensor_msgs/LaserScan message
 *                  (http://docs.ros.org/api/sensor_msgs/html/msg/LaserScan.html)
 *
 * @return true if there is an obstacle
 *         false if there isn't an obstacle
 *
 */
    static bool obstacle_in_range(double min_angle, double max_angle, float max_obstacle_distance,
                                  const sensor_msgs::LaserScan::ConstPtr& raw_scan);

/**
 * Determines linear speed
 *
 * Both in_near and in_mid will be true if the obstacle is in near range.
 * Only in_mid will be true if obstacle is in medium range.
 * Both in_near and in_mid will be false if the obstacle is in far range
 *
 * @return 0 if obstacle is in near range
 *         10 if obstacle is in medium range
 *         20 if obstacle is in far range
 */
    static int linear_speed(bool in_near, bool in_mid);

/**
 * Determines Angular Speed
 *
 * Both in_near and in_mid will be true if the obstacle is in near range.
 * in_mid will be true if obstacle is in medium range.
 * in_far will be true if the obstacle is in far range.
 * All three will be false if there is no obstacle.
 *
 * @return 20 if obstacle is in near range
 *         10 if obstacle is in medium range
 *         5  if obstacle is in far range
 *         0  if there is no obstacle
 */

    static int angular_speed(bool in_near, bool in_mid, bool in_far);

/**
 * modify the twist message to be sent out
 *
 * Consider three distance to correspond to obstacle in near, medium, and far distances.
 * Only modifying linear speed in x direction and angular speed in z direction, other directions remain 0
 * if no obstacle are found, remain straight.
 * if there is obstacle, turn left or right using function angular_speed_sign to determine.
 * the speed values need to be adjusted by further experiments
 *
 * @param raw_scan A reference to a sensor_msgs/LaserScan message
 *                  (http://docs.ros.org/api/sensor_msgs/html/msg/LaserScan.html)
 *
 * @return vel_msg A geometry_msgs/Twist message (http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html)
 *         vel_msg.linear.x = linear_speed (some constant)
 *         vel_msg.angular.z = angular_speed_sign*angular_speed = (the sign determined from the function) * (some constant)
 *
 */
    static geometry_msgs::Twist manage_twist(const sensor_msgs::LaserScan::ConstPtr& raw_scan);


private:
    void scanCallBack(const sensor_msgs::LaserScan::ConstPtr& raw_scan);

    ros::Subscriber scan_subscriber;
    ros::Publisher twist_publisher;
};
#endif //DECISION_LIDAR_DECISION_H

