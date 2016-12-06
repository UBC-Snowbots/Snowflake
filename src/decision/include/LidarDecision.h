/*
 * Created By: Gareth Ellis
 * Created On: September 22, 2016
 * Description: The Lidar decision node, takes in a raw Lidar scan
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
 * determine whether there is obstacle in a certain range
 *
 * The range parameter include starting angle to ending angle (min_angle, max_angle)
 * so the detection area will be from (min_angle to max angle and from 0 to obstacle_distance)
 * The sensor mesages gives a distance value for each angle,
 * if any of the distance value for the angles from min_angle to max_angle is smaller than obstacle_distance,
 * then the function decides that there is an obstacle
 * the angles (including those from sensor mesages) are in radius
 * the distances are in meters
 *
 * @param min_angle, max_angle, obstacle_distance, are all numbers
 * @param sensor_msgs is the full set of messages sent in from the lidar scan
 *          angle_min (the starting angle of the full scan messages),
 *          angle_increment (the angle differece between two consecutive data),
 *          and ranges (vector data storing the distance measured for every angle)
 *          are used.
 *
 * @return true if there is an obstacle
 *          false if there isn't an obstacle
 *
 */
    static bool obstacle_in_range(double min_angle, double max_angle, float obstacle_distance,
                                  const sensor_msgs::LaserScan::ConstPtr& raw_scan);

/**
 * determine whether to turn left (positive angular velocity in z direction) or right (negative)
 *
 * The function is a very simple function that translate the calculation done in manage_twist
 * using obstacle_in_range to ceate bool value on_right, which will be true if the obstacle is
 * at the right and false if not
 *
 * @return (+1) if the obstacle is on the right
 *          (-1) if the obstacle is on the left
 *
 */
    static int angular_speed_sign(bool on_right);
/**
 * determine the linear speed
 *
 * The function is a very simple function that translate the calculation done in manage_twist
 * using obstacle_in_range to ceate bool value in_near and in_mid. Both in_near and in_mid will be true
 * if the obstacle is in near range. in_mid will be true if obstacle is in midium range. Both in_near
 * and in_mid will be false if the obstacle is in far range
 *
 * @return 0 if obstacle is in near range
 *         10 if obstacle is in midium range
 *         20 if obstacle is in far range
 *         NOTE THAT 10 AND 20 ARE TEMPORARY VALUE
 */
    static int linear_speed(bool in_near, bool in_mid);
/**
 * determine the linear speed
 *
 * The function is a very simple function that translate the calculation done in manage_twist
 * using obstacle_in_range to ceate bool value in_near and in_mid. Both in_near and in_mid will be true
 * if the obstacle is in near range. in_mid will be true if obstacle is in midium range. in_far will be
 * true if the obstacle is in far range. All three will be false if there is no obstacle.
 *
 * @return 20 if obstacle is in near range
 *         10 if obstacle is in midium range
 *         5  if obstacle is in far range
 *         0  if there is no obstacle
 *         NOTE THAT 5, 10 AND 20 ARE TEMPORARY VALUE
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
 * @param sensor_msgs is the full set of messages sent in from the lidar scan
 *          angle_min (the starting angle of the full scan messages),
 *          angle_increment (the angle differece between two consecutive data),
 *          and ranges (vector data storing the distance measured for every angle)
 *          are used.
 *
 * @return vel_msg
 *          vel_msg.linear.x = linear_speed (some constant)
 *          vel_msg.angular.z = angular_speed_sign*angular_speed = (the sign determined from the function) * (some constant)
 *
 */
    static geometry_msgs::Twist manage_twist(const sensor_msgs::LaserScan::ConstPtr& raw_scan);


private:
    void scanCallBack(const sensor_msgs::LaserScan::ConstPtr& raw_scan);

    ros::Subscriber scan_subscriber;
    ros::Publisher twist_publisher;
};
#endif //DECISION_LIDAR_DECISION_H

