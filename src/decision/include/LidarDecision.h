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
 * The function the opening on left and right according to distance_used
 * (distance_used is the distance, for any distance smaller than that will be considered as an obstacle)
 * It will start from the most left (angle_min in LaserScan message) untill it finds an obstacle (the
 * distance value from the LaserScan message is smaller than the distance_used), and records how many
 * angle of opening is on the left. It will do same for the right. Then compare the two and decide
 * whether to turn left or right, if more space is at left than sign = 1
 * (positive angular speed for z dircetion= turn left), else if more space is at right than sign = -1
 *
 *
 * @param distance_used in meters
 * @param sensor_msgs is the full set of messages sent in from the lidar scan
 *          angle_min (the starting angle of the full scan messages),
 *          angle_increment (the angle differece between two consecutive data),
 *          and ranges (vector data storing the distance measured for every angle)
 *          are used.
 *
 * @return (+1) if there is more space on left
 *          (-1) if there is more space on right
 *
 */
    static int angular_speed_sign(const sensor_msgs::LaserScan::ConstPtr& raw_scan, float distance_used);

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
    void publishTwist(geometry_msgs::Twist twist);

    ros::Subscriber scan_subscriber;
    ros::Publisher twist_publisher;
};
#endif //DECISION_LIDAR_DECISION_H

