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

class LidarDecision {
public:
    LidarDecision(int argc, char **argv, std::string node_name);
    static bool obstacle_in_range(double min_angle, double max_angle, float obstacle_distance,
                                  const sensor_msgs::LaserScan::ConstPtr& raw_scan);

    static geometry_msgs::Twist manage_twist(float obstacle_distance, float forward_speed,  float angular_speed,
                                                     const sensor_msgs::LaserScan::ConstPtr& raw_scan);
private:
    void scanCallBack(const sensor_msgs::LaserScan::ConstPtr& raw_scan);
    void publishTwist(geometry_msgs::Twist twist);

    ros::Subscriber scan_subscriber;
    ros::Publisher twist_publisher;
};
#endif //DECISION_LIDAR_DECISION_H

