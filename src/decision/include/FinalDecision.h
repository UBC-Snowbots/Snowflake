/*
 * Created By: YOUR NAME HERE
 * Created On: September 22, 2016
 * Description: The Decision Node for FINAL, takes in a point relative to
 *              the robots location and heading and broadcasts a
 *              recommended twist message
 */

#ifndef DECISION_FINAL_DECISION_H
#define DECISION_FINAL_DECISION_H

#include <iostream>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>

class FinalDecision {
public:
    FinalDecision(int argc, char **argv, std::string node_name);
    static geometry_msgs::Twist arbitrator(geometry_msgs::Twist recent_lidar, geometry_msgs::Twist recent_vision, geometry_msgs::Twist recent_gps);

private:
    void gpsCallBack(const geometry_msgs::Twist::ConstPtr& gps_decision);
    void lidarCallBack(const geometry_msgs::Twist::ConstPtr& lidar_decision);
    void visionCallBack(const geometry_msgs::Twist::ConstPtr& vision_decision);
    void publishTwist(geometry_msgs::Twist twist);
    bool turning(geometry_msgs::Twist twist);

    ros::Subscriber lidar_subscriber;
    ros::Subscriber vision_subscriber;
    ros::Subscriber gps_subscriber;
    ros::Publisher twist_publisher;
    geometry_msgs::Twist recent_lidar;
    geometry_msgs::Twist recent_vision;
    geometry_msgs::Twist recent_gps;


};

#endif //DECISION_FINAL_DECISION_H
