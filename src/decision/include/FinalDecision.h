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
    // The constructor
    FinalDecision(int argc, char **argv, std::string node_name);

    /**
     * Arbitrates the three messages received from Lidar, Vision, and GPS
     *
     * If there is a change in one or more of the messages, the function decides which one has the highest priority and it is published.
     * The messages are held in the following order of significance:
     *      -Lidar
     *      -Vision
     *      -GPS
     *
     * @param recent_lidar  Twist message from Lidar
     * @param recent_vision Twist message from the camera, telling the robot where lines and boundaries are
     * @param recent_gps    Twist message from the GPS
     *
     * @return              The Twist message with the highest priority.
     */
    geometry_msgs::Twist arbitrator(geometry_msgs::Twist recent_lidar, geometry_msgs::Twist recent_vision, geometry_msgs::Twist recent_gps);

private:
    // This is called whenever a new message is received
    void gpsCallBack(const geometry_msgs::Twist::ConstPtr& gps_decision);
    // This is called whenever a new message is received
    void lidarCallBack(const geometry_msgs::Twist::ConstPtr& lidar_decision);
    // This is called whenever a new message is received
    void visionCallBack(const geometry_msgs::Twist::ConstPtr& vision_decision);
    void publishTwist(geometry_msgs::Twist twist);
    // This function evaluates whether GPS, Vision, or Lidar is sending a turning command
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
