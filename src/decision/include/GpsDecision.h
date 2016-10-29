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
#include <std_msgs/Float32.h>

class GpsDecision {
public:
    GpsDecision(int argc, char **argv, std::string node_name);
    static double distance(const geometry_msgs::Point::ConstPtr& relative_gps);
    static double desiredAngle(const geometry_msgs::Point relative_gps,float current_heading,geometry_msgs::Point currentPoint);
    void  rotate(double desiredAngle,double angular_velocity);
    static double angle(double x,double y);

private:
    void gpsCurrentCallBack(const geometry_msgs::Point::ConstPtr& relative_gps);
    void gpsCallBack(const geometry_msgs::Point::ConstPtr& relative_gps);
    void compassCallBack(const std_msgs::Float32::ConstPtr& compass_heading);
    void publishTwist(geometry_msgs::Twist twist);

    ros::Subscriber compass_subcriber;
    ros::Subscriber gps_subscriber;
    ros::Publisher twist_publisher;
    float current_heading;
    geometry_msgs::Point currentPoint;

};
#endif //DECISION_GPS_DECISION_H
