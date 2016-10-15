/*
 * Created By: Gareth Ellis
 * Created On: September 22, 2016
 * Description: The vision decision node, takes in an image from the robot's
 *              camera and produces a recommended twist message
 */

#ifndef DECISION_VISION_DECISION_H
#define DECISION_VISION_DECISION_H

#include <iostream>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Image.h>
#include <ros/ros.h>

class VisionDecision {
public:
    VisionDecision(int argc, char **argv, std::string node_name);
    static double getImageRatio(const sensor_msgs::Image::ConstPtr& raw_scan);
private:
    static double mapRange(double x, double inMin, double inMax, double outMin, double outMax);
    static double getDesiredAngle(double imageRatio, double inLowerBound, double inUpperBound);
    void imageCallBack(const sensor_msgs::Image::ConstPtr& raw_image);
    void publishTwist(geometry_msgs::Twist twist);
   
    
    ros::Subscriber image_subscriber;
    ros::Publisher twist_publisher;
};
#endif //DECISION_VISION_DECISION_H
