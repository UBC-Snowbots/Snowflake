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
    static double mapRange(double x, double inMin, double inMax, double outMin, double outMax);
    static int getDesiredAngle(double numSamples, const sensor_msgs::Image::ConstPtr &image_scan);
    static int getAngleAt(bool rightSide, double numSamples, const sensor_msgs::Image::ConstPtr &image_scan);
    static double getDesiredAngularSpeed(double desiredAngle);
    static double getDesiredSpeed(double desiredAngle);
    static int getMiddle(int row, bool rightSide, const sensor_msgs::Image::ConstPtr& image_scan);
private:
    static int getEndPixel(int startingPos, int noiseMax, int incrementer, int row,
                           const sensor_msgs::Image::ConstPtr& image_scan);
    static int getStartPixel(int startingPos, int noiseMax, int incrementer, int row,
                             const sensor_msgs::Image::ConstPtr& image_scan);
    void imageCallBack(const sensor_msgs::Image::ConstPtr& image_scan);
    void publishTwist(geometry_msgs::Twist twist);
   
    
    ros::Subscriber image_subscriber;
    ros::Publisher twist_publisher;
};
#endif //DECISION_VISION_DECISION_H
