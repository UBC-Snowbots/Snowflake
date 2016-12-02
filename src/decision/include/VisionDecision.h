/*
 * Created By: Robyn Castro
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
#include <math.h>

const int NOISEMAX = 30;

class VisionDecision {
public:
    VisionDecision(int argc, char **argv, std::string node_name);

    static double mapRange(double x, double inMin, double inMax, double outMin, double outMax);
    static int getDesiredAngle(double numSamples, const sensor_msgs::Image::ConstPtr &image);
    static int getAngleOfLine(bool rightSide, double numSamples, const sensor_msgs::Image::ConstPtr &image);
    static double getDesiredAngularSpeed(double desiredAngle);
    static double getDesiredLinearSpeed(double desiredAngle);
    static int getMiddle(int startingPos, int row, bool rightSide, const sensor_msgs::Image::ConstPtr& image);
private:
    static int getEndPixel(int startingPos, int incrementer, int row,
                           const sensor_msgs::Image::ConstPtr& image);
    static int getStartPixel(int startingPos, int incrementer, int row,
                             const sensor_msgs::Image::ConstPtr& image);
    void imageCallBack(const sensor_msgs::Image::ConstPtr& image);
    void publishTwist(geometry_msgs::Twist twist);
   
    
    ros::Subscriber image_subscriber;
    ros::Publisher twist_publisher;
};
#endif //DECISION_VISION_DECISION_H
