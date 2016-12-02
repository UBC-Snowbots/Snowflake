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
#include <tiff.h>

const int NOISEMAX = 30;

class VisionDecision {
public:
    VisionDecision(int argc, char **argv, std::string node_name);

    /**
     * Returns the angle of a valid line
     *
     * @param numSamples the number of slopes to sample the angle
     * @param image_scan the image to parse
     *
     * @return the angle of the line to the positive y-axis.
     */
    static int getDesiredAngle(double numSamples, const sensor_msgs::Image::ConstPtr &image);

    /**
     * Determines the angle of the line parsed from the left or right side.
     *
     * @param rightSide determines whether to parse from the left or from the right side.
     * @param numSamples how many slopes to sample the angle.
     * @param image_scan the image to parse.
     *
     * @returns the angle of the line, or INVALID if line is invalid.
     */
    static int getAngleOfLine(bool rightSide, double numSamples, const sensor_msgs::Image::ConstPtr &image);

    /**
     *  Returns a rotation speed based on the imageRatio
     *
     *  @param imageRatio
     *      rightBlackPixels - leftBlackPixels
     *  @returns double
     *      rotation speed of robot
     */
    static double getDesiredAngularSpeed(double desiredAngle);

    /**
     *  Returns the desired forward speed
     *
     *  @param desiredAngle
     *      angle robot will turn
     *  @returns double
     *      moving speed percentage of robot
     */
    static double getDesiredLinearSpeed(double desiredAngle);

private:
    /**
     * Determines the middle of the white line given a row
     *
     * @param startingPos which column to start parsing in
     * @param row the row to parse
     * @param rightSide determines whether to parse from the right or the left
     * @param image_scan the image to parse
     */
    static int getMiddle(int startingPos, int row, bool rightSide, const sensor_msgs::Image::ConstPtr& image);

    /**
     * Returns the white pixel right before the black space in between
     * the lines
     *
     * @param startingPos column to start parsing
     * @param NOISEMAX how large noise can be
     * @param incrementer decides whether to parse from the left or from the right
     * @param row determines the row to parse
     * @param image_scan the image to parse
     *
     * @returns the white pixel's column position, -1 if none found
     */
    static int getEndPixel(int startingPos, int incrementer, int row,
                           const sensor_msgs::Image::ConstPtr& image);
    /**
     * Returns the white pixel right after the black space in between
     * the line and the left side or right side of the screen.
     *
     * @param startingPos column to start parsing
     * @param NOISEMAX how large noise can be
     * @param incrementer decides whether to parse from the left or from the right
     * @param row determines the row to parse
     * @param image_scan the image to parse
     *
     * @returns the white pixel's column position, -1 if none found
     */
    static int getStartPixel(int startingPos, int incrementer, int row,
                             const sensor_msgs::Image::ConstPtr& image);

    /**
     * Re-maps a number from one range to another
     *
     * @param x
     *      the number to map
     * @param inMin
     *      lower bound of value's current range
     * @param inMax
     *      upper bound of value's current range
     * @param outMin
     *      lower bound of value's target range
     * @param outMax
     *      upper bound of value's target range
     * @return double
     *      the mapped number
     */
    static double mapRange(double x, double inMin, double inMax, double outMin, double outMax);

    void imageCallBack(const sensor_msgs::Image::ConstPtr& image);
    void publishTwist(geometry_msgs::Twist twist);
   
    
    ros::Subscriber image_subscriber;
    ros::Publisher twist_publisher;
};
#endif //DECISION_VISION_DECISION_H
