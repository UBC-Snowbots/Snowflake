/*
 * Created By: Robyn Castro
 * Created On: September 22, 2016
 * Description: The vision decision node, takes in an image from the robot's
 *              camera and produces a recommended twist message
 */

#ifndef DECISION_VISION_DECISION_H
#define DECISION_VISION_DECISION_H

// STD
#include <iostream>

// Snowbots
#include <sb_utils.h>

// ROS
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Image.h>
#include <ros/ros.h>
#include <math.h>
#include <tiff.h>

// This constant is used when verifying if a pixel is noise.
// It determines the maximum consecutive number of pixels that will still
// be considered noise. The lower the number, the smaller the expected noise
// size.
const int NOISE_MAX = 30;

// Since 90 can never be returned by arctan. 90 will be used as a special angle
// to signal desired angular speed and desired linear speed functions to return 0.
const int STOP_SIGNAL_ANGLE = 90;

class VisionDecision {
public:
    VisionDecision(int argc, char **argv, std::string node_name);

    /**
     * Determines the turning angle in relation to the orientation of
     * the white line in the image.
     *
     * @param numSamples determines the number of samples to average when determining
     *                   the angle.
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

    /*
     * Returns the edge pixel of the line. Which side depends on parameter
     * isStartPixel.
     *
     * @param startingPos column to start parsing
     * @param NOISE+MAX how large noise can be
     * @param incrementer decides whether to parse from the left or from the right
     * @param row determines the row to parse
     * @param image_scan the image to parse
     * @param isStartPixel determines which edge pixel of the white line to return.
     *
     * @returns the white pixel's column position, -1 if none found
     */
    static int getEdgePixel(int startingPos, int incrementer, int row,
                             const sensor_msgs::Image::ConstPtr& image, bool isStartPixel);

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

    /**
     * Initializes the incrementer's starting position and how it will parse.
     *
     * @param image_scan the image to parse.
     * @param rightSide determines whether to parse from the left or the right side of the image.
     * @param startingPos where the parser will start parsing the image.
     */
    static int initializeIncrementerPosition(bool rightSide, const sensor_msgs::Image::ConstPtr &image_scan, int *startingPos);

    /**
     * Checks whether the line is perpendicular to the robot's vision.
     *
     * @param image_scan the image to parse.
     */
    static bool isPerpendicular(const sensor_msgs::Image::ConstPtr &image_scan);

    /**
     * Gets the first valid white pixel bottom to top at the specified column.
     *
     * @param image_scan the image to parse.
     * @param column the column to parse at.
     *
     */
    static int getVerticalEdgePixel(const sensor_msgs::Image::ConstPtr &image_scan, int column);

    void imageCallBack(const sensor_msgs::Image::ConstPtr& image);
    void publishTwist(geometry_msgs::Twist twist);
   
    
    ros::Subscriber image_subscriber;
    ros::Publisher twist_publisher;

    // The value to scale the angular velocity on the
    // twist message we create by
    double angular_velocity_multiplier;
};
#endif //DECISION_VISION_DECISION_H
