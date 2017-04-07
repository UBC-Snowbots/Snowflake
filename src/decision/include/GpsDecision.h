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

    /**
      * calculates the distance between next Point and current point
      * @param relative_gps the relative gps location of the next move
      * @return the distance between the next location and the current location
    */
    static double distance(const geometry_msgs::Point::ConstPtr& relative_gps);

    /**
     * @param relative_gps the relative gps location of the next move
     * @param current_heading the current heading relative to north in degrees (0 to 360 degrees)
     * @return the angle(less than PI) in degrees that robot need to turn
     * positive value means turning clockwise
     * negative value means turing counter-clockwise
     */
    static double desiredAngle(const geometry_msgs::Point relative_gps,float current_heading,geometry_msgs::Point currentPoint);
    /**
     * @param desiredAngle the angle(less than PI) in degrees that robot need to turn
     * positive value means turning clockwise
     * negative value means turing counter-clockwise
     * @param angular_velocity the angular_velocity(rad/s)
     * @return rotate the robot by giving angles and angular velocity
     */
    void  rotate(const geometry_msgs::Point::ConstPtr& relative_gps);

private:
    //gps callback for the current point
    void gpsCurrentCallBack(const geometry_msgs::Point::ConstPtr& relative_gps);
    //gps callback for the next point
    void gpsCallBack(const geometry_msgs::Point::ConstPtr& relative_gps);
    //gps callback for the current heading
    void compassCallBack(const std_msgs::Float32::ConstPtr& compass_heading);
    //publish twist mesg during gps callback ofr the next point
    void publishTwist(geometry_msgs::Twist twist);
    //get desired angular speed given the angle to turn
    double getDesiredAngularSpeed(double desiredAngle);
    //get desired linear speed given the distance to move
    double getDesiredLinearSpeed(double distance);
    //map function
    double mapRange(double x, double inMin, double inMax, double outMin, double outMax);
    ros::Subscriber compass_subscriber;
    ros::Subscriber gps_subscriber;
    ros::Publisher twist_publisher;
    float current_heading;
    geometry_msgs::Point currentPoint;

};
#endif //DECISION_GPS_DECISION_H
