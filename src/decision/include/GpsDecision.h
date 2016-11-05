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

    /**caclutes the distance between wayPoint and current point
     * @ parameter: given the gps locaiton of the next move
     * @ return: the distance between the next location and the current location
    */
    static double distance(const geometry_msgs::Point::ConstPtr& relative_gps);

    /**
 * @ parameter
 *    relative_gps: given the gps locaiton of the next move
 *    current_heading: the curren heading relative to north in degrees (0 to 360 degrees)
 * @ return:
 *   desiredAngle: the angle(less than PI) in degrees that robot need to turn
 *                 positive value means turning clockwise
 *                 negative value means turing counter-clockwise
 */
    static double desiredAngle(const geometry_msgs::Point relative_gps,float current_heading,geometry_msgs::Point currentPoint);
    /**
     * @ parameter
     *  desiredAngle: the angle(less than PI) in degrees that robot need to turn
     *                 positive value means turning clockwise
     *                 negative value means turing counter-clockwise
     *  angular_velocity: the angular_velocity(rad/s)
     * @ effects: rotate the robot by giving angles and angular velocity
     */
    void  rotate(double desiredAngle,double angular_velocity);

private:
    //gps callback for the current point
    void gpsCurrentCallBack(const geometry_msgs::Point::ConstPtr& relative_gps);
    //gps callback for the next point
    void gpsCallBack(const geometry_msgs::Point::ConstPtr& relative_gps);
    //gps callback for the current heading
    void compassCallBack(const std_msgs::Float32::ConstPtr& compass_heading);
    void publishTwist(geometry_msgs::Twist twist);

    ros::Subscriber compass_subcriber;
    ros::Subscriber gps_subscriber;
    ros::Publisher twist_publisher;
    float current_heading;
    geometry_msgs::Point currentPoint;

};
#endif //DECISION_GPS_DECISION_H
