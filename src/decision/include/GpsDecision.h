/*
 * Created By: Gareth Ellis
 * Created On: July 16th, 2016
 * Description: An example node that subscribes to a topic publishing strings,
 *              and re-publishes everything it receives to another topic with
 *              a "!" at the end
 */

#ifndef DECISION_GPS_DECISION_H
#define DECISION_GPS_DECISION_H

#include <iostream>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <ros/ros.h>

class GpsDecision {
public:
    GpsDecision(int argc, char **argv, std::string node_name);
private:
    void gpsCallBack(const geometry_msgs::Point::ConstPtr& relative_gps);
    void publishTwist(geometry_msgs::Twist twist);

    ros::Subscriber scan_subscriber;
    ros::Publisher twist_publisher;
};
#endif //DECISION_GPS_DECISION_H
