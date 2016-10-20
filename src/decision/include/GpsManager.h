//
// Created by gareth on 20/10/16.
//

#ifndef DECISION_GPSMANAGER_H
#define DECISION_GPSMANAGER_H

#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "sensor_msgs/NavSatFix.h"

class GpsManager {
public:
    GpsManager(int argc, char **argv, std::string node_name);
private:
    ros::Subscriber raw_gps_subscriber;
    ros::Publisher compass_heading_publisher;

    // The most recent navsatfix
    sensor_msgs::NavSatFix most_recent_nav_sat_fix;

    void rawGpsCallBack(const sensor_msgs::NavSatFix::ConstPtr nav_sat_fix);
};


#endif //DECISION_GPSMANAGER_H
