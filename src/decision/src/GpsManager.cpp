//
// Created by gareth on 20/10/16.
//

#include "GpsManager.h"

GpsManager::GpsManager(int argc, char **argv, std::string node_name){
    // Setup NodeHandles
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;
    ros::NodeHandle public_nh("~");

    // Setup Subscribers
    std::string raw_gps_topic_name = "/gps_driver/raw_gps";
    int refresh_rate = 10;
    raw_gps_subscriber = public_nh.subscribe(raw_gps_topic_name, refresh_rate,
                                             &GpsManager::rawGpsCallBack, this);

    // Setup Publishers
    std::string compass_heading_topic = public_nh.resolveName("current_heading");
    uint32_t queue_size = 10;
    compass_heading_publisher = nh.advertise<std_msgs::Float32_>(compass_heading_topic, queue_size);
}

void GpsManager::rawGpsCallBack(const sensor_msgs::NavSatFix::ConstPtr nav_sat_fix) {
    this->most_recent_nav_sat_fix = *nav_sat_fix;
}