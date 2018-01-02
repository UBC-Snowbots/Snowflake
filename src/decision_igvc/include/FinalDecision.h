/*
 * Created By: Kevin Luo
 * Modified by: Valerian Ratu
 * Created On: September 22, 2016
 * Description: The Final Decision node, arbitrates between the decisions of
 * other nodes.
 */

#ifndef DECISION_FINAL_DECISION_H
#define DECISION_FINAL_DECISION_H

#include <geometry_msgs/Twist.h>
#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/String.h>

class FinalDecision {
  public:
    // The constructor
    FinalDecision(int argc, char** argv, std::string node_name);

    /**
     * Arbitrates the three messages received from Lidar, Vision, and GPS
     *
     * If there is a change in one or more of the messages, the function decides
     * which one has the highest priority and it is published.
     * The messages are held in the following order of significance:
     *      -Lidar
     *      -Vision
     *      -GPS
     *
     * @param recent_lidar  Twist message from Lidar
     * @param recent_vision Twist message from the camera, telling the robot
     * where lines and boundaries are
     * @param recent_gps    Twist message from the GPS
     *
     * @return              The Twist message with the highest priority.
     */
    static geometry_msgs::Twist arbitrator(geometry_msgs::Twist recent_lidar,
                                           geometry_msgs::Twist recent_vision,
                                           geometry_msgs::Twist recent_gps);

  private:
    // Sensor callbacks
    void gpsCallBack(const geometry_msgs::Twist::ConstPtr& gps_decision);
    void lidarCallBack(const geometry_msgs::Twist::ConstPtr& lidar_decision);
    void visionCallBack(const geometry_msgs::Twist::ConstPtr& vision_decision);

    // Publishes twist
    void publishTwist(geometry_msgs::Twist twist);

    // Subscribers
    ros::Subscriber lidar_subscriber;
    ros::Subscriber vision_subscriber;
    ros::Subscriber gps_subscriber;

    // Publishers
    ros::Publisher twist_publisher;

    // Sensor data placeholders
    geometry_msgs::Twist recent_lidar;
    geometry_msgs::Twist recent_vision;
    geometry_msgs::Twist recent_gps;
};

#endif // DECISION_FINAL_DECISION_H
