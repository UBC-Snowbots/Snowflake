/*
 * Created By: Marcus Swift
 * Created On: November 20th, 2017
 * Description: An extended kalman filter node that takes in sensor data from
 * the GPS, encoders and IMU and returns the bots estimated postion and
 * orientation usng the ekf class
 */

#ifndef EKFNODE_EKFNODE_H
#define EKFNODE_EKFNODE_H

#include "geometry_msgs/Pose.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Imu.h"
#include <EKF.h>
#include <cmath>
#include <iostream>
#include <ros/ros.h>
#include <sb_utils.h>
#include <std_msgs/String.h>

class EKFNode : public EKF {
  private:
    // subscriber and publisher data types
    sensor_msgs::Imu imu_data;
    nav_msgs::Odometry gps_data;
    nav_msgs::Odometry encoder_data;
    // subscribers and publisher
    ros::Subscriber gps_sub;
    ros::Subscriber encoder_sub;
    ros::Subscriber imu_sub;
    ros::Publisher pose_pub;
    // how much time has past since the ekf started
    double time;
    // initial bot position (current values are for rostest files)
    const double initial_pos_x = 481917;
    const double initial_pos_y = 5456662;
    const double initial_pos_z = 0;
    const double initial_yaw_angle = 0; // change to M_PI/2 for rosbag files

    /**
     * Callback function for when new GPS data is received
     *
     * @param The GPS data received in the callback in the form of an odometry
     * msg
     */
    void gpsCallBack(const nav_msgs::Odometry::ConstPtr& gps_message);

    /**
     * Callback function for when new encoder data is received
     *
     * @param The encoder data received in the callback in the form of an
     * odometry msg
     */
    void encoderCallBack(const nav_msgs::Odometry::ConstPtr& encoder_message);

    /**
     * Callback function for when new IMU data is received
     *
     * @param The IMU data received in the callback in the form of an IMU msg
     */
    void imuCallBack(const sensor_msgs::Imu::ConstPtr& imu_message);

    /**
     * Publishes the estimated postion calculated from the EKF
     *
     * @param Takes in a geometry_msgs/Pose message
     */
    void publishPose(geometry_msgs::Pose pose_msg);

  public:
    // Constructor
    EKFNode(int argc, char** argv, std::string node_name);
};

#endif // EKFNODE_EKFNODE_H