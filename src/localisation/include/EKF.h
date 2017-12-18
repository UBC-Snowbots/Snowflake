/*
 * Created By: Marcus Swift
 * Created On: November 20th, 2017
 * Description: An extended kalman filter node that takes in sensor data from
 * the GPS, encoders and IMU and returns the bots estimated postion and orientation	
 */

#ifndef EKF_EKFNODE_H
#define EKF_EKFNODE_H

#include "geometry_msgs/Pose.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Imu.h"
#include <iostream>
#include <std_msgs/String.h>
#include <ros/ros.h>
#include <sb_utils.h>
#include <cmath>
#include <Eigen/QR>
#include <Eigen/LU>
#include <Eigen/Geometry>

using Eigen::Matrix2d;
using Eigen::Matrix3d;
using Eigen::MatrixXd;
using Eigen::Vector3d;

class EKF {
private:
	//subscriber and publisher data types
	geometry_msgs::Pose bot_position;
	sensor_msgs::Imu imu_data;
	nav_msgs::Odometry gps_data;
	nav_msgs::Odometry encoder_data;
	//subscribers and publisher
    ros::Subscriber gps_sub;
	ros::Subscriber encoder_sub;
	ros::Subscriber imu_sub;
    ros::Publisher pose_pub;
	//how much time has past since the ekf started
	double time;
	//standard deviation of the bot's sensor's measurements
	const double sdev_gyro = 2.*M_PI/180.;        
	const double sdev_speed = 0.15;   
	const double sdev_gps_x_measurement = 5;
	const double sdev_gps_y_measurement = 5; 
	const double sdev_angle_measurement = 10.*M_PI/180.;
	//initial bot position (current values are for rosbag files)
	const double intial_pos_x = 481917;       //rostest initial values 482003
	const double intial_pos_y = 5456662;      //5456550
	const double intial_pos_z = 0;	  		  //0
	const double intial_ori_x = 0;    		  //0
	const double intial_ori_y = 0;    		  //0
	const double intial_ori_z = sin(M_PI/.4); //0
	const double intial_ori_w = cos(M_PI/.4); //1
	//matrices
	Matrix3d p; //covariance matrix
	Matrix3d j; //jacobian matrix (differentiate angle)
	Matrix3d q; //used to calculate the covariance matrix (process model)
	Matrix3d s; //used to calculate the Kalman gain
	Matrix3d k; //Kalman gain
	Vector3d position; //vector of bot position
	Vector3d z; //vector of gps and imu position/orientation measurements
	MatrixXd ju{MatrixXd(3,2)}; //jacobian matrix (differentiate speed)
	//const matrices
	//accounts for unknown external influences
	Matrix3d q1{(Matrix3d() << 0.01*0.01, 0,         0, 
							   0,         0.01*0.01, 0, 
							   0,         0,         (1.*(M_PI/180.))*(1.*(M_PI/180.))).finished()};
	//covariance of the encoder speed measurement and the imu angular velocity measurement
	Matrix2d pu{(Matrix2d() << sdev_speed*sdev_speed, 0,
							   0,                       sdev_gyro*sdev_gyro).finished()};
	//find out!!!
	Matrix3d h{(Matrix3d() << 1, 0, 0,
							  0, 1, 0, 
							  0, 0, 1).finished()};
	//covariance of the gps position measurements and the imu orientation measurement
	Matrix3d r{(Matrix3d() << sdev_gps_x_measurement*sdev_gps_x_measurement, 0, 0, 
							  0, sdev_gps_y_measurement*sdev_gps_y_measurement, 0, 
							  0, 0, sdev_angle_measurement*sdev_angle_measurement).finished()};
	//covariance for initial position currently assume it's perfect
	Matrix3d intial_p{(Matrix3d() << 0, 0, 0, 
									 0, 0, 0, 
									 0, 0, 0).finished()};

    /**
     * Callback function for when new GPS data is received
     *
     * @param The GPS data received in the callback in the form of an odometry msg
     */
    void gpsCallBack(const nav_msgs::Odometry::ConstPtr &gps_message);
	
	/**
     * Callback function for when new encoder data is received
     *
     * @param The encoder data received in the callback in the form of an odometry msg
     */	
	void encoderCallBack(const nav_msgs::Odometry::ConstPtr &encoder_message);
	
	/**
     * Callback function for when new IMU data is received
     *
     * @param The IMU data received in the callback in the form of an IMU msg
     */
	void imuCallBack(const sensor_msgs::Imu::ConstPtr &imu_message);
	 
    /**
     * Publishes the estimated postion calculated from the EKF
     *
     * @param Takes in a geometry_msgs/Pose message
     */
    void publishPose(geometry_msgs::Pose pose_msg);

public:
	//Constructor
    EKF(int argc, char **argv, std::string node_name);
	
	/**
     * Takes an angle and adjusts it if necessary so that it says within the
	 * bounds of -pi and +pi currently an angle that equals to pi, -pi, 3pi,
	 * -3pi, etc will be rebound to pi
     *
     * @param an angle in radians
     *
     * @return a rebounded angle in radians
     */
	static double defineAngleInBounds(double angle);
	
	/**
     * Takes in an angle of orientation on the z axis and converts it to
     * quaternion form
     *
     * @param angle of orientation on the z axis in radians
     *
     * @return angle in quaternion form
     */
	static geometry_msgs::Quaternion angleToQuaternion(double angle);
	
	/**
     * Takes in angle of orientation in quaternion form and converts it to
     * an angle of orientation on the z axis
     *
     * @param angle in quaternion form
     *
     * @return angle of orientation on the z axis in radians
     */
	static double quaternionToAngle(geometry_msgs::Quaternion quat_angle);
	
	
};

#endif //EKF_EKFNODE_H
