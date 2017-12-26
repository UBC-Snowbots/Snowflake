/*
 * Created By: Marcus Swift
 * Created On: December 23th, 2017
 * Description: An extended kalman filter class that takes in sensor data from
 * the GPS, encoders and IMU and returns the bots estimated postion and
 * orientation
 */

#ifndef EKF_EKFNODE_H
#define EKF_EKFNODE_H

#include "geometry_msgs/Pose.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Imu.h"
#include <Eigen/Geometry>
#include <Eigen/LU>
#include <Eigen/QR>
#include <cmath>
#include <iostream>
#include <ros/ros.h>
#include <tf/tf.h>
#include <sb_utils.h>
#include <std_msgs/String.h>

using Eigen::Matrix2d;
using Eigen::Matrix3d;
using Eigen::MatrixXd;
using Eigen::Vector3d;

class EKF {
  private:
    // standard deviation of the bot's sensor's measurements
    const double sdev_gyro              = 2. * M_PI / 180.;
    const double sdev_speed             = 0.15;
    const double sdev_gps_x_measurement = 5;
    const double sdev_gps_y_measurement = 5;
    const double sdev_angle_measurement = 10. * M_PI / 180.;
    // matrices
    Matrix3d p; // covariance matrix
    Matrix3d j; // jacobian matrix (differentiate angle)
    Matrix3d q; // used to calculate the covariance matrix (process model)
    Matrix3d s; // used to calculate the Kalman gain
    Matrix3d k; // Kalman gain
    Vector3d position; // vector of bot position
    Vector3d z; // vector of gps and imu position/orientation measurements
    MatrixXd ju{MatrixXd(3, 2)}; // jacobian matrix (differentiate speed)
    // const matrices
    // accounts for unknown external influences
    Matrix3d q1{(Matrix3d() << 0.01 * 0.01,
                 0,
                 0,
                 0,
                 0.01 * 0.01,
                 0,
                 0,
                 0,
                 (1. * (M_PI / 180.)) * (1. * (M_PI / 180.)))
                .finished()};
    // covariance of the encoder speed measurement and the imu angular velocity
    // measurement
    Matrix2d pu{
    (Matrix2d() << sdev_speed * sdev_speed, 0, 0, sdev_gyro* sdev_gyro)
    .finished()};
    // observation matrix
    Matrix3d h{(Matrix3d() << 1, 0, 0, 0, 1, 0, 0, 0, 1).finished()};
    // covariance of the gps position measurements and the imu orientation
    // measurement
    Matrix3d r{(Matrix3d() << sdev_gps_x_measurement * sdev_gps_x_measurement,
                0,
                0,
                0,
                sdev_gps_y_measurement* sdev_gps_y_measurement,
                0,
                0,
                0,
                sdev_angle_measurement* sdev_angle_measurement)
               .finished()};
    // covariance for initial position currently assume it's perfect
    Matrix3d intial_p{(Matrix3d() << 0, 0, 0, 0, 0, 0, 0, 0, 0).finished()};

public:
	// ros's specific position data type
    geometry_msgs::Pose bot_position;
	//Default Constructor
	EKF();
	
    // Constructor
    EKF(double pos_x, double pos_y, double pos_z, double ori_x, double ori_y, 
	 double ori_z, double ori_w);

    /**
     * Takes an angle and adjusts it if necessary so that it says within the
     * bounds of -pi and +pi
     *
     * @param an angle in radians
     *
     * @return a rebounded angle in radians
     */
    static double defineAngleInBounds(double angle);
	
	/**
     * Takes in encoder and imu measurements to perform the process model of the
     * kalman filter.
     *
     * @param ros specific data types for encoder and imu measurement data
     */
	void processModel(nav_msgs::Odometry encoder, sensor_msgs::Imu imu, double dt);
	
	/**
     * Takes in gps and imu measurements to perform the measurement model of the
     * kalman filter.
     *
     * @param ros specific data types for gps and imu measurement data
     *
     * @return ros specific data type of the bot's position
     */
	geometry_msgs::Pose measurementModel(nav_msgs::Odometry gps, sensor_msgs::Imu imu);

};

#endif // EKF_EKFNODE_H
