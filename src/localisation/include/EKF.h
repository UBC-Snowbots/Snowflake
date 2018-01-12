/*
 * Created By: Marcus Swift
 * Created On: December 23th, 2017
 * Description: An extended kalman filter class that takes in sensor data from
 * the GPS, encoders and IMU and returns the bots estimated postion and
 * orientation
 */

#ifndef LOCALISATION_EKF_H
#define LOCALISATION_EKF_H

#include <Eigen/Geometry>
#include <Eigen/LU>
#include <Eigen/QR>
#include <cmath>
#include <geometry_msgs/Pose.h>
#include <iostream>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sb_utils.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/String.h>
#include <tf/tf.h>

using Eigen::Matrix2d;
using Eigen::Matrix3d;
using Eigen::MatrixXd;
using Eigen::Vector3d;

class EKF {
  private:
    // current position of the robot
    geometry_msgs::Pose bot_position;
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
    Matrix3d q1;
    // covariance of the encoder speed measurement and the imu angular velocity
    // measurement
    Matrix2d pu;
    // observation matrix
    Matrix3d h;
    // covariance of the gps position measurements and the imu orientation
    // measurement
    Matrix3d r;

  public:
    /**
     * Takes in gps and imu measurements to perform the measurement model of the
     * kalman filter.
     *
     * @param ros specific data types for gps and imu measurement data
     *
     * @return ros specific data type of the bot's position
     */
    geometry_msgs::Pose updateState(nav_msgs::Odometry gps,
                                    sensor_msgs::Imu imu);

    /**
     * Takes in encoder and imu measurements to perform the process model of the
     * kalman filter.
     *
     * @param ros specific data types for encoder and imu measurement data
     */
    void
    predictState(nav_msgs::Odometry encoder, sensor_msgs::Imu imu, double dt);

    /**
     * Takes an angle and adjusts it if necessary so that it says within the
     * bounds of -pi and +pi
     *
     * @param an angle in radians
     *
     * @return a re-bounded angle in radians
     */
    static double constrainAngleInBounds(double angle);

    /**
     * Sets the values pf constant matrices p, q1, pu, h and r
     *
     * @param error values used to set the covariance matrices
     */
    void setConstants(double g_sdev,
                      double s_sdev,
                      double x_sdev,
                      double y_sdev,
                      double a_sdev,
                      std::vector<double> initial_p,
                      std::vector<double> q1_const,
                      std::vector<double> h_const);

    /**
     * Sets the initial bot position and orientation
     *
     * @param The x, y position values and yaw angle of the robot in radians
     */
    void setInitialPosition(double pos_x, double pos_y, double yaw_angle);
};

#endif // LOCALISATION_EKF_H
