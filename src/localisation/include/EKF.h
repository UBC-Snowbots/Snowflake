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
    // constant matrices
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
     * Takes in gps and imu measurements to update the current state
     * (position/orientation) of the bot which has been predicted for
     * the majority of the time with the predict state function.
     * This is to remove the drift associated with prediction.
     *
     * @param ros specific data type for gps data
     *
     * @param ros specific data type for imu measurement data
     *
     * @return ros specific data type of the bot's position
     */
    geometry_msgs::Pose updateState(nav_msgs::Odometry gps,
                                    sensor_msgs::Imu imu);

    /**
     * Takes in encoder and imu measurements to predict the current state
     * (position/orientation) of the bot using a basic a unicycle model
     * to model the motion of the bot.
     *
     * @param ros specific data type for encoder data
     *
     * @param ros specific data type imu measurement data
     *
     * @param change in time since last time the function was run
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
     * @param The error of the IMU angular velocity measurement
     *
     * @param The error of the linear (forward) speed encoder measurement
     *
     * @param The error of the gps x coordinate measurement
     *
     * @param The error of the gps y coordinate measurement
     *
     * @param The error of the IMU orientation measurement
     *
     * @param vector used to initialise the covariance matrix
     *
     * @param vector used to set the matrix used to account for unknown external
     * influences
     *
     * @param vector used to set the observation matrix
     */
    void setConstants(double g_sdev,
                      double s_sdev,
                      double x_sdev,
                      double y_sdev,
                      double a_sdev,
                      double uncertainty_x,
                      double uncertainty_y,
                      double uncertainty_ori,
                      double uncertainty_x_travel,
                      double uncertainty_y_travel,
                      double uncertainty_rot,
                      std::vector<double> h_const);

    /**
     * Sets the initial bot position and orientation
     *
     * @param The intial x position value
     *
     * @param The intial y position value
     *
     * @param The intial yaw angle of the robot in radians
     */
    void setInitialPosition(double pos_x, double pos_y, double yaw_angle);
};

#endif // LOCALISATION_EKF_H
