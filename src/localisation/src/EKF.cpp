/*
 * Created By: Marcus Swift
 * Created On: December 23th, 2017
 * Description: An extended kalman filter class that takes in sensor data from
 * the GPS, encoders and IMU and returns the bots estimated postion and
 * orientation
 */

#include <EKF.h>

geometry_msgs::Pose EKF::updateState(nav_msgs::Odometry gps,
                                     sensor_msgs::Imu imu) {
    // put current bot position (from encoders) and gps and imu position into
    // vectors for calcuations
    position << bot_position.position.x, bot_position.position.y,
    constrainAngleInBounds(tf::getYaw(bot_position.orientation));
    z << gps.pose.pose.position.x, gps.pose.pose.position.y,
    constrainAngleInBounds(tf::getYaw(imu.orientation));

    s        = r + h * p * h.transpose();
    k        = p * h.transpose() * s.inverse();   // Kalman gain
    position = position + k * (z - h * position); // update the bot's position
    p =
    p - p * h.transpose() * s.inverse() * h * p; // update the Covariance matrix

    bot_position.position.x = position(0);
    bot_position.position.y = position(1);
    bot_position.orientation =
    tf::createQuaternionMsgFromYaw(constrainAngleInBounds(position(2)));

    return bot_position;
}

void EKF::predictState(nav_msgs::Odometry encoder,
                       sensor_msgs::Imu imu,
                       double dt) {
    double angle = tf::getYaw(bot_position.orientation); // bot's current angle
    angle        = constrainAngleInBounds(angle);
    double speed = encoder.twist.twist.linear.x; // bot's current speed

    // Jacobian Matrices of the process model:
    j << 1, 0, -dt * speed * sin(angle), 0, 1, dt * speed * cos(angle), 0, 0, 1;
    ju << dt * cos(angle), 0, dt * sin(angle), 0, 0, dt;
    q = ju * pu * ju.transpose() + q1;
    p = j * p * j.transpose() + q; // update the Covariance matrix

    // update the bot's position
    bot_position.position.x += dt * speed * cos(angle);
    bot_position.position.y += dt * speed * sin(angle);
    angle += dt * imu.angular_velocity.z;
    bot_position.orientation =
    tf::createQuaternionMsgFromYaw(constrainAngleInBounds(angle));
}

double EKF::constrainAngleInBounds(double angle) {
    double constrained_angle;

    // Uses the modulus functions of 2PI and PI to rebound the angle
    if (fmod(angle, 2 * M_PI) == fmod(angle, M_PI)) {
        constrained_angle = fmod(angle, M_PI);
    } else if (fmod(angle, M_PI) > 0) {
        constrained_angle = fmod(angle, M_PI) - M_PI;
    } else {
        constrained_angle = fmod(angle, M_PI) + M_PI;
    }

    return constrained_angle;
}

void EKF::setConstants(double sdev_gyro,
                       double sdev_speed,
                       double sdev_gps_x_measurement,
                       double sdev_gps_y_measurement,
                       double sdev_angle_measurement,
                       double uncertainty_x,
                       double uncertainty_y,
                       double uncertainty_ori,
                       double uncertainty_x_travel,
                       double uncertainty_y_travel,
                       double uncertainty_rot,
                       std::vector<double> h_const) {
    for (int i = 0; i < 9; i++) { h(i) = h_const[i]; }
    pu << sdev_speed * sdev_speed, 0, 0, sdev_gyro * sdev_gyro;
    r << sdev_gps_x_measurement * sdev_gps_x_measurement, 0, 0, 0,
    sdev_gps_y_measurement * sdev_gps_y_measurement, 0, 0, 0,
    sdev_angle_measurement * sdev_angle_measurement;
    p << uncertainty_x * uncertainty_x, 0, 0, 0, uncertainty_y * uncertainty_y,
    0, 0, 0, uncertainty_ori * uncertainty_ori;
    q1 << uncertainty_x_travel * uncertainty_x_travel, 0, 0, 0,
    uncertainty_y_travel * uncertainty_y_travel, 0, 0, 0,
    uncertainty_rot * uncertainty_rot;
}

void EKF::setInitialPosition(double pos_x, double pos_y, double yaw_angle) {
    bot_position.position.x = pos_x;
    bot_position.position.y = pos_y;
    bot_position.orientation =
    tf::createQuaternionMsgFromYaw(constrainAngleInBounds(yaw_angle));
}