/*
 * Created By: Marcus Swift
 * Created On: December 23th, 2017
 * Description: An extended kalman filter class that takes in sensor data from
 * the GPS, encoders and IMU and returns the bots estimated postion and
 * orientation
 */

#include <EKF.h>

EKF::EKF() {
	// set covariance for initial position
    p = intial_p;
}

EKF::EKF(double pos_x, double pos_y, double pos_z, double ori_x, double ori_y, 
 double ori_z, double ori_w) {
    // set covariance for initial position
    p = intial_p;

    // set initial position
    bot_position.position.x    = pos_x;
    bot_position.position.y    = pos_y;
    bot_position.position.z    = pos_z;
    bot_position.orientation.x = ori_x;
    bot_position.orientation.y = ori_y;
    bot_position.orientation.z = ori_z;
    bot_position.orientation.w = ori_w;

}

geometry_msgs::Pose EKF::measurementModel(nav_msgs::Odometry gps, sensor_msgs::Imu imu) {
    // put current bot position (from encoders) and gps and imu position into
    // vectors for calcuations
    position << bot_position.position.x, bot_position.position.y,
    defineAngleInBounds(tf::getYaw(bot_position.orientation));
    z << gps.pose.pose.position.x, gps.pose.pose.position.y,
    defineAngleInBounds(tf::getYaw(imu.orientation));

    s        = r + h * p * h.transpose();
    k        = p * h.transpose() * s.inverse();   // Kalman gain
    position = position + k * (z - h * position); // update the bot's position
    p =
    p - p * h.transpose() * s.inverse() * h * p; // update the Covariance matrix

    bot_position.position.x  = position(0);
    bot_position.position.y  = position(1);
    bot_position.orientation = 
	tf::createQuaternionMsgFromYaw(defineAngleInBounds(position(2)));

    return bot_position;
}

void EKF::processModel(nav_msgs::Odometry encoder, sensor_msgs::Imu imu, double dt) {
    double angle = tf::getYaw(bot_position.orientation); // bot's current angle
	angle = defineAngleInBounds(angle);
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
	tf::createQuaternionMsgFromYaw(defineAngleInBounds(angle));
}

double EKF::defineAngleInBounds(double angle) {
    double rebounded_angle;

    // Uses the modulus functions of 2PI and PI to rebound the angle
    if (fmod(angle, 2 * M_PI) == fmod(angle, M_PI)) {
        rebounded_angle = fmod(angle, M_PI);
    } else if (fmod(angle, M_PI) > 0) {
        rebounded_angle = fmod(angle, M_PI) - M_PI;
    } else {
        rebounded_angle = fmod(angle, M_PI) + M_PI;
    }

    return rebounded_angle;
}

