/*
 * Created By: Marcus Swift
 * Created On: November 20th, 2017
 * Description: An extended kalman filter node that takes in sensor data from
 * the GPS, encoders and IMU and returns the bots estimated postion and
 * orientation
 */

#include <EKF.h>

EKF::EKF(int argc, char** argv, std::string node_name) {
    // Setup NodeHandle
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;

    // Setup Subscribers
    gps_sub     = nh.subscribe("/gps_driver/odom", 10, &EKF::gpsCallBack, this);
    encoder_sub = nh.subscribe("/odom", 10, &EKF::encoderCallBack, this);
    imu_sub     = nh.subscribe("/imu", 10, &EKF::imuCallBack, this);

    // Setup Publisher
    pose_pub = nh.advertise<geometry_msgs::Pose>("/cmd_pose", 10);

    // set covariance for initial position
    p = intial_p;

    // set initial position
    bot_position.position.x    = intial_pos_x;
    bot_position.position.y    = intial_pos_y;
    bot_position.position.z    = intial_pos_z;
    bot_position.orientation.x = intial_ori_x;
    bot_position.orientation.y = intial_ori_y;
    bot_position.orientation.z = intial_ori_z;
    bot_position.orientation.w = intial_ori_w;

    // initial time
    time = ros::Time::now().toSec();
}

void EKF::gpsCallBack(const nav_msgs::Odometry::ConstPtr& gps_message) {
    // Measurement Model of the Kalman Filter

    // Gather GPS data currently, assume gps updates slower than encoder and imu
    gps_data.pose  = gps_message->pose;
    gps_data.twist = gps_message->twist;

    // put current bot position (from encoders) and gps and imu position into
    // vectors for calcuations
    position << bot_position.position.x, bot_position.position.y,
    quaternionToAngle(bot_position.orientation);
    z << gps_data.pose.pose.position.x, gps_data.pose.pose.position.y,
    quaternionToAngle(imu_data.orientation);

    s        = r + h * p * h.transpose();
    k        = p * h.transpose() * s.inverse();   // Kalman gain
    position = position + k * (z - h * position); // update the bot's position
    p =
    p - p * h.transpose() * s.inverse() * h * p; // update the Covariance matrix

    bot_position.position.x  = position(0);
    bot_position.position.y  = position(1);
    bot_position.orientation = angleToQuaternion(position(2));

    publishPose(bot_position);
}

void EKF::encoderCallBack(const nav_msgs::Odometry::ConstPtr& encoder_message) {
    // Process Model of the Kalman Filter

    // Gather encoder data, currently assume encoder updates slower than imu
    encoder_data.pose  = encoder_message->pose;
    encoder_data.twist = encoder_message->twist;

    double angle =
    quaternionToAngle(bot_position.orientation);      // bot's current angle
    double speed = encoder_data.twist.twist.linear.x; // bot's current speed
    double dt = 1;
    // double dt = ros::Time::now().toSec() - time; //set change in time to 1
    // for rostest
    time += dt;

    // Jacobian Matrices of the process model:
    j << 1, 0, -dt * speed * sin(angle), 0, 1, dt * speed * cos(angle), 0, 0, 1;
    ju << dt * cos(angle), 0, dt * sin(angle), 0, 0, dt;
    q = ju * pu * ju.transpose() + q1;
    p = j * p * j.transpose() + q; // update the Covariance matrix

    // update the bot's position
    bot_position.position.x += dt * speed * cos(angle);
    bot_position.position.y += dt * speed * sin(angle);
    angle += dt * imu_data.angular_velocity.z;
    defineAngleInBounds(angle);
    bot_position.orientation = angleToQuaternion(angle);
}

void EKF::imuCallBack(const sensor_msgs::Imu::ConstPtr& imu_message) {
    // Gather IMU data
    imu_data.orientation         = imu_message->orientation;
    imu_data.angular_velocity    = imu_message->angular_velocity;
    imu_data.linear_acceleration = imu_message->linear_acceleration;
}

void EKF::publishPose(geometry_msgs::Pose pose_msg) {
    // publish bot's position
    pose_pub.publish(pose_msg);
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

geometry_msgs::Quaternion EKF::angleToQuaternion(double angle) {
    angle = defineAngleInBounds(angle);

    // vector is the k identity vector [0,0,1] i.e the z axis
    geometry_msgs::Quaternion quat;
    quat.x = 0;              // v0*sin(angle/2)
    quat.y = 0;              // v1*sin(angle/2)
    quat.z = sin(angle / 2); // v2*sin(angle/2)
    quat.w = cos(angle / 2); // cos(angle/2)

    return quat;
}

double EKF::quaternionToAngle(geometry_msgs::Quaternion quat_angle) {
    double angle;

    // Use Eigen's Angle Axis class to convert a quaternion into a
    // a rotation angle around an arbitrary 3D axis (assume the z axis)
    Eigen::Quaternion<double> eigen_quat(
    quat_angle.w, quat_angle.x, quat_angle.y, quat_angle.z);
    Eigen::AngleAxis<double> ang_axis(eigen_quat);

    // if the axis is upside down e.g. [0,0,-1] then flip the sign of the angle
    if (ang_axis.axis()(2) < 0) {
        angle = -ang_axis.angle();
    } else {
        angle = ang_axis.angle();
    }

    return defineAngleInBounds(angle);
}
