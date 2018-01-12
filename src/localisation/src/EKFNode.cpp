/*
 * Created By: Marcus Swift
 * Created On: November 20th, 2017
 * Description: An extended kalman filter node that takes in sensor data from
 * the GPS, encoders and IMU and returns the bots estimated postion and
 * orientation using the ekf class
 */

#include <EKF.h>
#include <EKFNode.h>

EKFNode::EKFNode(int argc, char** argv, std::string node_name) {
    // Setup NodeHandle
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;

    // Setup Subscribers
    gps_sub = nh.subscribe("/gps_driver/odom", 1, &EKFNode::gpsCallBack, this);
    encoder_sub = nh.subscribe("/odom", 1, &EKFNode::encoderCallBack, this);
    imu_sub     = nh.subscribe("/imu", 1, &EKFNode::imuCallBack, this);

    // Setup Publisher
    pose_pub = nh.advertise<geometry_msgs::Pose>("/cmd_pose", 1000);

    // Get Parameters
    double sdev_gyro, sdev_speed, sdev_gps_x_measurement,
    sdev_gps_y_measurement, sdev_angle_measurement, initial_pos_x,
    initial_pos_y, initial_yaw_angle;
    std::vector<double> initial_p(9);
    std::vector<double> q1_const(9);
    std::vector<double> h_const(9);
    nh.param<double>("ekf_node/sdev_gyro", sdev_gyro, M_PI / 180.);
    nh.param<double>("ekf_node/sdev_speed", sdev_speed, 0.15);
    nh.param<double>(
    "ekf_node/sdev_gps_x_measurement", sdev_gps_x_measurement, 5.);
    nh.param<double>(
    "ekf_node/sdev_gps_y_measurement", sdev_gps_y_measurement, 5.);
    nh.param<double>(
    "ekf_node/sdev_angle_measurement", sdev_angle_measurement, M_PI / 180.);
    nh.param<std::vector<double>>(
    "ekf_node/p", initial_p, {0., 0., 0., 0., 0., 0., 0., 0., 0.});
    nh.param<std::vector<double>>(
    "ekf_node/q1",
    q1_const,
    {0.01 * 0.01,
     0.,
     0.,
     0.,
     0.01 * 0.01,
     0.,
     0.,
     0.,
     (1. * (M_PI / 180.)) * (1. * (M_PI / 180.))});
    nh.param<std::vector<double>>(
    "ekf_node/h", h_const, {1., 0., 0., 0., 1., 0., 0., 0., 1.});
    nh.param<double>("ekf_node/initial_pos_x", initial_pos_x, 481917.);
    nh.param<double>("ekf_node/initial_pos_y", initial_pos_y, 5456662.);
    nh.param<double>("ekf_node/initial_yaw_angle", initial_yaw_angle, 0.);

    // set initial position
    setInitialPosition(initial_pos_x, initial_pos_y, initial_yaw_angle);

    // initialise out measurements
    imu_data.angular_velocity.z = 0;
    imu_data.orientation = tf::createQuaternionMsgFromYaw(initial_yaw_angle);
    gps_data.pose.pose.position.x     = initial_pos_x;
    gps_data.pose.pose.position.y     = initial_pos_y;
    encoder_data.twist.twist.linear.x = 0;

    // set constant matrices
    setConstants(sdev_gyro,
                 sdev_speed,
                 sdev_gps_x_measurement,
                 sdev_gps_y_measurement,
                 sdev_angle_measurement,
                 initial_p,
                 q1_const,
                 h_const);

    // initial time
    time = ros::Time::now().toSec();
}

void EKFNode::gpsCallBack(const nav_msgs::Odometry::ConstPtr& gps_message) {
    // Gather GPS data currently, assume gps updates slower than encoder and imu
    gps_data.pose  = gps_message->pose;
    gps_data.twist = gps_message->twist;

    publishPose(updateState(gps_data, imu_data));
}

void EKFNode::encoderCallBack(
const nav_msgs::Odometry::ConstPtr& encoder_message) {
    // Gather encoder data, currently assume encoder updates slower than imu
    encoder_data.pose  = encoder_message->pose;
    encoder_data.twist = encoder_message->twist;

    double dt = ros::Time::now().toSec() -
                time; // set change in time to 1 second if running the rostest
    time += dt;

    predictState(encoder_data, imu_data, dt);
}

void EKFNode::imuCallBack(const sensor_msgs::Imu::ConstPtr& imu_message) {
    // Gather IMU data
    imu_data.orientation         = imu_message->orientation;
    imu_data.angular_velocity    = imu_message->angular_velocity;
    imu_data.linear_acceleration = imu_message->linear_acceleration;
}

void EKFNode::publishPose(geometry_msgs::Pose pose_msg) {
    // publish bot's position
    pose_pub.publish(pose_msg);
}