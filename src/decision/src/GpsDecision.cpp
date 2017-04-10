/*
 * Created By: Gareth Ellis and Chris Chen
 * Created On: September 22, 2016
 * Description: The Decision Node for GPS, takes in a point relative to
 *              the robots location and heading and broadcasts a
 *              recommended twist message
 */

#include <GpsDecision.h>

GpsDecision::GpsDecision(int argc, char **argv, std::string node_name) {
    // Setup NodeHandles
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    // Setup Subscriber(s)
    uint32_t refresh_rate = 10;
    std::string current_location_topic = "/gps_manager/current_location";
    current_location_subscriber = private_nh.subscribe(current_location_topic, refresh_rate,
                                                    &GpsDecision::currentLocationCallback, this);
    std::string imu_topic = "/imu";
    imu_subscriber = private_nh.subscribe(imu_topic, refresh_rate,
                                          &GpsDecision::imuCallback, this);
    std::string waypoint_topic = "/gps_manager/current_waypoint";
    waypoint_subscriber = private_nh.subscribe(waypoint_topic, refresh_rate,
                                               &GpsDecision::waypointCallback, this);

    // Setup Publisher(s)
    uint32_t queue_size = 10;
    std::string twist_publisher_topic = private_nh.resolveName("twist");
    twist_publisher = private_nh.advertise<geometry_msgs::Twist>(twist_publisher_topic, queue_size);

    // Setup the mover class, which will figure out what command to send to the robot
    // based on our distance and heading to the destination GPS waypoint
    double heading_factor, distance_factor;
    SB_getParam(private_nh, "heading_factor", heading_factor, 1.0);
    SB_getParam(private_nh, "distance_factor", distance_factor, 1.0);
    mover.setHeadingFactor(heading_factor);
    mover.setDistanceFactor(distance_factor);
}

void GpsDecision::currentLocationCallback(const geometry_msgs::Point::ConstPtr &current_location) {
    this->current_location = *current_location;
}

void GpsDecision::imuCallback(const sensor_msgs::Imu::ConstPtr &imu_msg) {
    this->current_heading = tf::getYaw(imu_msg->orientation);
}

void GpsDecision::waypointCallback(const geometry_msgs::Point::ConstPtr &waypoint) {
    // Create a new twist message and publish it
    geometry_msgs::Twist twist = mover.createTwistMessage(current_location, current_heading, *waypoint);
    twist_publisher.publish(twist);
}

