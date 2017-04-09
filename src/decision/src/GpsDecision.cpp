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
    std::string heading_topic = "/gps_manager/current_heading";
    heading_subscriber = private_nh.subscribe(heading_topic, refresh_rate,
                                              &GpsDecision::headingCallback, this);
    std::string waypoint_topic = "/gps_manager/current_waypoint";
    waypoint_subscriber = private_nh.subscribe(waypoint_topic, refresh_rate,
                                               &GpsDecision::waypointCallback, this);

    // Setup Publisher(s)
    uint32_t queue_size = 10;
    std::string twist_publisher_topic = private_nh.resolveName("twist");
    twist_publisher = private_nh.advertise<geometry_msgs::Twist>(twist_publisher_topic, queue_size);

    // Misc.
    // Setup the mover class, which will figure out what command to send to the robot
    // based on our distance and heading to the destination GPS waypoint
    // TODO: Better factor values. (Should probably be params)
    mover.setHeadingFactor(1);
    mover.setDistanceFactor(1);
}

void GpsDecision::currentLocationCallback(const geometry_msgs::Point::ConstPtr &current_location) {
    this->current_location = *current_location;
}

void GpsDecision::headingCallback(const sensor_msgs::Imu::ConstPtr& imu_msg) {
    this->current_heading = tf::getYaw(imu_msg->orientation);
}

void GpsDecision::waypointCallback(const geometry_msgs::Point::ConstPtr &waypoint) {
    // Create a new twist message and publish it
    geometry_msgs::Twist twist = mover.createTwistMessage(current_location, current_heading, *waypoint);
    twist_publisher.publish(twist);
}

