/*
 * Created By: Gareth Ellis and Chris Chen
 * Created On: September 22, 2016
 * Description: The Decision Node for GPS, takes in a point relative to
 *              the robots location and heading and broadcasts a
 *              recommended twist message
 */

#include <GpsDecision.h>
#include <std_msgs/Float32.h>

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
    std::string imu_topic = "/gps_manager/current_heading";
    imu_subscriber = private_nh.subscribe(imu_topic, refresh_rate,
                                          &GpsDecision::imuCallback, this);
    std::string waypoint_topic = "/gps_manager/current_waypoint";
    waypoint_subscriber = private_nh.subscribe(waypoint_topic, refresh_rate,
                                               &GpsDecision::waypointCallback, this);

    // Setup Publisher(s)
    uint32_t queue_size = 10;
    std::string twist_publisher_topic = private_nh.resolveName("twist");
    twist_publisher = private_nh.advertise<geometry_msgs::Twist>(twist_publisher_topic, queue_size);

    // Setup the GpsMover class, which will figure out what command to send to the robot
    // based on our distance and heading to the destination GPS waypoint
    double linear_heading_factor, linear_distance_factor,
            angular_heading_factor, angular_distance_factor;
    double linear_cap, angular_cap;
    SB_getParam(private_nh, "linear_heading_factor", linear_heading_factor, 1.0);
    SB_getParam(private_nh, "linear_distance_factor", linear_distance_factor, 1.0);
    SB_getParam(private_nh, "angular_heading_factor", angular_heading_factor, 1.0);
    SB_getParam(private_nh, "angular_distance_factor", angular_distance_factor, 1.0);
    mover.setFactors(linear_distance_factor, linear_heading_factor,
                    angular_distance_factor, angular_heading_factor);
    SB_getParam(private_nh, "max_linear_speed", linear_cap, 1.0);
    SB_getParam(private_nh, "max_angular_speed", angular_cap, 1.0);
    mover.setMaxSpeeds(linear_cap, angular_cap);
}

void GpsDecision::currentLocationCallback(const geometry_msgs::Point::ConstPtr &current_location) {
    this->current_location = *current_location;
}

void GpsDecision::imuCallback(const std_msgs::Float32::ConstPtr &heading) {
    this->current_heading = heading->data;
}

void GpsDecision::waypointCallback(const geometry_msgs::Point::ConstPtr &waypoint) {
    // Create a new twist message and publish it
    geometry_msgs::Twist twist = mover.createTwistMessage(current_location, current_heading, *waypoint);
    twist_publisher.publish(twist);
}

