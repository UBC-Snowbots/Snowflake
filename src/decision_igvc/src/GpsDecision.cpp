/*
 * Created By: Gareth Ellis
 * Created On: May 18, 2017
 * Description: The Decision Node for GPS, takes in a point relative to
 *              the robots location and heading and broadcasts a
 *              recommended twist message. Revised to use TF's
 */

#include <GpsDecision.h>
#include <std_msgs/Float32.h>

GpsDecision::GpsDecision(int argc, char** argv, std::string node_name) {
    // Setup NodeHandles
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    // Setup Subscriber(s)
    uint32_t queue_size        = 1;
    std::string waypoint_topic = "/gps_manager/current_waypoint";
    waypoint_subscriber        = nh.subscribe(
    waypoint_topic, queue_size, &GpsDecision::waypointCallback, this);

    // Setup Publisher(s)
    std::string twist_publisher_topic = private_nh.resolveName("twist");
    twist_publisher = private_nh.advertise<geometry_msgs::Twist>(
    twist_publisher_topic, queue_size);

    // Setup the GpsMover class, which will figure out what command to send to
    // the robot
    // based on our distance and heading to the destination GPS waypoint
    double linear_heading_factor, linear_distance_factor,
    angular_heading_factor, angular_distance_factor;
    SB_getParam(
    private_nh, "linear_heading_factor", linear_heading_factor, 1.0);
    SB_getParam(
    private_nh, "linear_distance_factor", linear_distance_factor, 1.0);
    SB_getParam(
    private_nh, "angular_heading_factor", angular_heading_factor, 1.0);
    SB_getParam(
    private_nh, "angular_distance_factor", angular_distance_factor, 1.0);
    mover.setFactors(linear_distance_factor,
                     linear_heading_factor,
                     angular_distance_factor,
                     angular_heading_factor);

    double linear_cap, angular_cap;
    SB_getParam(private_nh, "max_linear_speed", linear_cap, 1.0);
    SB_getParam(private_nh, "max_angular_speed", angular_cap, 1.0);
    mover.setMaxSpeeds(linear_cap, angular_cap);

    SB_getParam(
    private_nh, "base_frame", base_frame, (std::string) "base_link");
    SB_getParam(
    private_nh, "global_frame", global_frame, (std::string) "odom_combined");
}

void GpsDecision::waypointCallback(
const geometry_msgs::PointStamped::ConstPtr& waypoint) {
    // Get the current position and heading of the robot
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener(tf_buffer);
    try {
        geometry_msgs::TransformStamped tfStamped = tf_buffer.lookupTransform(
        global_frame, base_frame, ros::Time(0), ros::Duration(1.0));
        // Get our current heading and location from the global_frame <->
        // base_frame tf
        geometry_msgs::Point current_location;
        current_location.x     = tfStamped.transform.translation.x;
        current_location.y     = tfStamped.transform.translation.y;
        double current_heading = tf::getYaw(tfStamped.transform.rotation);
        // Get the non-stamped waypoint, because we don't care about the
        // stamped info at this point
        geometry_msgs::Point non_stamped_waypoint = waypoint->point;
        // Create a new twist message and publish it
        geometry_msgs::Twist twist = mover.createTwistMessage(
        current_location, current_heading, non_stamped_waypoint);
        twist_publisher.publish(twist);
    } catch (tf2::LookupException e) {
        // If we can't lookup the tf, then warn the user and tell robot to stop
        ROS_WARN_STREAM("Could not lookup tf between " << global_frame
                                                       << " and "
                                                       << base_frame);
        // TODO: Confirm that this is initialized to 0 (or find a nice way to
        // init it to 0)
        geometry_msgs::Twist allZeroTwist;
        twist_publisher.publish(allZeroTwist);
    }
}
