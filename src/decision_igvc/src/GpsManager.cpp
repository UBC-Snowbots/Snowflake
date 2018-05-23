/*
 * Created By: Gareth Ellis
 * Created On: May 17, 2017
 * Description: Manages the GPS waypoints we are given
 *              (publishing the next one once we've arrived
 *              at the current one)
 */

#include <GpsManager.h>
#include <RvizUtils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

GpsManager::GpsManager(int argc, char** argv, std::string node_name) {
    // Setup NodeHandles
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    // Setup Subscribers
    uint32_t queue_size       = 1;
    std::string tf_topic_name = "/tf";
    tf_subscriber =
    nh.subscribe(tf_topic_name, queue_size, &GpsManager::tfCallBack, this);

    // Setup Publishers
    std::string current_waypoint_topic =
    private_nh.resolveName("current_waypoint");
    current_waypoint_publisher = nh.advertise<geometry_msgs::PointStamped>(
    current_waypoint_topic, queue_size);
    std::string marker_topic = private_nh.resolveName("current_waypoint_marker");
    rviz_marker_publisher = nh.advertise<visualization_msgs::Marker>(
            marker_topic, queue_size);

    // Get Params
    SB_getParam(
    private_nh, "base_frame", base_frame, (std::string) "base_link");
    SB_getParam(
    private_nh, "global_frame", global_frame, (std::string) "odom_combined");
    SB_getParam(
    private_nh, "at_goal_tolerance", at_goal_tolerance, (float) 1.0);
    std::vector<double>
    waypoints_raw; // The raw list of waypoints retrieved from the param server
    if (!SB_getParam(private_nh, "waypoints", waypoints_raw)) {
        ROS_ERROR(
        "Waypoints should be a list in the form [lat, lon, lat, lon, ...]");
        ros::shutdown();
    } else {
        std::vector<Waypoint> waypoint_list = parseWaypoints(waypoints_raw);
        populateWaypointStack(waypoint_list);
    }
}

void GpsManager::tfCallBack(const tf2_msgs::TFMessageConstPtr tf_message) {
    // If we've visited all the waypoints, no need to do anything here,
    // In fact, we can celebrate and shutdown
    if (waypoint_stack.size() <= 0) {
        ROS_INFO_STREAM("Reached all waypoints! *cue digital clapping*");
        ros::shutdown();
        return;
    }

    // Set the TimeStamp on the current waypoint and publish it
    waypoint_stack.top().header.stamp = ros::Time::now();
    current_waypoint_publisher.publish(waypoint_stack.top());


    // Check if the tf_message contains the transform we're looking for
    for (geometry_msgs::TransformStamped tf_stamped : tf_message->transforms) {
        if (tf_stamped.header.frame_id == global_frame &&
            tf_stamped.child_frame_id == base_frame) {
            // We've found the transform we're looking for, so see how close we
            // are to the waypoint
            geometry_msgs::PointStamped curr_waypoint = waypoint_stack.top();
            double dx =
            tf_stamped.transform.translation.x - curr_waypoint.point.x;
            double dy =
            tf_stamped.transform.translation.y - curr_waypoint.point.y;
            double distance = sqrt(pow(dx, 2) + pow(dy, 2));
            if (distance < at_goal_tolerance) {
                waypoint_stack.pop();
                ROS_INFO_STREAM("Hit waypoint " << waypoint_stack.size());
            }
            publishRvizWaypointMarker(curr_waypoint, tf_stamped);
        }
    }
}

void GpsManager::publishRvizWaypointMarker(geometry_msgs::PointStamped p,
                                           geometry_msgs::TransformStamped global_to_local_transform) {

    // Inverse the transform
    tf2::Transform t;
    tf2::fromMsg(global_to_local_transform.transform, t);
    t = t.inverse();

    // Create a new geometry_msgs::Transform
    geometry_msgs::TransformStamped t_stamped;
    t_stamped.transform = tf2::toMsg(t);
    t_stamped.header = global_to_local_transform.header;
    t_stamped.header.frame_id = base_frame;

    // Define a new point relative to base frame
    geometry_msgs::PointStamped output;
    tf2::doTransform(p, output, t_stamped);
    output.point.z = 0; // Necessary to remove vertical transform of waypoint

    // Create marker
    std::vector<std_msgs::ColorRGBA> colors;
    std_msgs::ColorRGBA color;
    color.a = 1.0f;
    color.r = 1.0f;
    colors.push_back(color);
    rviz_marker_publisher.publish(snowbots::RvizUtils::createMarker(output.point, colors, snowbots::RvizUtils::createrMarkerScale(0.5, 0.5, 0.5), base_frame, "debug", visualization_msgs::Marker::POINTS));
}

std::vector<Waypoint>
GpsManager::parseWaypoints(std::vector<double> waypoints_raw) {
    if (waypoints_raw.size() % 2 != 0) {
        ROS_ERROR(
        "Given odd length list of waypoints, check that \"waypoints\" "
        "parameter "
        "has an (numerically) even length. As this is a series of lat/lon "
        "coordinates,"
        "it should always be even");
        ros::shutdown();
        return {};
    }

    std::vector<Waypoint> waypoints;
    for (int i = 0; i < waypoints_raw.size(); i += 2) {
        Waypoint curr_waypoint;
        curr_waypoint.lat = waypoints_raw[i];
        curr_waypoint.lon = waypoints_raw[i + 1];
        waypoints.push_back(curr_waypoint);
    }

    return waypoints;
}

void GpsManager::populateWaypointStack(std::vector<Waypoint> waypoint_list) {
    for (int i = (int) waypoint_list.size() - 1; i >= 0; i--) {
        // Convert the waypoint to UTM
        double northing, easting;
        std::string zone_throwaway;
        gps_common::LLtoUTM(waypoint_list[i].lat,
                            waypoint_list[i].lon,
                            northing,
                            easting,
                            zone_throwaway);

        // Build a point with our UTM coordinates
        geometry_msgs::PointStamped point_stamped;
        point_stamped.header.frame_id = global_frame;
        point_stamped.header.stamp    = ros::Time();
        point_stamped.point.x         = easting;
        point_stamped.point.y         = northing;

        waypoint_stack.push(point_stamped);
    }

    ROS_INFO_STREAM("Ready to rock 'n' roll with " << waypoint_stack.size()
                                                   << " waypoints!");
}
