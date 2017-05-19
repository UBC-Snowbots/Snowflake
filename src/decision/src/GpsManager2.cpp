/*
 * Created By: Gareth Ellis
 * Created On: May 17, 2017
 * Description: Manages the GPS waypoints we are given
 *              (publishing the next one once we've arrived
 *              at the current one)
 */

/*
 * ~~~~~~~~~~~ CONVENTIONS USED ~~~~~~~~~~~
              +X
              ^
              |
      -θ  +<----->+ +θ
          |   |   |
          V   |   V
+Y <---------------------> -Y
              |
              |
              |
              V
              -X

 */


#include "GpsManager2.h"

GpsManager::GpsManager(int argc, char **argv, std::string node_name){
    // Setup NodeHandles
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    // Setup Subscribers
    uint32_t refresh_rate = 1;
    std::string raw_gps_topic_name = "/gps_driver/navsatfix";
    raw_gps_subscriber = private_nh.subscribe(raw_gps_topic_name, refresh_rate,
                                             &GpsManager::rawGpsCallBack, this);
    std::string imu_topic_name = "/imu";
    imu_subscriber = private_nh.subscribe(imu_topic_name, refresh_rate,
                                          &GpsManager::imuCallback, this);

    // Setup Publishers
    uint32_t queue_size = 1;
    std::string current_waypoint_topic = private_nh.resolveName("current_waypoint");
    current_waypoint_publisher = nh.advertise<geometry_msgs::PointStamped>
            (current_waypoint_topic, queue_size);

    // Get Params
    SB_getParam(private_nh, "base_frame", base_frame, (std::string)"base_link");
    SB_getParam(private_nh, "global_frame", global_frame, (std::string)"odom_combined");
    SB_getParam(private_nh, "at_goal_tolerance", at_goal_tolerance, (float)1.0);
    std::vector<double> waypoints_raw; // The raw list of waypoints retrieved from the param server
    if (!SB_getParam(private_nh, "waypoints", waypoints_raw)) {
        ROS_ERROR("Waypoints should be a list in the form [lat, lon, lat, lon, ...]");
        ros::shutdown();
    } else {
        std::vector<Waypoint>waypoint_list = parseWaypoints(waypoints_raw);
        populateWaypointStack(waypoint_list);
    }

    // Initialize misc. variables
    received_initial_navsatfix = false;
    received_initial_heading = false;

    ROS_INFO("Hold on a sec, just waiting on initial compass and gps readings");
}

void GpsManager::rawGpsCallBack(const sensor_msgs::NavSatFix::ConstPtr nav_sat_fix) {
    curr_navsatfix = *nav_sat_fix;

    if (!received_initial_navsatfix){
        received_initial_navsatfix = true;
        ROS_INFO("Received initial NavSatFix");
        if (received_initial_heading) {
            ROS_INFO("Received initial compass and gps readings, good to go!");
        }
    }

    // Don't do anything if we don't have a heading yet
    if (!received_initial_heading) return;

    // If we're at the goal, set the current waypoint to the next one in the stack
    if (waypoint_stack.size() > 0 &&
            distanceToWaypoint(waypoint_stack.top()) < at_goal_tolerance) {
        ROS_INFO("\nHit waypoint \nlat: %f \nlon: %f",
                 waypoint_stack.top().lat, waypoint_stack.top().lon);
        // Start going to the next waypoint
        waypoint_stack.pop();
    }

    // Publish the current waypoint
    if (waypoint_stack.size() > 0) {
        publishWaypoint(waypoint_stack.top());
    } else {
        ROS_INFO("Visited all waypoints");
        // TODO: This will probably need to be handled more gracefully. Right now, the robot will probably get to the final waypoint and freak out
    }
}

void GpsManager::imuCallback(const sensor_msgs::Imu::ConstPtr imu_msg) {
    // Get current heading (yaw) from the imu message
    curr_heading = tf::getYaw(imu_msg->orientation);

    if (!received_initial_heading) {
        received_initial_heading = true;
        ROS_INFO("Received initial compass heading");
        if (received_initial_navsatfix) {
            ROS_INFO("Received initial compass and gps readings, good to go!");
        }
    }
}


void GpsManager::publishWaypoint(Waypoint waypoint){
    // Get the distance to the waypoint
    double distance = distanceToWaypoint(waypoint);
    // Get the heading to the waypoint
    double heading_to_waypoint = angleToWaypoint(waypoint);

    // Calculate the x and y displacement to the waypoint in the robot's
    // frame of reference
    double d_theta = heading_to_waypoint - this->curr_heading;
    double dx = cos(d_theta) * distance;
    double dy = sin(d_theta) * distance;
    // Use the tf tree to convert the waypoint to the global frame of reference
    geometry_msgs::PointStamped point_msg;
    point_msg.point.x = dx;
    point_msg.point.y = dy;
    point_msg.point.z = 0;
    point_msg.header.frame_id = base_frame;
    point_msg.header.stamp = curr_navsatfix.header.stamp;
    // TODO: Check if frames exist first and warn and do nothing if not
    SB_doTransform(point_msg, point_msg, global_frame);
    current_waypoint_publisher.publish(point_msg);
}

double GpsManager::distanceToWaypoint(Waypoint waypoint){
    Waypoint temp_waypoint;
    temp_waypoint.lat = curr_navsatfix.latitude;
    temp_waypoint.lon = curr_navsatfix.longitude;
    return distanceBetweenWaypoints(temp_waypoint, waypoint_stack.top());
}

double GpsManager::distanceBetweenWaypoints(Waypoint waypoint_1, Waypoint waypoint_2){
    // Convert lat/lon from degrees to radians
    double lat1 = waypoint_1.lat * (M_PI/180);
    double lon1 = waypoint_1.lon * (M_PI/180);
    double lat2 = waypoint_2.lat * (M_PI/180);
    double lon2 = waypoint_2.lon * (M_PI/180);

    // Find distance using Haversine formula
    double distance = pow(sin((lat2 - lat1)/2), 2) + cos(lat1)* cos(lat2) * pow(sin((lon2 - lon1)/2),2);
    distance = 2 * atan2(sqrt(distance), sqrt(1-distance));
    distance *= 6371000; // Earth Radius in meters

    return distance;
}

double GpsManager::angleToWaypoint(Waypoint waypoint){
    Waypoint temp_waypoint;
    temp_waypoint.lat = curr_navsatfix.latitude;
    temp_waypoint.lon = curr_navsatfix.longitude;
    return angleBetweenWaypoints(temp_waypoint, waypoint_stack.top());
}

double GpsManager::angleBetweenWaypoints(Waypoint waypoint_1, Waypoint waypoint_2){
    // Convert lat/lon from degrees to radians
    double lat1 = waypoint_1.lat * (M_PI/180);
    double lon1 = waypoint_1.lon * (M_PI/180);
    double lat2 = waypoint_2.lat * (M_PI/180);
    double lon2 = waypoint_2.lon * (M_PI/180);

    double delta_lon = lon2 - lon1;
    double y = sin(delta_lon) * cos(lat2);
    double x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(delta_lon);
    // Atan2 returns a value between -Pi and Pi, which we want to normalize to
    // between 0 and 2Pi
    double angle = fmod((atan2(y,x) + (2 * M_PI)), 2 * M_PI);
    // We also need to make positive rotation COUNTER-clockwise as per the standard
    // ROS frame of reference
    angle = (2 * M_PI) - angle;
    return angle;
}

std::vector<Waypoint> GpsManager::parseWaypoints(std::vector<double> waypoints_raw){
    if (waypoints_raw.size() % 2 != 0){
        ROS_ERROR("Given odd length list of waypoints, check that \"waypoints\" parameter "
                          "has an (numerically) even length. As this is a series of lat/lon coordinates,"
                          "it should always be even");
        ros::shutdown();
        return {};
    }

    std::vector<Waypoint> waypoints;
    for (int i = 0; i < waypoints_raw.size(); i += 2){
        Waypoint curr_waypoint;
        curr_waypoint.lat = waypoints_raw[i];
        curr_waypoint.lon = waypoints_raw[i+1];
        waypoints.push_back(curr_waypoint);
    }

    return waypoints;
}

void GpsManager::populateWaypointStack(std::vector<Waypoint> waypoint_list){
    for (int i = (int)waypoint_list.size() - 1; i >= 0; i--){
        waypoint_stack.push(waypoint_list[i]);
    }
}

