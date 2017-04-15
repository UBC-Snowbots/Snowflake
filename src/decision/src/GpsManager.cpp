/*
 * Created By: Gareth Ellis
 * Created On: October 20, 2016
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


#include "GpsManager.h"

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
    std::string current_location_topic = private_nh.resolveName("current_location");
    current_location_publisher = nh.advertise<geometry_msgs::Point>(current_location_topic, queue_size);
    std::string current_waypoint_topic = private_nh.resolveName("current_waypoint");
    current_waypoint_publisher = nh.advertise<geometry_msgs::Point>(current_waypoint_topic, queue_size);
    std::string current_heading_topic = private_nh.resolveName("current_heading");
    current_heading_publisher = nh.advertise<std_msgs::Float32>(current_heading_topic, queue_size);

    // Get Params
    SB_getParam(private_nh, "at_goal_tolerance", at_goal_tolerance, (float)1.0);
    if (!SB_getParam(private_nh, "waypoints", waypoints_raw)){
        ROS_ERROR("Waypoints should be a list in the form [lat, lon, lat, lon, ...]");
    } else {
        waypoint_list = parseWaypoints(waypoints_raw);
        populateWaypointStack();
    }

    // Initialize misc. variables
    received_initial_navsatfix = false;
    origin_heading = -1;
    most_recent_heading = -1;

    ROS_INFO("Hold on a sec, just waiting on initial compass and gps readings");
}

void GpsManager::imuCallback(const sensor_msgs::Imu::ConstPtr imu_msg){
    // Get current heading (yaw) from the imu message
    float curr_heading = (float)tf::getYaw(imu_msg->orientation);
    // If this is the first received compass message, make it origin_heading
    if (origin_heading == -1){
        origin_heading = curr_heading;
        ROS_INFO("Received initial compass reading");
        if (received_initial_navsatfix){
            ROS_INFO("Received initial compass and gps readings, good to go!");
        }
    }
    // Received heading is now the most recent one
    most_recent_heading = curr_heading;
    // Publish the current heading of the robot, relative to the initial heading being 0 degrees
    publishTranslatedHeading(curr_heading);
}

void GpsManager::rawGpsCallBack(const sensor_msgs::NavSatFix::ConstPtr nav_sat_fix) {
    // Set origin_navsatfix to the first navsatfix we receive
    if (!received_initial_navsatfix){
        origin_navsatfix = *nav_sat_fix;
        received_initial_navsatfix = true;
        ROS_INFO("Received initial NavSatFix");
        if (origin_heading != -1){
            ROS_INFO("Received initial compass and gps readings, good to go!");
        }
    }
    // Make sure we've got origin readings before broadcasting waypoints
    if (origin_heading != -1 && received_initial_navsatfix) {
        // If we're at the goal, start publishing the next waypoint
        if (waypoint_stack.size() > 0 && distanceToNextWaypoint(*nav_sat_fix) < at_goal_tolerance) {
            ROS_INFO("\nHit waypoint \nlat: %f \nlon: %f",
                    waypoint_stack.top().lat, waypoint_stack.top().lon);
            // Start going to the next waypoint
            waypoint_stack.pop();
        }
        if (waypoint_stack.size() > 0) {
            publishWaypoint(waypoint_stack.top());
        } else {
            ROS_INFO("Visited all waypoints");
            // TODO: This will probably need to be handled more gracefully. Right now, the robot will probably get to the final waypoint and freak out
        }
        // Convert the nav_sat_fix to a Point with x,y in meters, and relative to the starting location
        // and rotation
        publishCurrentLocation(*nav_sat_fix);
    }
}

void GpsManager::publishTranslatedHeading(float heading){
    // Subtract the initial heading from the current heading
    std_msgs::Float32 translated_heading = std_msgs::Float32();
    translated_heading.data = heading - origin_heading;
    current_heading_publisher.publish(translated_heading);
}

void GpsManager::publishCurrentLocation(sensor_msgs::NavSatFix curr_location){
    Waypoint tmp_waypoint;
    tmp_waypoint.lat = curr_location.latitude;
    tmp_waypoint.lon = curr_location.longitude;
    geometry_msgs::Point point_msg = convertToRobotsPerspective(
            tmp_waypoint, origin_navsatfix, origin_heading);
    current_location_publisher.publish(point_msg);
}

void GpsManager::publishWaypoint(Waypoint waypoint){
    geometry_msgs::Point point_msg = convertToRobotsPerspective(waypoint, origin_navsatfix, origin_heading);
    current_waypoint_publisher.publish(point_msg);
}

geometry_msgs::Point GpsManager::convertToRobotsPerspective(Waypoint waypoint,
                                                            sensor_msgs::NavSatFix origin_navsatfix,
                                                            double origin_heading){
    Waypoint origin_waypoint;
    origin_waypoint.lat = origin_navsatfix.latitude;
    origin_waypoint.lon = origin_navsatfix.longitude;
    // Get distance and angle between the waypoint and the origin
    double origin_to_waypoint_distance = distanceBetweenWaypoints(origin_waypoint, waypoint);
    double origin_to_waypoint_angle = angleBetweenWaypoints(origin_waypoint, waypoint);
    // Convert the waypoint to x,y coordinates in the robots frame of reference
    double gps_x = cos(origin_to_waypoint_angle) * origin_to_waypoint_distance; // x value from gps perspective
    double gps_y = sin(origin_to_waypoint_angle) * origin_to_waypoint_distance; // y value from gps perspective
    // Convert gps x,y to robot's frame of reference using a rotation matrix
    geometry_msgs::Point point_msg;
    origin_heading *= -1;
    point_msg.x = cos(origin_heading) * gps_x + sin(origin_heading) * gps_y;
    point_msg.y = sin(origin_heading) * gps_x - cos(origin_heading) * gps_y;
    return point_msg;
}

std::vector<Waypoint> GpsManager::parseWaypoints(std::vector<double> waypoints_raw){
    if (waypoints_raw.size() % 2 != 0){
        ROS_ERROR("Given odd length list of waypoints, check that \"waypoints\" parameter "
                  "has an (numerically) even length. As this is a series of lat/lon coordinates,"
                  "it should always be even");
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

double GpsManager::distanceToNextWaypoint(sensor_msgs::NavSatFix curr_navsatfix){
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

double GpsManager::angleBetweenWaypoints(Waypoint waypoint_1, Waypoint waypoint_2){
    // Convert lat/lon from degrees to radians
    double lat1 = waypoint_1.lat * (M_PI/180);
    double lon1 = waypoint_1.lon * (M_PI/180);
    double lat2 = waypoint_2.lat * (M_PI/180);
    double lon2 = waypoint_2.lon * (M_PI/180);

    double delta_lon = lon2 - lon1;
    double y = sin(delta_lon) * cos(lat2);
    double x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(delta_lon);
    // Atan2 returns a value between -Pi and Pi, which we want to normalize to a compass bearing
    double angle = fmod((atan2(y,x) + (2 * M_PI)), 2 * M_PI);
    return angle;
}

void GpsManager::populateWaypointStack(){
    for (int i = (int)waypoint_list.size() - 1; i >= 0; i--){
        waypoint_stack.push(waypoint_list[i]);
    }
}
