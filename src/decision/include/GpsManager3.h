/*
 * Created By: Gareth Ellis
 * Created On: May 17, 2017
 * Description: Manages the GPS waypoints we are given
 *              (publishing the next one once we've arrived
 *              at the current one)
 */

#ifndef DECISION_GPSMANAGER3_H
#define DECISION_GPSMANAGER3_H

#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "sensor_msgs/NavSatFix.h"
#include "geometry_msgs/Point.h"
#include <sensor_msgs/Imu.h>
#include <gps_conversions.h>
#include <tf/transform_datatypes.h>
#include <vector>
#include <algorithm>
#include <stack>
#include <sb_utils.h>

// A struct to hold lat/lon coordinates (a waypoint)
// lat/lon should be in degrees
struct Waypoint {
    double lat;
    double lon;
};

class GpsManager3 {
public:
    GpsManager3(int argc, char **argv, std::string node_name);
    /**
     * Parses a raw list of waypoints (in order lat,lon,lat,lon,...) into a vector of pairs
     *
     * @param waypoints_raw the raw waypoints to be parsed
     * @return a vector of pairs of lat/lon coordinates parsed from raw_waypoints
     */
    static std::vector<Waypoint> parseWaypoints(std::vector<double> raw_waypoints);

private:
    /**
     * Called whenever a new tf message is published
     *
     * Checks if we're at the waypoint yet, and starts publishing the next waypoint if we are
     *
     * @param tf_message contains all the tf's currently being published on a given topic
     */
    void tfCallBack(const tf2_msgs::TFMessageConstPtr tf_message);
    /**
     * Clears waypoint_stack, then puts all the elements from waypoint_list onto the waypoint_stack,
     * with the 0'th element from waypoint_list at the top of waypoint_stack
     */
    void populateWaypointStack(std::vector<Waypoint> waypoint_list);

    ros::Subscriber tf_subscriber;
    ros::Publisher current_waypoint_publisher;

    std::string base_frame; // The base frame of the robot ("base_link", "base_footprint", etc.)
    std::string global_frame; // The global frame ("map", "odom", etc.)

    // Params
    float at_goal_tolerance;
    std::stack<geometry_msgs::PointStamped> waypoint_stack; // A stack of all unvisited waypoints stored in the UTM coordinate frame

};


#endif //DECISION_GPSMANAGER3_H
