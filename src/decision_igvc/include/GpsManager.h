/*
 * Created By: Gareth Ellis
 * Created On: May 17, 2017
 * Description: Manages the GPS waypoints we are given
 *              (publishing the next one once we've arrived
 *              at the current one)
 */

#ifndef DECISION_GPSManager_H
#define DECISION_GPSManager_H

#include <algorithm>
#include <geometry_msgs/Point.h>
#include <gps_common/conversions.h>
#include <ros/ros.h>
#include <sb_utils.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <stack>
#include <std_msgs/Float32.h>
#include <tf/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <vector>

// A struct to hold lat/lon coordinates (a waypoint)
// lat/lon should be in degrees
struct Waypoint {
    double lat;
    double lon;
};

class GpsManager {
  public:
    GpsManager(int argc, char** argv, std::string node_name);
    /**
     * Parses a raw list of waypoints (in order lat,lon,lat,lon,...) into a
     * vector of pairs
     *
     * @param waypoints_raw the raw waypoints to be parsed
     * @return a vector of pairs of lat/lon coordinates parsed from
     * raw_waypoints
     */
    static std::vector<Waypoint>
    parseWaypoints(std::vector<double> raw_waypoints);

  private:
    /**
     * Called whenever a new tf message is published
     *
     * Checks if we're at the waypoint yet, and starts publishing the next
     * waypoint if we are
     *
     * @param tf_message contains all the tf's currently being published on a
     * given topic
     */
    void tfCallBack(const tf2_msgs::TFMessageConstPtr tf_message);
    /**
     * Clears waypoint_stack, then puts all the elements from waypoint_list onto
     * the waypoint_stack,
     * with the 0'th element from waypoint_list at the top of waypoint_stack
     */
    void populateWaypointStack(std::vector<Waypoint> waypoint_list);

    /**
     * Publishes a marker to the next waypoint in Rviz
     * @param p the next waypoint
     * @param global_to_local_transform the transform from global to local frame
     * NOTE: Have to modify the transform so that the Z point is zeroed out
     * due to the original transform also translating in the Z direction.
     */
    void publishRvizWaypointMarker(geometry_msgs::PointStamped p, geometry_msgs::TransformStamped global_to_local_transform);

    ros::Subscriber tf_subscriber;
    ros::Publisher current_waypoint_publisher;
    ros::Publisher rviz_marker_publisher;

    std::string base_frame;   // The base frame of the robot ("base_link",
                              // "base_footprint", etc.)
    std::string global_frame; // The global frame ("map", "odom", etc.)

    // Params
    float at_goal_tolerance;
    std::stack<geometry_msgs::PointStamped> waypoint_stack; // A stack of all
                                                            // unvisited
                                                            // waypoints stored
                                                            // in the UTM
                                                            // coordinate frame
};

#endif // DECISION_GPSManager_H
