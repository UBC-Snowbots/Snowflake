/*
 * Created By: Gareth Ellis
 * Created On: October 20, 2016
 * Description: Manages the GPS waypoints we are given
 *              (publishing the next one once we've arrived
 *              at the current one)
 */

#ifndef DECISION_GPSMANAGER_H
#define DECISION_GPSMANAGER_H

#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "sensor_msgs/NavSatFix.h"
#include "geometry_msgs/Point.h"
#include <sensor_msgs/Imu.h>
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

class GpsManager {
public:
    GpsManager(int argc, char **argv, std::string node_name);
    /**
     * Parses a raw list of waypoints (in order lat,lon,lat,lon,...) into a vector of pairs
     *
     * @param waypoints_raw the raw waypoints to be parsed
     * @return a vector of pairs of lat/lon coordinates parsed from raw_waypoints
     */
    static std::vector<Waypoint> parseWaypoints(std::vector<double> raw_waypoints);
    /**
     * Finds the distance between two waypoints using the haversine formula
     *
     * @param waypoint_1 the first waypoint
     * @param waypoint_2 the second waypoint
     */
    static double distanceBetweenWaypoints(Waypoint waypoint_1, Waypoint waypoint_2);
    /**
     * Find the angle between two waypoints relative to true north
     *
     * Angle is from the perspective of the first waypoint
     *
     * @ param waypoint_1 the first waypoint
     * @ param waypoint_2 the second waypoint
     *
     * @ return the angle between the two waypoints, and relative to magnetic north
     */
    static double angleBetweenWaypoints(Waypoint waypoint_1, Waypoint waypoint_2);
    /**
     * Converts a Waypoint to the robots perspective
     *
     * Converts the lat/lon coordinates to x,y coordinates in meters, relative to
     * where the robot started, taking in to account the robot's initial rotation
     *
     * @param waypoint the waypoint to convert
     * @param origin_navsatfix a navsatfix at the origin
     * @param heading the heading at the origin
     *
     * @return a geometry_msgs::Point with x and y representing the given waypoint converted to the
     *         the robots perspective
     */
    static geometry_msgs::Point convertToRobotsPerspective(Waypoint waypoint,
                                                            sensor_msgs::NavSatFix origin_navsatfix,
                                                            double heading);
private:
    ros::Subscriber raw_gps_subscriber;
    ros::Subscriber imu_subscriber;
    ros::Publisher current_location_publisher;
    ros::Publisher current_waypoint_publisher;
    ros::Publisher current_heading_publisher;

    // Params
    float at_goal_tolerance;
    std::vector<double> waypoints_raw;      // The raw list of waypoints retrieved from the param server
    std::stack<Waypoint> waypoint_stack;    // A stack of all unvisited waypoints
    std::vector<Waypoint> waypoint_list;    // A list of all the waypoints to visit

    bool received_initial_navsatfix;            // Whether or not we've received the first navsatfix yet
    sensor_msgs::NavSatFix origin_navsatfix;    // The initial nav_sat_fix at (0,0)

    double origin_heading;                      // The compass heading at initialization (relative to true north)
    double most_recent_heading;                 // The most recent heading received from the gps

    /**
     * Publishes current_position and next waypoint on each received navsatfix
     *
     * Every time a new nav_sat_fix is received, the next waypoint and current location
     * are calculated and published as points relative to the robot's initial position and
     * rotation
     *
     * @param nav_sat_fix a NavSatFix at the current location (should be straight from the gps)
     *
     */
    void rawGpsCallBack(const sensor_msgs::NavSatFix::ConstPtr nav_sat_fix);
    /**
     * Sets most_recent_heading to the new compass heading
     *
     * @param heading the most recent heading (should be directly from the compass)
     */
    void imuCallback(const sensor_msgs::Imu::ConstPtr imu_msg);
    /**
     * Returns the distance to the next waypoint in waypoints
     *
     * @param curr_location a navsatfix at the current location
     *
     * @return the distance to the next waypoint in waypoints
     */
    double distanceToNextWaypoint(sensor_msgs::NavSatFix curr_location);
    /**
     * Clears waypoint_stack, then puts all the elements from waypoint_list onto the waypoint_stack,
     * with the 0'th element from waypoint_list at the top of waypoint_stack
     */
    void populateWaypointStack();
    /**
     * Publishes the next waypoint as a location relative to the initial location and rotation
     *
     * @param waypoint the waypoint to publish
     */
    void publishWaypoint(Waypoint waypoint);
    /**
     * Publishes the current heading of the robot, relative to the initial heading being 0
     *
     * @param heading the heading to translate to the robot's perspective
     */
    void publishTranslatedHeading(float heading);
    /**
      * Publishes the current location as a location relative to the initial location and rotation
      *
      * @param curr_location the current location
      */
    void publishCurrentLocation(sensor_msgs::NavSatFix curr_location);
};


#endif //DECISION_GPSMANAGER_H
