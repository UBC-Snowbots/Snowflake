/*
 * Created By: Gareth Ellis
 * Created On: October 21, 2016
 * Description: Tests for GpsManager
 * Notes:
 * Some really great sites for generating test data:
 * REMEMBER YOU'RE USING A SPHERE MODEL FOR THE EARTH
 * http://www.geomidpoint.com/destination/
 * http://www.movable-type.co.uk/scripts/latlong.html
 */


#include <GpsManager.h>
#include <gtest/gtest.h>

/**
 * Converts a vector of waypoints to pairs for testing with the form (lat,lon)
 *
 * @param waypoints vector of waypoints
 *
 * @return the waypoints as a vector of doubles
 */
std::vector<std::pair<double, double>> pairsFromWaypointVector(std::vector<Waypoint> waypoints){
    std::vector<std::pair<double, double>> new_waypoints;
    for (int i = 0; i < waypoints.size(); i++){
        std::pair<double,double> new_waypoint (waypoints[i].lat, waypoints[i].lon);
        new_waypoints.push_back(new_waypoint);
    }
    return new_waypoints;
}

TEST(GpsManager, parseWaypoints){
    std::vector<std::pair<double, double>> empty_result = {};
    std::vector<double> odd_length_input (9, 10.123123);

    std::vector<std::pair<double, double>> one_waypoint_result = {std::pair<double, double> (9.87,10.98)};
    std::vector<double> one_waypoint_input = {9.87, 10.98};

    std::vector<std::pair<double,double>> multi_waypoint_result = {
            std::pair<double, double> (9.87, 10.98),
            std::pair<double, double> (98.123, 97.122),
            std::pair<double, double> (87.12, 12.12)
    };
    std::vector<double> multi_waypoint_input = {9.87, 10.98, 98.123, 97.122, 87.12, 12.12};

    EXPECT_EQ(empty_result, pairsFromWaypointVector(GpsManager::parseWaypoints(odd_length_input)));
    EXPECT_EQ(one_waypoint_result, pairsFromWaypointVector(GpsManager::parseWaypoints(one_waypoint_input)));
    EXPECT_EQ(multi_waypoint_result, pairsFromWaypointVector(GpsManager::parseWaypoints(multi_waypoint_input)));

}

TEST(GpsManager, distanceBetweenWaypoints300Meters){
    Waypoint wp1, wp2;
    wp1.lat = 49.254229;
    wp1.lon = -123.240815;
    wp2.lat = 49.255595;
    wp2.lon = -123.236577;
    EXPECT_NEAR(343.0391, GpsManager::distanceBetweenWaypoints(wp1, wp2), 0.001);
}

TEST(GpsManager, distanceBetweenWaypointsLessThenAMeter){
    Waypoint wp1, wp2;
    wp1.lat = 49.254226;
    wp1.lon = -123.240825;
    wp2.lat = 49.254229;
    wp2.lon = -123.240832;
    EXPECT_NEAR(0.60777, GpsManager::distanceBetweenWaypoints(wp1, wp2), 0.001);
}

TEST(GpsManager, angleBetweenWaypoints0){
    Waypoint wp1, wp2;
    wp1.lat = 49.254226;
    wp1.lon = -123.240825;
    wp2.lat = 49.25423499;
    wp2.lon = -123.240825;
    EXPECT_NEAR(0, GpsManager::angleBetweenWaypoints(wp1, wp2), 0.001);
}

TEST(GpsManager, angleBetweenWaypoints10Degrees){
    Waypoint wp1, wp2;
    wp1.lat = 49.254226;
    wp1.lon = -123.240825;
    wp2.lat = 49.25423486;
    wp2.lon = -123.24082261;
    EXPECT_NEAR(10 * (M_PI/180), GpsManager::angleBetweenWaypoints(wp1, wp2), 0.001);
}

TEST(GpsManager, angleBetweenWaypoints180Degrees){
    Waypoint wp1, wp2;
    wp1.lat = 49.254226;
    wp1.lon = -123.240825;
    wp2.lat = 49.25421701;
    wp2.lon = -123.240825;
    EXPECT_NEAR(M_PI, GpsManager::angleBetweenWaypoints(wp1, wp2), 0.001);
}

TEST(GpsManager, angleBetweenWaypoints270Degrees){
    Waypoint wp1, wp2;
    wp1.lat = 49.254226;
    wp1.lon = -123.240825;
    wp2.lat = 49.254226;
    wp2.lon = -123.24083874;
    EXPECT_NEAR(1.5 * M_PI, GpsManager::angleBetweenWaypoints(wp1, wp2), 0.001);
}

TEST(GpsManager, angleBetweenWaypoints359Degrees){
    Waypoint wp1, wp2;
    wp1.lat = 49.254226;
    wp1.lon = -123.240825;
    wp2.lat = 49.25423499;
    wp2.lon = -123.24082524;
    EXPECT_NEAR(359 * (M_PI/180), GpsManager::angleBetweenWaypoints(wp1, wp2), 0.001);
}

TEST(GpsManager, convertToRobotsPrespective_no_rotation_straight_ahead){
    // Waypoint is 10 meters directly ahead
    // Origin rotation is 0
    Waypoint wp;
    sensor_msgs::NavSatFix nsfix;
    nsfix.latitude = 49.254226;
    nsfix.longitude = -123.240825;
    wp.lat = 49.25431591;
    wp.lon = -123.240825;

    geometry_msgs::Point converted_waypoint = GpsManager::convertToRobotsPerspective(wp, nsfix, 0);
    EXPECT_NEAR(10, converted_waypoint.x, 0.005);
    EXPECT_NEAR(0, converted_waypoint.y, 0.005);
}

TEST(GpsManager, convertToRobotsPrespective_no_rotation_directly_behind){
    // Waypoint is 10 meters directly behind
    // Origin rotation is 0
    Waypoint wp;
    sensor_msgs::NavSatFix nsfix;
    nsfix.latitude = 49.25431591;
    nsfix.longitude = -123.240825;
    wp.lat = 49.254226;
    wp.lon = -123.240825;

    geometry_msgs::Point converted_waypoint = GpsManager::convertToRobotsPerspective(wp, nsfix, 0);
    EXPECT_NEAR(-10, converted_waypoint.x, 0.005);
    EXPECT_NEAR(0, converted_waypoint.y, 0.005);
}

TEST(GpsManager, convertToRobotsPrespective_no_rotation_directly_left){
    // Waypoint is 10 meters directly ahead
    // Origin rotation is 0
    Waypoint wp;
    sensor_msgs::NavSatFix nsfix;
    nsfix.latitude = 49.254226;
    nsfix.longitude = -123.240825;
    wp.lat = 49.254226;
    wp.lon = -123.24096275;

    geometry_msgs::Point converted_waypoint = GpsManager::convertToRobotsPerspective(wp, nsfix, 0);
    EXPECT_NEAR(0, converted_waypoint.x, 0.005);
    EXPECT_NEAR(10, converted_waypoint.y, 0.005);
}

TEST(GpsManager, convertToRobotsPrespective_no_rotation_directly_right){
    // Waypoint is 10 meters directly ahead
    // Origin rotation is 0
    Waypoint wp;
    sensor_msgs::NavSatFix nsfix;
    nsfix.latitude = 49.254226;
    nsfix.longitude = -123.24096275;
    wp.lat = 49.254226;
    wp.lon = -123.240825;

    geometry_msgs::Point converted_waypoint = GpsManager::convertToRobotsPerspective(wp, nsfix, 0);
    EXPECT_NEAR(0, converted_waypoint.x, 0.005);
    EXPECT_NEAR(-10, converted_waypoint.y, 0.005);
}

TEST(GpsManager, convertToRobotsPrespective_origin_rotated_left_straight_ahead){
    // Waypoint is 10 meters directly ahead
    Waypoint wp;
    sensor_msgs::NavSatFix nsfix;
    nsfix.latitude = 49.254226;
    nsfix.longitude = -123.240825;
    wp.lat = 49.25431591;
    wp.lon = -123.240825;
    double origin_heading = 0.5 * M_PI;

    geometry_msgs::Point converted_waypoint = GpsManager::convertToRobotsPerspective(wp, nsfix, origin_heading);
    EXPECT_NEAR(0, converted_waypoint.x, 0.005);
    EXPECT_NEAR(-10, converted_waypoint.y, 0.005);
}

TEST(GpsManager, convertToRobotsPrespective_origin_rotated_left_directly_left){
    // Waypoint is 10 meters directly ahead
    Waypoint wp;
    sensor_msgs::NavSatFix nsfix;
    nsfix.latitude = 49.254226;
    nsfix.longitude = -123.240825;
    wp.lat = 49.254226;
    wp.lon = -123.24096275;
    double origin_heading = 0.5 * M_PI;

    geometry_msgs::Point converted_waypoint = GpsManager::convertToRobotsPerspective(wp, nsfix, origin_heading);
    EXPECT_NEAR(10, converted_waypoint.x, 0.005);
    EXPECT_NEAR(0, converted_waypoint.y, 0.005);
}

TEST(GpsManager, convertToRobotsPrespective_origin_rotated_left_directly_behind){
    // Waypoint is 10 meters directly behind
    Waypoint wp;
    sensor_msgs::NavSatFix nsfix;
    nsfix.latitude = 49.25431591;
    nsfix.longitude = -123.240825;
    wp.lat = 49.254226;
    wp.lon = -123.240825;
    double origin_heading = 0.5 * M_PI;

    geometry_msgs::Point converted_waypoint = GpsManager::convertToRobotsPerspective(wp, nsfix, origin_heading);
    EXPECT_NEAR(0, converted_waypoint.x, 0.005);
    EXPECT_NEAR(10, converted_waypoint.y, 0.005);
}

TEST(GpsManager, convertToRobotsPrespective_rotated_45_straight_ahead){
    // Waypoint is 10 meters directly ahead
    // Origin rotation is Pi/2
    Waypoint wp;
    sensor_msgs::NavSatFix nsfix;
    nsfix.latitude = 49.254226;
    nsfix.longitude = -123.240825;
    wp.lat = 49.25431591;
    wp.lon = -123.240825;
    double origin_heading = M_PI/4;

    geometry_msgs::Point converted_waypoint = GpsManager::convertToRobotsPerspective(wp, nsfix, origin_heading);
    EXPECT_NEAR(10 * cos(origin_heading), converted_waypoint.x, 0.005);
    EXPECT_NEAR(-10 * cos(origin_heading), converted_waypoint.y, 0.005);
}

TEST(GpsManager, convertToRobotsPrespective_rotated_60_straight_ahead){
    // Waypoint is 10 meters directly ahead
    // Origin rotation is 60 degrees
    Waypoint wp;
    sensor_msgs::NavSatFix nsfix;
    nsfix.latitude = 49.254226;
    nsfix.longitude = -123.240825;
    wp.lat = 49.25431591;
    wp.lon = -123.240825;
    double origin_heading = 60 * (M_PI/180);

    geometry_msgs::Point converted_waypoint = GpsManager::convertToRobotsPerspective(wp, nsfix, origin_heading);
    EXPECT_NEAR(10 * cos(origin_heading), converted_waypoint.x, 0.005);
    EXPECT_NEAR(-10 * cos(M_PI/2 - origin_heading), converted_waypoint.y, 0.005);
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
