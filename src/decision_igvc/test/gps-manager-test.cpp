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
std::vector<std::pair<double, double>>
pairsFromWaypointVector(std::vector<Waypoint> waypoints) {
    std::vector<std::pair<double, double>> new_waypoints;
    for (int i = 0; i < waypoints.size(); i++) {
        std::pair<double, double> new_waypoint(waypoints[i].lat,
                                               waypoints[i].lon);
        new_waypoints.push_back(new_waypoint);
    }
    return new_waypoints;
}

TEST(GpsManager, parseWaypoints) {
    std::vector<std::pair<double, double>> one_waypoint_result = {
    std::pair<double, double>(9.87, 10.98)};
    std::vector<double> one_waypoint_input = {9.87, 10.98};

    std::vector<std::pair<double, double>> multi_waypoint_result = {
    std::pair<double, double>(9.87, 10.98),
    std::pair<double, double>(98.123, 97.122),
    std::pair<double, double>(87.12, 12.12)};
    std::vector<double> multi_waypoint_input = {
    9.87, 10.98, 98.123, 97.122, 87.12, 12.12};

    EXPECT_EQ(
    one_waypoint_result,
    pairsFromWaypointVector(GpsManager::parseWaypoints(one_waypoint_input)));
    EXPECT_EQ(
    multi_waypoint_result,
    pairsFromWaypointVector(GpsManager::parseWaypoints(multi_waypoint_input)));
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
