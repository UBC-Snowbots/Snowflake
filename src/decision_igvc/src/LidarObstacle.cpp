/*
 * Created By: Gareth Ellis
 * Created On: January 22, 2017
 * Description: This class represents an Obstacle, as detected by the lidar,
 *              and provides various utilities for working with obstacles
 */

#include <LidarObstacle.h>


LidarObstacle::LidarObstacle(){};

LidarObstacle::LidarObstacle(angle_t angle, distance_t distance) {
    readings.emplace_back(Reading{angle, distance});
}

LidarObstacle::LidarObstacle(std::vector<Reading> readings) {
    this->mergeInReadings(readings);
}

distance_t LidarObstacle::getAvgDistance() {
    float total_distance = std::accumulate(readings.begin(), readings.end(), 0,
                                           [] (int accumulator, auto reading){
                                                return accumulator + reading.range;
                                            });
    return total_distance / readings.size();
}

angle_t LidarObstacle::getAvgAngle() {
    float total_angle = std::accumulate(readings.begin(), readings.end(), 0.0,
                                        [] (float accumulator, auto reading){
                                                return accumulator + reading.angle;
                                            });
    return total_angle / readings.size();
}

angle_t LidarObstacle::getMinAngle() {
    // Readings are sorted, so min angle is just the first Reading
    return readings[0].angle;
}

angle_t LidarObstacle::getMaxAngle() {
    // Readings are sorted, so max angle is just the last Reading
    return readings[readings.size()-1].angle;
}

distance_t LidarObstacle::getLastDistance(){
    return readings[readings.size()-1].range;
}

distance_t LidarObstacle::getFirstDistance(){
    return readings[0].range;
}

const std::vector<Reading>& LidarObstacle::getAllLaserReadings() {
    return readings;
};

distance_t LidarObstacle::getMinDistance() {
    std::vector<Reading> readings = getAllLaserReadings();
    Reading reading_with_min_distance = *std::min_element(readings.begin(), readings.end(),
                                                          [&] (auto const& reading1, auto const& reading2){
                                                                   return reading1.range < reading2.range;
                                                          });
    return reading_with_min_distance.range;
}

distance_t LidarObstacle::getMaxDistance() {
    std::vector<Reading> readings = getAllLaserReadings();
    Reading reading_with_max_distance = *std::max_element(readings.begin(), readings.end(),
                                                          [&] (auto const& reading1, auto const& reading2){
                                                                   return reading1.range > reading2.range;
                                                          });
    return reading_with_max_distance.range;
}

float LidarObstacle::dangerScore() {
    // angle score increases as an obstacle's angle relative to the robot increases
    float angle_score = cos(getAvgAngle());
    // distance score increases as an obstacle's distance relative to the robot increases
    float distance_score = (1 / getMinDistance());
    // danger score is sum of angle score and distance score
    return (angle_score + distance_score);
}

void LidarObstacle::mergeInReadings(std::vector<Reading> &new_readings) {
    this->readings.insert(readings.end(), new_readings.begin(), new_readings.end());
    // Ensure that the readings are still sorted
    std::sort(readings.begin(), readings.end(), [&] (auto const& reading1, auto const& reading2){
        return reading1.angle < reading2.angle;
    });
}

void LidarObstacle::mergeInLidarObstacle(LidarObstacle obstacle) {
    // Get readings from obstacle being merged in
    std::vector<Reading> new_readings = obstacle.getAllLaserReadings();
    this->mergeInReadings(new_readings);
}
