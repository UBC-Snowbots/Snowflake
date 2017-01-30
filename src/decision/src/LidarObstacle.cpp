#include <LidarDecision.h>

/*
 * Created By: Gareth Ellis
 * Created On: January 22, 2017
 * Description: This class represents a Obstacle, as detected by the lidar
 */

LidarObstacle::LidarObstacle(){};

LidarObstacle::LidarObstacle(angle_t angle, distance_t distance) {
    readings.emplace_back(std::pair<angle_t, distance_t>(angle, distance));
}

LidarObstacle::LidarObstacle(std::vector<reading> readings) {
    this->mergeInReadings(readings);
}

distance_t LidarObstacle::getAvgDistance() {
    float total_distance = std::accumulate(readings.begin(), readings.end(), 0,
                                           [] (int accumulator, auto reading){
                                                return accumulator + reading.second;
                                            });
    return total_distance / readings.size();
}

angle_t LidarObstacle::getAvgAngle() {
    float total_angle = std::accumulate(readings.begin(), readings.end(), 0,
                                        [] (int accumulator, auto reading){
                                                return accumulator + reading.first;
                                            });
    return total_angle / readings.size();
}

angle_t LidarObstacle::getMinAngle() {
    // Readings are sorted, so min angle is just the first reading
    return readings[0].first;
}

angle_t LidarObstacle::getMaxAngle() {
    // Readings are sorted, so max angle is just the last reading
    return readings[readings.size()-1].first;
}

float LidarObstacle::dangerScore() {
    return sin(getAvgAngle()) * (1 / getAvgDistance());
}

const std::vector<pair<angle_t, distance_t>>& LidarObstacle::getAllLaserReadings() {
    return readings;
};

void LidarObstacle::mergeInReadings(std::vector<reading>& readings) {
    this->readings.insert(readings.end(), readings.begin(), readings.end());
    // Ensure that the readings are still sorted
    std::sort(readings.begin(), readings.end(), [&] (auto const& reading1, auto const& reading2){
        return reading1.first < reading2.first;
    });
}

void LidarObstacle::mergeInLidarObstacle(LidarObstacle obstacle) {
    // Get readings from obstacle being merged in
    std::vector<pair<angle_t, distance_t>> new_readings = obstacle.getAllLaserReadings();
    this->mergeInReadings(new_readings);
}
