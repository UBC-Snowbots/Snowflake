#include <LidarDecision.h>

/*
 * Created By: Gareth Ellis
 * Created On: January 22, 2017
 * Description: This class represents a Obstacle, as detected by the lidar
 */

LidarObstacle::LidarObstacle(){};

LidarObstacle::LidarObstacle(angle_t angle, distance_t distance) {
    readings.emplace_back(Reading{angle, distance});
}

LidarObstacle::LidarObstacle(std::vector<Reading> readings) {
    this->mergeInReadings(readings);
}

distance_t LidarObstacle::getAvgDistance() {
    float total_distance = std::accumulate(readings.begin(), readings.end(), 0,
                                           [] (float accumulator, auto reading){
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

float LidarObstacle::dangerScore() {
    return sin(getAvgAngle()) * (1 / getAvgDistance());
}

const std::vector<Reading>& LidarObstacle::getAllLaserReadings() {
    return readings;
};

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
