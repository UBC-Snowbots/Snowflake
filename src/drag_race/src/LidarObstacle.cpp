/*
 * Created By: Gareth Ellis
 * Created On: January 22, 2017
 * Description: This class represents an Obstacle, as detected by the lidar,
 *              and provides various utilities for working with obstacles
 */

#include <LidarObstacle.h>

// TODO: min wall length, and determination of whether to merge obstacles and
// what their type is,
// TODO: all really belongs in LidarObstacleManager
LidarObstacle::LidarObstacle() : LidarObstacle(std::vector<Reading>()) {}

LidarObstacle::LidarObstacle(double min_wall_length)
  : LidarObstacle(min_wall_length, std::vector<Reading>()) {}

LidarObstacle::LidarObstacle(angle_t angle, distance_t distance)
  : LidarObstacle({Reading{angle, distance}}) {}

LidarObstacle::LidarObstacle(double min_wall_length,
                             angle_t angle,
                             distance_t distance)
  : LidarObstacle(min_wall_length, {Reading{angle, distance}}) {}

LidarObstacle::LidarObstacle(std::vector<Reading> readings)
  : LidarObstacle(1, readings) {}

LidarObstacle::LidarObstacle(double min_wall_length,
                             std::vector<Reading> readings)
  : min_wall_length(min_wall_length), obstacle_type(NONE) {
    this->mergeInReadings(readings);
}

distance_t LidarObstacle::getAvgDistance() {
    double total_distance = std::accumulate(
    readings.begin(), readings.end(), 0, [](int accumulator, auto reading) {
        return accumulator + reading.range;
    });
    return total_distance / readings.size();
}

angle_t LidarObstacle::getAvgAngle() {
    double total_angle = std::accumulate(readings.begin(),
                                         readings.end(),
                                         0.0,
                                         [](double accumulator, auto reading) {
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
    return readings[readings.size() - 1].angle;
}

distance_t LidarObstacle::getLastDistance() {
    return readings[readings.size() - 1].range;
}

distance_t LidarObstacle::getFirstDistance() {
    return readings[0].range;
}

const std::vector<Reading>& LidarObstacle::getAllLaserReadings() {
    return readings;
};

distance_t LidarObstacle::getMinDistance() {
    std::vector<Reading> readings = getAllLaserReadings();
    Reading reading_with_min_distance =
    *std::min_element(readings.begin(),
                      readings.end(),
                      [&](auto const& reading1, auto const& reading2) {
                          return reading1.range < reading2.range;
                      });
    return reading_with_min_distance.range;
}

distance_t LidarObstacle::getMaxDistance() {
    std::vector<Reading> readings = getAllLaserReadings();
    Reading reading_with_max_distance =
    *std::max_element(readings.begin(),
                      readings.end(),
                      [&](auto const& reading1, auto const& reading2) {
                          return reading1.range > reading2.range;
                      });
    return reading_with_max_distance.range;
}

void LidarObstacle::mergeInLidarObstacle(LidarObstacle& obstacle) {
    // Get readings from obstacle being merged in
    std::vector<Reading> new_readings = obstacle.getAllLaserReadings();
    this->mergeInReadings(new_readings);
}

void LidarObstacle::mergeInReadings(std::vector<Reading>& new_readings) {
    this->readings.insert(
    readings.end(), new_readings.begin(), new_readings.end());
    // Ensure that the readings are still sorted
    std::sort(readings.begin(),
              readings.end(),
              [&](auto const& reading1, auto const& reading2) {
                  return reading1.angle < reading2.angle;
              });
    // Ensure the obstacle type is still correct
    determineObstacleType();
    // Update the center of the obstacle
    updateCenter();
}

void LidarObstacle::updateCenter() {
    // Create points for all readings
    std::vector<Point> points = getReadingsAsPoints();

    // Average Points to get new center
    Point averaged_point{0, 0};
    for (Point p : points) {
        averaged_point.x += p.x;
        averaged_point.y += p.y;
    }
    averaged_point.x /= points.size();
    averaged_point.y /= points.size();
    center = averaged_point;
}

void LidarObstacle::determineObstacleType() {
    // TODO: Setup some sort of min number of readings to be considered a cone
    // If this obstacle has no readings, then it's NONE
    if (readings.size() == 0) obstacle_type = NONE;
    // If the obstacle is long enough, then it's a WALL
    else if (getLength() > min_wall_length)
        obstacle_type = WALL;
    else // If it's not NONE or a WALL, then it's a CONE
        obstacle_type = CONE;
}

// TODO: Is there a more appropriate place for these functions?

double LidarObstacle::getLength() {
    // Using Law of Cosines (c^2 = a^2 + b^2 + 2ab*Cos(C)) to get the length
    // from the leftmost to the rightmost point
    double left_length  = readings[0].range;
    double right_length = readings.back().range;
    double theta        = std::abs(readings[0].angle - readings.back().angle);
    return std::sqrt(std::pow(left_length, 2) + std::pow(right_length, 2) -
                     2 * left_length * right_length * std::cos(theta));
}

double LidarObstacle::getMinWallLength() {
    return min_wall_length;
}

ObstacleType LidarObstacle::getObstacleType() {
    return obstacle_type;
}

Point LidarObstacle::getCenter() {
    return center;
}

std::vector<Point> LidarObstacle::getReadingsAsPoints() {
    std::vector<Point> points;
    for (Reading reading : readings) {
        Point p = pointFromReading(reading);
        points.emplace_back(p);
    }
    return points;
}

Point LidarObstacle::pointFromReading(const Reading& reading) {
    double x = reading.range * std::cos(reading.angle);
    double y = reading.range * std::sin(reading.angle);
    return Point{x, y};
}

double distanceBetweenPoints(const Point& p1, const Point& p2) {
    return std::sqrt(std::pow(p1.x - p2.x, 2.0) + std::pow(p1.y - p2.y, 2.0));
}
bool operator==(const Point& p1, const Point& p2) {
    return (p1.x == p2.x && p1.y == p2.y);
}
