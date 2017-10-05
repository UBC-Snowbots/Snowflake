/*
 * Created By: Gareth Ellis
 * Created On: January 22, 2017
 * Description: This class represents an Obstacle, as detected by the lidar,
 *              and provides various utilities for working with obstacles
 */

#ifndef LIDAROBSTACLE_H
#define LIDAROBSTACLE_H

// STD
#include <algorithm>
#include <vector>

using distance_t = double;
using angle_t    = double;

// Types of Obstacle
enum ObstacleType { NONE, CONE, WALL };

struct Reading {
    angle_t angle;
    distance_t range;
};

// TODO: We should just use geometry_msgs::Point
struct Point {
    double x;
    double y;
};
double distanceBetweenPoints(const Point& p1, const Point& p2);
bool operator==(const Point& p1, const Point& p2);

class LidarObstacle {
  public:
    // TODO: see what functions we can delete here.... probably don't want all
    // them
    /**
     * Creates a LidarObstacle with no readings
     */
    LidarObstacle();

    /**
     * Creates a LidarObstacle with no readings
     *
     * @param min_wall_length the minimum length for the obstacle to be
     * considered a wall
     */
    // TODO: default constructors suck (not really but still...). we should
    // require min_wall_length
    LidarObstacle(double min_wall_length);

    /**
     * Creates a LidarObstacle with a given distance and angle
     *
     * @param distance The distance to the obstacle
     * @param angle The angle to the obstacle
     */
    LidarObstacle(angle_t angle, distance_t distance);

    /**
     * Creates a LidarObstacle with a given distance and angle
     *
     * @param min_wall_length the minimum length for the obstacle to be
     * considered a wall
     * @param distance The distance to the obstacle
     * @param angle The angle to the obstacle
     */
    LidarObstacle(double min_wall_length, angle_t angle, distance_t distance);

    /**
     * Creates a LidarObstacle from a given set of readings
     *
     * @param readings readings to initialize the obstacle with
     */
    LidarObstacle(std::vector<Reading> readings);

    /**
     * Creates a LidarObstacle from a given set of readings
     *
     * @param min_wall_length the minimum length for the obstacle to be
     * considered a wall
     * @param readings readings to initialize the obstacle with
     */
    LidarObstacle(double min_wall_length, std::vector<Reading> readings);

    /**
    * Gets the distance to the rightmost point of the Obstacle
    *
    * @return the distance to the rightmost point of the Obstacle
    */
    distance_t getFirstDistance();

    /**
    * Gets the distance to the leftmost point of the Obstacle
    *
    * @return the distance to the leftmost point of the obstacle
    */

    distance_t getLastDistance();
    /**
     * Gets the average distance of the Obstacle from the robot
     *
     * The distance from the robot is the average of all scan distances
     *
     * @return the distance of the Obstacle from the robot
     */
    double getAvgDistance();

    /**
     * Gets the angle of the Obstacle from the robot
     *
     * The angle from the robot is the average of all scan angles
     *
     * @return the angle of the Obstacle from the robot
     */
    double getAvgAngle();

    /**
     * Gets the minimum angle from of an object from the robot
     *
     * @return the minimum angle of the obstacle from the robot
     */
    double getMinAngle();

    /**
     * Gets the maximum angle from of an object from the robot
     *
     * @return the maximum angle of the obstacle from the robot
     */
    double getMaxAngle();

    /**
     * Gets the minimum distance from of an object from the robot
     *
     * @return the minimum distance of the obstacle from the robot
     */
    double getMinDistance();

    /**
     * Gets the maximum distance from of an object from the robot
     *
     * @return the maximum distance of the obstacle from the robot
     */
    double getMaxDistance();

    /**
     * Gets all laser readings comprising the obstacle
     *
     * @return readings A list of pairs of all laser readings
     */
    const std::vector<Reading>& getAllLaserReadings();

    /**
     * Merges the given LidarObstacle in to this LidarObstacle
     *
     * Adds the given LidarObstacle's scan distances and scan angles to
     * this LidarObstacles scan distances and scan angles respectively
     *
     * @param obstacle The LidarObstacle to be merged in
     */
    void mergeInLidarObstacle(LidarObstacle& obstacle);

    /**
     * Gets the type of the this obstacle
     *
     * @return the type of this obstacle
     */
    ObstacleType getObstacleType();

    /**
     * Determines and sets what the obstacle type should be
     */
    void determineObstacleType();

    /**
     * Returns the length of the obstacle
     *
     * Obstacle length is in this case defined as the distance between
     * the leftmost and rightmost readings
     *
     * @return the length of the obstacle
     */
    // TODO: This is a terrible definition. A reading slightly to the right of
    // the leftmost reading could have infited range
    double getLength();

    /**
     * Get the minimum length for this obstacle to be considered a wall
     *
     * @return the minimum length for this obstacle to be considered a wall
     */
    double getMinWallLength();

    /**
     * Gets the readings composing this obstacle as 2D Points
     *
     * @return readings composing this obstacle as 2D Points
     */
    std::vector<Point> getReadingsAsPoints();

    /**
     * Computes a (x,y) point from a given reading
     *
     * @param reading the reading to compute the point for
     * @return the point interpretation of the given reading
     */
    static Point pointFromReading(const Reading& reading);

    /**
     * Gets the center of the obstacle
     * @return the center of the obstacle
     */
    Point getCenter();

    // TODO: Write functions for modeling obstacles as circles and squares to
    // reduce comparison times
  private:
    /**
     * Merges a given set of readings into the obstacle
     *
     * @param readings the readings to be merged in
     */
    void mergeInReadings(std::vector<Reading>& new_readings);

    /**
     * Updates the center of the obstacle based on our current readings
     */
    void updateCenter();
    // TODO: we should maybe consider moving to storing obstacles as points
    // The distances and angles of all the laser scan hits that comprise the
    // object.
    // readings are stored in sorted order, from min to max angle.
    std::vector<Reading> readings;

    // The type of the obstacle
    ObstacleType obstacle_type;

    // The center of the obstacle
    Point center;

    double min_wall_length;
};

#endif // LIDAROBSTACLE_H
