/*
 * Created By: Gareth Ellis
 * Created On: January 22, 2017
 * Description: This class represents an Obstacle, as detected by the lidar,
 *              and provides various utilities for working with obstacles
 */

#ifndef LIDAROBSTACLE_H
#define LIDAROBSTACLE_H

// STD
#include <vector>
#include <algorithm>


using distance_t = float;
using angle_t = float;

struct Reading {
    angle_t angle;
    distance_t range;
};

class LidarObstacle {
public:
    /**
     * Creates a LidarObstacle with no readings
     */
    LidarObstacle();

    /**
     * Creates a LidarObstacle with a given distance and angle
     *
     * @param distance The distance to the obstacle
     * @param angle The angle to the obstacle
     */
    LidarObstacle(angle_t angle, distance_t distance);

    /**
     * Creates a LidarObstacle from a given set of readings (pair<angle, distance>)
     *
     * @param readings a vector of pairs of the form pair<angle, distance>
     */
    LidarObstacle(std::vector<Reading> readings);

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
    float getAvgDistance();

    /**
     * Gets the angle of the Obstacle from the robot
     *
     * The angle from the robot is the average of all scan angles
     *
     * @return the angle of the Obstacle from the robot
     */
    float getAvgAngle();

    /**
     * Gets the minimum angle from of an object from the robot
     *
     * @return the minimum angle of the obstacle from the robot
     */
    float getMinAngle();

    /**
     * Gets the maximum angle from of an object from the robot
     *
     * @return the maximum angle of the obstacle from the robot
     */
    float getMaxAngle();

    /**
     * Gets the minimum distance from of an object from the robot
     *
     * @return the minimum distance of the obstacle from the robot
     */
    float getMinDistance();

    /**
     * Gets the maximum distance from of an object from the robot
     *
     * @return the maximum distance of the obstacle from the robot
     */
    float getMaxDistance();

    /**
     * Calculates a danger score for the obstacle
     *
     * ie. how dangerous the obstacle is to the robot
     *
     * @return danger_score how dangerous the obstacle is to the robot
     */
    float dangerScore();

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
    void mergeInLidarObstacle(LidarObstacle obstacle);

private:
    /**
     * Merges a given set of readings into the obstacle
     *
     * @param readings the readings to be merged in
     */
    void mergeInReadings(std::vector<Reading> &new_readings);

    // The distances and angles of all the laser scan hits that comprise the object.
    // pairs are stored in sorted order, from min to max angle.
    std::vector<Reading> readings;
};

#endif //LIDAROBSTACLE_H
