/*
 * Created By: Gareth Ellis
 * Created On: January 26, 2016
 * Description: The Lidar decision node, takes in a raw Lidar scan
 *              and broadcasts a recommended Twist message
 */

#ifndef DECISION_LIDAR_DECISION_H
#define DECISION_LIDAR_DECISION_H

#include <ros/ros.h>
#include <sb_utils.h>
#include <iostream>
#include <cmath>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>

using distance_t = float;
using angle_t = float;

using namespace std;

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
     * @ return the minimum angle of the obstacle from the robot
     */
    float getMinAngle();

    /**
     * Gets the maximum angle from of an object from the robot
     *
     * @ return the maximum angle of the obstacle from the robot
     */
    float getMaxAngle();

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


class LidarDecision {
public:
    LidarDecision(int argc, char **argv, std::string node_name);

    /**
     * Creates a twist command from a given lidar scan that will best avoid obstacles in the scan
     *
     * @return A twist command from a given lidar scan that will best avoid obstacles in the scan
     */
    geometry_msgs::Twist generate_twist_message(const sensor_msgs::LaserScan::ConstPtr &raw_scan);

    /**
     * Finds all obstacles in a given scan
     *
     * @param sensor_msgs\LaserScan a laser scan in which to find obstacles
     * @param max_obstacle_angle_diff the maximum possible angular difference between two lidar hits
     *          for them to be considered the same obstacle
     *
     * @return a vector of all obstacles
     */
    static std::vector<LidarObstacle> findObstacles(const sensor_msgs::LaserScan& scan,
                                                        float max_obstacle_angle_diff);

    /**
     * Finds the obstacle most dangerous to the robot
     *
     * @param obstacles all obstacles to be considered
     *
     * @return the most dangerous obstacle out of those given
     */
    static LidarObstacle mostDangerousObstacle(const std::vector<LidarObstacle> obstacles);

    /**
     * Merges obstacles that are separated by less then max_angle_diff together
     *
     * The merge occurs in place
     *
     * @param obstacles the list of obstacles to be merged (if similar)
     * @param max_angle_diff the max difference that two obstacles may differ by and still be considered
     *                      part of the same obstacle
     */
    static void mergeSimilarObstacles(std::vector<LidarObstacle>& obstacles, float max_angle_diff);

    /**
     * Creates a twist message from a given obstacle
     *
     * - If obstacle is not within danger distance, ignore
     * - If obstacle is to left, turn right
     * - If obstacle is to right, turn left
     * - If obstacle is exactly in front, turn left (arbitrary decision)
     *
     * @param max_obstacle_danger_distance distance which the obstacle must be within to be considered
     * @param obstacle the obstacle to be considered
     *
     * @return a twist message
     */
    static geometry_msgs::Twist twist_message_from_obstacle(LidarObstacle obstacle,
                                                            distance_t danger_distance,
                                                            angle_t danger_angle,
                                                            float linear_vel_multiplier,
                                                            float angular_vel_multiplier);

private:
    void scanCallBack(const sensor_msgs::LaserScan::ConstPtr& raw_scan);

    // The maximum angle which two scans can be different by to be considered the same obstacle
    float max_obstacle_angle_diff;
    // The distance at which an obstacle can be and be considered a danger
    float max_obstacle_danger_distance;
    // The angle, measured from 0 being directly in front of the robot, at which an obstacle
    // is considered a danger to the robot
    float obstacle_danger_angle;
    // The multiplier for angular velocity that that robot uses to create twist messages
    float twist_turn_rate;
    // The multiplier for linear velocity that that robot uses to create twist messages
    float twist_velocity;
    ros::Subscriber scan_subscriber;
    ros::Publisher twist_publisher;
};


#endif //DECISION_LIDAR_DECISION_H

