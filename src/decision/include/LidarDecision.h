/*
 * Created By: Gareth Ellis
 * Created On: January 26, 2016
 * Description: The Lidar decision node, takes in a raw Lidar scan
 *              and broadcasts a recommended Twist message
 */

#ifndef DECISION_LIDAR_DECISION_H
#define DECISION_LIDAR_DECISION_H

// STD
#include <iostream>
#include <cmath>

// Snowbots
#include <sb_utils.h>
#include <LidarObstacle.h>

// ROS
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>


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
                                                    float max_obstacle_angle_diff,
                                                    float max_obstacle_distance_diff);

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
     * @param max_distance_diff the max difference that two obstacles may differ by and still be considered
     *                      part of the same obstacle
     */
    static void mergeSimilarObstacles(std::vector<LidarObstacle>& obstacles,
                                      float max_angle_diff,
                                      float max_distance_diff);

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
    // The maximum di which two scans can be different by to be considered the same obstacle
    float max_obstacle_distance_diff;
    // The distance at which an obstacle can be considered a danger
    float max_obstacle_danger_distance;
    // The angle, measured from 0 being directly in front of the robot, at which an obstacle
    // is considered a danger to the robot
    float max_obstacle_danger_angle;
    // The multiplier for linear speed in the twist message produced by this node
    float twist_angular_speed_multiplier;
    // The multiplier for linear speed in the twist message produced by this node
    float twist_linear_speed_multiplier;
    ros::Subscriber scan_subscriber;
    ros::Publisher twist_publisher;
};


#endif //DECISION_LIDAR_DECISION_H

