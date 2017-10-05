/*
 * Created By: Gareth Ellis
 * Created On: July ??, 2017
 * Description: A node that runs a drag race outlined with cones,
 *              starting when the green light is detected (a boolean topic)
 *              stopping before the wall at the end of the drag strip
 */

#ifndef DRAG_RACE_NODE_DRAG_RACE_H
#define DRAG_RACE_NODE_DRAG_RACE_H

// STD Includes
#include <iostream>
#include <vector>

// ROS Includes
// TODO: Sort me for neatness
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>

// SB Includes
#include <DragRaceController.h>
#include <LidarObstacleManager.h>
#include <sb_utils.h>

class DragRaceNode {
  public:
    DragRaceNode(int argc, char** argv, std::string node_name);

  private:
    // TODO: Doc comment
    void scanCallBack(const sensor_msgs::LaserScan::ConstPtr& scan);

    void greenLightCallBack(const std_msgs::Bool& green_light_detected);

    // Manages obstacles, including the cones and wall
    LidarObstacleManager obstacle_manager;

    // Manages line handling and movement
    DragRaceController drag_race_controller;

    // How far lines can be before being considered invalid
    double max_distance_from_robot_accepted;

    // How far from the target line the robot should be
    double target_distance;

    // Where the target line is
    bool line_to_the_right;

    // Velocity limits
    double angular_vel_cap;
    double linear_vel_cap;

    // Scaling
    double theta_scaling_multiplier;
    double angular_speed_multiplier;
    double linear_speed_multiplier;

    // Traffic light detection
    int minimum_green_recognised_count;
    int green_count_recognised;

    // End zone collision detection
    // TODO: Could make all these local variables?
    int incoming_obstacle_ticks;
    int obstacle_ticks_threshold;
    double collision_distance;
    double front_angle;
    double side_angle_max;
    double side_angle_min;
    double region_fill_percentage;
    bool front_collision_only;

    // Signals that we're at the end of the course
    bool end_of_course;

    // Subscribes to the LaserScan
    ros::Subscriber scan_subscriber;
    // Subscribes to traffic light detection
    ros::Subscriber traffic_light_subscriber;

    // Publishes Twist messages to control the robot
    ros::Publisher twist_publisher;
    // Publishes the obstacles so we can see them in RViz
    ros::Publisher cone_debug_publisher;
    // Publishes the cone lines so we can see them in RViz
    ros::Publisher cone_line_debug_publisher;
    // Publishes the cone line we're using to determine the twist message
    // so we can see it in RViz
    ros::Publisher best_line_debug_publisher;
};

#endif // DRAG_RACE_NODE_DRAG_RACE_H
