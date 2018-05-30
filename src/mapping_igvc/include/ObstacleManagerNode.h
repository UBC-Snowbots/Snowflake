/*
 * Created By: Gareth Ellis
 * Created On: April 3, 2018
 * Description: The node wrapper for the `ObstacleManager` that provides us
 *              with a discrete map of our environment and can generate
 *              Occupancy grids for navigation
 */

#ifndef MAPPING_IGVC_OBSTACLE_MANAGER_NODE_H
#define MAPPING_IGVC_OBSTACLE_MANAGER_NODE_H

// ROS Includes
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>

// Snowbots Includes
#include <sb_utils.h>
#include <RvizUtils.h>
#include "ObstacleManager.h"
#include "mapping_igvc/ConeObstacle.h"
#include "mapping_igvc/LineObstacle.h"

class ObstacleManagerNode {
public:
    ObstacleManagerNode(int argc, char **argv, std::string node_name);

private:
    /**
     * Callback function for cone obstacles
     *
     * @param cone_msg the cone obstacle
     */
    void coneObstacleCallback(const mapping_igvc::ConeObstacle::ConstPtr& cone_msg);

    /**
     * Callback function for line obstacles
     *
     * @param line_msg the line obstacle
     */
    void lineObstacleCallback(const mapping_igvc::LineObstacle::ConstPtr& line_msg);

    /**
     * Publishes all obstacles as markers that can be visualised in RViz
     *
     * @param timer_event this param is required to let us call this function on a timer, but we ignore it
     */
    void publishObstacleMarkers(const ros::TimerEvent& timer_event);

    /**
     * Publishes the generated occupancy grid
     *
     * @param timer_event this param is required to let us call this function on a timer, but we ignore it
     */
    void publishGeneratedOccupancyGrid(const ros::TimerEvent& timer_event);

    // TODO: Should this function be in the `sb_geom` package?
    // TODO: Test me
    /**
     * Transforms a given Spline from a one given frame to another given frame
     *
     * @param from_frame the frame the Spline is to be transformed FROM
     * @param to_frame  the frame the Spline is to be transformed TO
     * @param transform_time the time at which to lookup the transform
     * @return `spline` transformed from `from_frame` to `to_frame` at time `transform_time`
     */
    sb_geom::Spline transformSpline(sb_geom::Spline spline, std::string from_frame, std::string to_frame, ros::Time transform_time);

    // The principle class this node wraps, manages all obstacles and lets
    // us generate a map containing all known obstacles
    ObstacleManager obstacle_manager;

    // Subscribers for obstacles
    ros::Subscriber cone_obstacle_subscriber;
    ros::Subscriber line_obstacle_subscriber;

    // Publisher for the generated occupancy grid
    ros::Publisher occ_grid_publisher;

    // Publisher for Line Obstacle Markers
    ros::Publisher line_marker_publisher;

    // Publisher for Cone Obstacle Markers
    ros::Publisher cone_marker_publisher;

    // The listener that gets our transforms
    tf::TransformListener* tf_listener;

    // Timer for publishing the generated occupancy grid
    ros::Timer occ_grid_generation_timer;

    // Timer for publishing debug RViz Markers
    ros::Timer debug_marker_generation_timer;

    // TODO: Better name?
    // The radius around us in which keep obstacles
    // (obstacles outside this radius are removed)
    double obstacle_pruning_radius;

    // The frame the generated occupancy grid will be in
    std::string occ_grid_frame;

    // The base frame of the robot
    std::string robot_base_frame;

    // The current sequence id for the published Occupancy Grid
    // (we increment this each time we publish a new grid)
    unsigned int occ_grid_seq;

    // How long to wait for obstacle transformations
    // (in cases where the obstacle is published before the transform)
    ros::Duration obstacle_tf_wait;

    // The number of points per meter to sample from each line when
    // generating debug markers for the lines
    int line_marker_resolution; 

};
#endif //MAPPING_IGVC_OBSTACLE_MANAGER_NODE_H
