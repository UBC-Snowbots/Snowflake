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

// Snowbots Includes
#include <sb_utils.h>
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
     * Publishes the generated occupancy grid
     *
     * @param timer_event this param is required to let us call this function on a timer, but we ignore it
     */
    void publishGeneratedOccupancyGrid(const ros::TimerEvent& timer_event);

    // The principle class this node wraps, manages all obstacles and lets
    // us generate a map containing all known obstacles
    ObstacleManager obstacle_manager;

    // Subscribers for obstacles
    ros::Subscriber cone_obstacle_subscriber;
    ros::Subscriber line_obstacle_subscriber;

    // Publisher for the generated occupancy grid
    ros::Publisher occ_grid_publisher;

    // Timer for publishing the generated occupancy grid
    ros::Timer occ_grid_generation_timer;

    // The frame the generated occupancy grid will be in
    std::string occ_grid_frame;

    // The current sequence id for the published Occupancy Grid
    // (we increment this each time we publish a new grid)
    int occ_grid_seq;

};
#endif //MAPPING_IGVC_OBSTACLE_MANAGER_NODE_H
