/*
 * Created By: Gareth Ellis
 * Created On: April 3, 2018
 * Description: The node wrapper for the `ObstacleManager` that provides us
 *              with a discrete map of our environment and can generate
 *              Occupancy grids for navigation
 */

#ifndef MAPPING_IGVC_OBSTACLE_MANAGER_NODE_H
#define MAPPING_IGVC_OBSTACLE_MANAGER_NODE_H

#include <ros/ros.h>
#include <sb_utils.h>

class ObstacleManagerNode {
public:
    ObstacleManagerNode(int argc, char **argv, std::string node_name);

private:

};
#endif //MAPPING_IGVC_OBSTACLE_MANAGER_NODE_H
