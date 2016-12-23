/*
 * Created By: Valerian Ratu
 * Created On: December 23, 2016
 * Description: A node which manages and organizes mapping information from various sources
 */

#ifndef PROJECT_MAP_H
#define PROJECT_MAP_H

#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_core/GridMap.hpp>
#include <grid_map_msgs/GridMap.h>

class Map {
public:
    Map(int argc, char **argv, std::string node_name);
    grid_map::Matrix getVisionLayer();
    grid_map::Matrix getLidarLayer();
    grid_map::Matrix getFootprintLayer();
    std::pair<int, int> getCurrentLocation();


private:

    grid_map::GridMap map;
    grid_map::Position pos;
};


#endif //PROJECT_MAP_H
