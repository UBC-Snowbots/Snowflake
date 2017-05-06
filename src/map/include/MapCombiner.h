//
// Created by valerian on 06/05/17.
// Description: Combines two GridMaps together
//

#ifndef PROJECT_MAPCOMBINER_H
#define PROJECT_MAPCOMBINER_H

#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_msgs/GridMap.h>
#include <ros/ros.h>

class MapCombiner {

public:
    MapCombiner();
    MapCombiner(int argc, char** argv, std::string node_name);

private:
    ros::Publisher pub;
    ros::Subscriber sub_one;
    ros::Subscriber sub_two;

    grid_map::GridMap combineMaps(const grid_map::GridMap& map_one, const grid_map::GridMap& map_two);
    void mapCallback(const grid_map_msgs::GridMap::ConstPtr& grid_map);

    grid_map_msgs::GridMap::ConstPtr map_one_pointer;
    grid_map_msgs::GridMap::ConstPtr map_two_pointer;
};



#endif //PROJECT_MAPCOMBINER_H
