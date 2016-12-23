/*
 * Created By: Valerian Ratu
 * Created On: December 23, 2016
 * Description: A node which manages and organizes mapping information from various sources
 */

#include <Map.h>

Map::Map(int argc, char **argv, std::string node_name){

    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;

    map.add("vision");
    map.add("lidar");
    map.add("footprint");

    pos(0,0);
}

grid_map::Matrix Map::getVisionLayer(){
    return map.get("vision");
}

grid_map::Matrix Map::getLidarLayer(){
    return map.get("lidar");
}

grid_map::Matrix Map::getFootprintLayer(){
    return map.get("footprint");
}

std::pair<int, int> Map::getCurrentLocation(){
    return std::pair<int, int>(10,10);
}




