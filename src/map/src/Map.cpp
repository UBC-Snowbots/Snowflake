/*
 * Created By: Valerian Ratu
 * Created On: December 23, 2016
 * Description: A node which manages and organizes mapping information from various sources
 */

#include <Map.h>

Map::Map(int argc, char **argv, std::string node_name){
    std::vector<std::string> layers("vision", "lidar", "footprint");
    map(layers);
    pos(0,0);
}

grid_map::Matrix Map::getVisionLayer(){
    return NULL;
}

grid_map::Matrix Map::getLidarLayer(){
    return NULL;
}

grid_map::Matrix Map::getFootprintLayer(){
    return NULL;
}

std::pair<int, int> Map::getCurrentLocation(){
    return NULL;
};




