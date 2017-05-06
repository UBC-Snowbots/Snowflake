//
// Created by valerian on 06/05/17.
//

#include "../include/MapCombiner.h"


MapCombiner::MapCombiner(){

}

MapCombiner::MapCombiner(int argc, char** argv, std::string node_name) {
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    std::string map_topic_one = "/mapping/source1";
    std::string map_topic_two = "/mapping/source2";

    int queue_size = 1;
    sub_one = nh.subscribe(map_topic_one, queue_size, &MapCombiner::mapCallback, this);
    sub_two = nh.subscribe(map_topic_two, queue_size, &MapCombiner::mapCallback, this);

    //todo: add store pointers as member variables
    
}