//
// Created by min on 23/05/18.
//

#include <PathFinderNode.h>

PathFinderNode::PathFinderNode(int argc,
                                     char** argv,
                                     std::string node_name) {
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    SB_getParam(private_nh, std::string("map_frame_name"), this->_map_frame_name, std::string("/map"));
    SB_getParam(private_nh, std::string("robot_frame_name"), this->_robot_frame_name, std::string("/base_link"));

    std::string topic_to_subscribe_to = "occupancy_grid"; // dummy topic name
    int refresh_rate                  = 10;
    this->subscriber                        = nh.subscribe(
    topic_to_subscribe_to, refresh_rate, &PathFinderNode::OccupancyGridCallback, this);

    std::string topic_to_publish_to =
    "path"; // dummy topic name
    uint32_t queue_size = 1;
    this->publisher           = private_nh.advertise<nav_msgs::Path>(
    topic_to_publish_to, queue_size);
}

void PathFinderNode::OccupancyGridCallback(const nav_msgs::OccupancyGrid grid) {
    geometry_msgs::Point start = getStartPoint();
    nav_msgs::Path path = PathFinder::calculatePath(start, this->_goal, grid);
    this->publisher.publish(path);
}

geometry_msgs::Point PathFinderNode::getStartPoint() {
    tf::StampedTransform transform;
    try{
        this->_listener.lookupTransform(this->_robot_frame_name, this->_map_frame_name,
                                 ros::Time(0), transform);
    }
    catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
    }

    geometry_msgs::Point start;
    start.x = transform.getOrigin().x();
    start.y = transform.getOrigin().y();

    return start;
}
