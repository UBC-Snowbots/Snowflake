/*
 * Created By: Min Gyo Kim
 * Created On: May 23rd 2018
 * Description: Implementation of Path Finder Node
 *              Once the node has received both occupancy grid and the goal
 * point,
 *              it retrieves the position of the robot through the tf tree, and
 *              calls PathFinder's static methods to calculate the path.
 *              Then it publishes the path.
 */

#include <PathFinderNode.h>

PathFinderNode::PathFinderNode(int argc, char** argv, std::string node_name) {
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    SB_getParam(private_nh,
                std::string("global_frame_name"),
                this->_global_frame_name,
                std::string("/map"));
    SB_getParam(private_nh,
                std::string("base_frame_name"),
                this->_base_frame_name,
                std::string("/base_link"));

    std::string grid_subscriber_topic = "/occupancy_grid";
    int refresh_rate                  = 10;
    this->grid_subscriber             = nh.subscribe(grid_subscriber_topic,
                                         refresh_rate,
                                         &PathFinderNode::occupancyGridCallback,
                                         this);

    std::string goal_subscriber_topic = "/goal";
    this->goal_subscriber             = nh.subscribe(
    goal_subscriber_topic, refresh_rate, &PathFinderNode::goalCallback, this);

    std::string topic_to_publish_to = "path";
    uint32_t queue_size             = 1;
    this->publisher =
    private_nh.advertise<nav_msgs::Path>(topic_to_publish_to, queue_size);

    this->_listener = new tf::TransformListener();
}

void PathFinderNode::occupancyGridCallback(const nav_msgs::OccupancyGrid grid) {
    this->_grid           = grid;
    this->_receivied_grid = true;
    if (this->_received_goal) { publishPath(); }
}

void PathFinderNode::goalCallback(const geometry_msgs::Point goal) {
    this->_goal          = goal;
    this->_received_goal = true;
    if (this->_receivied_grid) { publishPath(); }
}

void PathFinderNode::publishPath() {
    tf::StampedTransform transform;

    try {
        this->_listener->lookupTransform(this->_global_frame_name,
                                         this->_base_frame_name,
                                         ros::Time::now(),
                                         transform);
    } catch (tf::TransformException ex) {
        // If we can't lookup the tf, then don't publish path
        ROS_WARN_STREAM(
        "Could not lookup tf between " << this->_global_frame_name << " and "
                                       << this->_base_frame_name);
        ROS_WARN_STREAM(ex.what());
        return;
    }

    geometry_msgs::Point start;
    start.x = transform.getOrigin().x();
    start.y = transform.getOrigin().y();

    nav_msgs::Path path =
    PathFinder::calculatePath(start, this->_goal, this->_grid);

    this->publisher.publish(path);
}
