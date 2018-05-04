/*
 * Created By: Gareth Ellis
 * Created On: April 3, 2018
 * Description: The node wrapper for the `ObstacleManager` that provides us
 *              with a discrete map of our environment and can generate
 *              Occupancy grids for navigation
 */

#include <ObstacleManagerNode.h>

ObstacleManagerNode::ObstacleManagerNode(int argc, char **argv, std::string node_name) :
    // Since there is no default constructor for `ObstacleManager`, assign it some random
    // values for now (it will be overwritten later in this constructor)
    obstacle_manager(1,1,1,1)
{
    // Setup NodeHandles
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    // Get Params
    double cone_merging_tolerance, line_merging_tolerance, obstacle_inflation_buffer, occ_grid_cell_size;
    SB_getParam(private_nh, "cone_merging_tolerance", cone_merging_tolerance, 0.3);
    SB_getParam(private_nh, "line_merging_tolerance", line_merging_tolerance, 0.3);
    SB_getParam(private_nh, "obstacle_inflation_buffer", obstacle_inflation_buffer, 1.0);
    SB_getParam(private_nh, "occ_grid_cell_size", occ_grid_cell_size, 1.0);

    // Setup the Obstacle Manager
    obstacle_manager = ObstacleManager(cone_merging_tolerance, line_merging_tolerance, obstacle_inflation_buffer, occ_grid_cell_size);

    // Setup Subscriber(s)
    cone_obstacle_subscriber = nh.subscribe<mapping_igvc::ConeObstacle>("cone_obstacles", 10, &ObstacleManagerNode::coneObstacleCallback, this);
    line_obstacle_subscriber = nh.subscribe<mapping_igvc::LineObstacle>("line_obstacles", 10, &ObstacleManagerNode::lineObstacleCallback, this);

    // Setup Publisher(s)
    std::string topic = private_nh.resolveName("occ_grid");
    occ_grid_publisher = private_nh.advertise<nav_msgs::OccupancyGrid>(topic, 10);
}

void ObstacleManagerNode::coneObstacleCallback(const mapping_igvc::ConeObstacle::ConstPtr &cone_msg) {
    obstacle_manager.addObstacle(*cone_msg);
}

void ObstacleManagerNode::lineObstacleCallback(const mapping_igvc::LineObstacle::ConstPtr &line_msg) {
    // Convert the polynomial to a spline
    sb_geom::PolynomialSegment poly_segment(line_msg->coefficients, line_msg->x_min, line_msg->x_max);
    sb_geom::Spline spline(poly_segment);

    obstacle_manager.addObstacle(spline);
}

