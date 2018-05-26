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
    obstacle_manager(1,1,1,1),
    occ_grid_seq(0)
{
    // Setup NodeHandles
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    // Get Params
    double cone_merging_tolerance, line_merging_tolerance, obstacle_inflation_buffer, occ_grid_cell_size, occ_grid_generation_rate;
    int line_merging_max_iters, closest_line_max_iters;
    SB_getParam(private_nh, "cone_merging_tolerance", cone_merging_tolerance, 0.3);
    SB_getParam(private_nh, "line_merging_tolerance", line_merging_tolerance, 0.3);
    SB_getParam(private_nh, "obstacle_inflation_buffer", obstacle_inflation_buffer, 1.0);
    SB_getParam(private_nh, "occ_grid_cell_size", occ_grid_cell_size, 1.0);
    SB_getParam(private_nh, "line_merging_max_iters", line_merging_max_iters, 10);
    SB_getParam(private_nh, "closest_line_max_iters", closest_line_max_iters, 15);
    SB_getParam(private_nh, "occ_grid_frame", occ_grid_frame, std::string("map"));
    SB_getParam(private_nh, "occ_grid_generation_rate", occ_grid_generation_rate, 1.0);

    // Setup the Obstacle Manager
    obstacle_manager = ObstacleManager(
            cone_merging_tolerance,
            line_merging_tolerance,
            obstacle_inflation_buffer,
            occ_grid_cell_size,
            (unsigned int)line_merging_max_iters,
            (unsigned int)closest_line_max_iters
    );

    // Setup Subscriber(s)
    cone_obstacle_subscriber = nh.subscribe<mapping_igvc::ConeObstacle>("cone_obstacles", 10, &ObstacleManagerNode::coneObstacleCallback, this);
    line_obstacle_subscriber = nh.subscribe<mapping_igvc::LineObstacle>("line_obstacles", 10, &ObstacleManagerNode::lineObstacleCallback, this);

    // Setup Publisher(s)
    std::string topic = private_nh.resolveName("occ_grid");
    occ_grid_publisher = private_nh.advertise<nav_msgs::OccupancyGrid>(topic, 10);

    // Setup Transform Listener
    this->tf_listener = new tf::TransformListener();

    // Setup the timer that will publish the occupancy grid at a set frequency
    // NOTE: This acts similairly to a callback, expect that instead of being
    // triggered when we receive a message, it is triggered after a set period
    // of time
    ros::Duration occ_grid_generation_period(1 / occ_grid_generation_rate);
    
    occ_grid_generation_timer = private_nh.createTimer(
            occ_grid_generation_period, &ObstacleManagerNode::publishGeneratedOccupancyGrid, this);
}

void ObstacleManagerNode::coneObstacleCallback(const mapping_igvc::ConeObstacle::ConstPtr &cone_msg) {
    // TODO (Part 4): Translate to `occ_grid_frame` before adding to obstacle manager
    obstacle_manager.addObstacle(*cone_msg);
}

void ObstacleManagerNode::lineObstacleCallback(const mapping_igvc::LineObstacle::ConstPtr &line_msg) {
    // TODO (Part 4): Translate to `occ_grid_frame` before adding to obstacle manager
    // To Transform:
    // - convert to spline (which will select some interpolation points)
    // - translate the interpolation points
    // - generate a new spline with these interpolation points
    // - add this new spline to the Manager

    // Create a spline from the given polynomial line
    sb_geom::PolynomialSegment poly_segment(line_msg->coefficients, line_msg->x_min, line_msg->x_max);
    sb_geom::Spline spline(poly_segment);

    // Check that we can transform the line into the map frame
    std::string* tf_err_msg;
    if (!tf_listener->canTransform(line_msg->header.frame_id, this->occ_grid_frame, line_msg->header.stamp, tf_err_msg)){
        ROS_WARN_STREAM(
                "Could not transform line from \"" << line_msg->header.frame_id <<  "to \"" << this->occ_grid_frame << ": " <<  *tf_err_msg);
        return;
    }

    sb_geom::Spline transformed_spline = transformSpline(spline, line_msg->header.frame_id, this->occ_grid_frame, line_msg->header.stamp);

    obstacle_manager.addObstacle(transformed_spline);
}

void ObstacleManagerNode::publishGeneratedOccupancyGrid(const ros::TimerEvent &timer_event) {
    nav_msgs::OccupancyGrid occ_grid = obstacle_manager.generateOccupancyGrid();

    occ_grid.header.stamp = ros::Time::now();
    occ_grid.header.frame_id = occ_grid_frame;
    occ_grid.header.seq = occ_grid_seq;
    occ_grid_seq++;

    occ_grid_publisher.publish(occ_grid);
}

sb_geom::Spline ObstacleManagerNode::transformSpline(sb_geom::Spline spline, std::string from_frame, std::string to_frame,
                                                               ros::Time transform_time) {
    // Transform the interpolation points that make up the spline
    std::vector<sb_geom::Point2D> transformed_points;
    for (sb_geom::Point2D& point : spline.getInterpolationPoints()) {
        // Construct a PointStamped from our Point2D
        geometry_msgs::PointStamped stamped_point;
        stamped_point.header.stamp = transform_time;
        stamped_point.header.frame_id = from_frame;
        stamped_point.point.x = point.x();
        stamped_point.point.y = point.y();
        stamped_point.point.z = 0;
        // Transform the PointStamped
        geometry_msgs::PointStamped transformed_stamped_point;
        tf_listener->transformPoint(to_frame, stamped_point, transformed_stamped_point);
        // Convert back to Point2D
        sb_geom::Point2D transformed_point(transformed_stamped_point.point.x, transformed_stamped_point.point.y);

        transformed_points.emplace_back(transformed_point);
    }

    // Construct a new Spline from the transformed points
    sb_geom::Spline transformed_spline(transformed_points);

    return transformed_spline;
}

