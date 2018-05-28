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
    double cone_merging_tolerance, line_merging_tolerance, obstacle_inflation_buffer, occ_grid_cell_size, occ_grid_generation_rate, debug_marker_generation_rate, obstacle_tf_wait_seconds;
    int line_merging_max_iters, closest_line_max_iters;
    SB_getParam(private_nh, "cone_merging_tolerance", cone_merging_tolerance, 0.3);
    SB_getParam(private_nh, "line_merging_tolerance", line_merging_tolerance, 0.3);
    SB_getParam(private_nh, "obstacle_inflation_buffer", obstacle_inflation_buffer, 1.0);
    SB_getParam(private_nh, "occ_grid_cell_size", occ_grid_cell_size, 1.0);
    SB_getParam(private_nh, "line_merging_max_iters", line_merging_max_iters, 10);
    SB_getParam(private_nh, "closest_line_max_iters", closest_line_max_iters, 15);
    SB_getParam(private_nh, "occ_grid_frame", occ_grid_frame, std::string("map"));
    SB_getParam(private_nh, "robot_base_frame", robot_base_frame, std::string("base_link"));
    SB_getParam(private_nh, "occ_grid_generation_rate", occ_grid_generation_rate, 1.0);
    SB_getParam(private_nh, "debug_marker_generation_rate", debug_marker_generation_rate, 1.0);
    SB_getParam(private_nh, "obstacle_tf_wait", obstacle_tf_wait_seconds, 0.3);
    obstacle_tf_wait = ros::Duration(obstacle_tf_wait_seconds);
    SB_getParam(private_nh, "line_marker_resolution", line_marker_resolution, 30);
    SB_getParam(private_nh, "obstacle_pruning_radius", obstacle_pruning_radius, 10.0);


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
    cone_obstacle_subscriber = nh.subscribe<mapping_igvc::ConeObstacle>("cone_obstacles", 5, &ObstacleManagerNode::coneObstacleCallback, this);
    line_obstacle_subscriber = nh.subscribe<mapping_igvc::LineObstacle>("line_obstacles", 5, &ObstacleManagerNode::lineObstacleCallback, this);

    // Setup Publisher(s)
    std::string topic = private_nh.resolveName("occ_grid");
    occ_grid_publisher = private_nh.advertise<nav_msgs::OccupancyGrid>(topic, 1);
    topic = private_nh.resolveName("debug/lines");
    line_marker_publisher = private_nh.advertise<visualization_msgs::MarkerArray>(topic, 1);
    topic = private_nh.resolveName("debug/cones");
    cone_marker_publisher = private_nh.advertise<visualization_msgs::Marker>(topic, 1);

    // Setup Transform Listener
    this->tf_listener = new tf::TransformListener();

    // Setup the timer that will publish the occupancy grid at a set frequency
    // NOTE: This acts similairly to a callback, expect that instead of being
    // triggered when we receive a message, it is triggered after a set period
    // of time
    ros::Duration occ_grid_generation_period(1 / occ_grid_generation_rate);
    occ_grid_generation_timer = private_nh.createTimer(
            occ_grid_generation_period, &ObstacleManagerNode::publishGeneratedOccupancyGrid, this);

    // Setup timer for debug marker publishing
    ros::Duration debug_marker_generation_period(1 / debug_marker_generation_rate);
    debug_marker_generation_timer = private_nh.createTimer(
            occ_grid_generation_period, &ObstacleManagerNode::publishObstacleMarkers, this);
}

void ObstacleManagerNode::coneObstacleCallback(const mapping_igvc::ConeObstacle::ConstPtr &cone_msg) {
    mapping_igvc::ConeObstacle cone = *cone_msg;

    // TODO: Checks for required header bits on msg?

    // Transform the Cone into the map frame
    geometry_msgs::PointStamped cone_center;
    cone_center.point.x = cone.center.x;
    cone_center.point.y = cone.center.y;
    cone_center.point.z = 0;
    cone_center.header.stamp = cone.header.stamp;
    cone_center.header.frame_id = cone.header.frame_id;

    geometry_msgs::PointStamped transformed_cone_center;

    try {
        // Wait a second to see if we get the tf (in case the obstacles are
        // publishing faster then the tf's are getting computed)
        tf_listener->waitForTransform(cone_center.header.frame_id, this->occ_grid_frame, cone_center.header.stamp, this->obstacle_tf_wait);

        // Transform the obstacle
        tf_listener->transformPoint(this->occ_grid_frame, cone_center, transformed_cone_center);
    } catch (tf::TransformException except) {
        ROS_WARN_STREAM("Could not transform cone from \"" << cone.header.frame_id <<  "\" to \"" << this->occ_grid_frame << "\" : " << except.what());
        return;
    }

    cone.center.x = transformed_cone_center.point.x;
    cone.center.y = transformed_cone_center.point.y;

    obstacle_manager.addObstacle(cone);
}

void ObstacleManagerNode::lineObstacleCallback(const mapping_igvc::LineObstacle::ConstPtr &line_msg) {
    // Create a spline from the given polynomial line
    sb_geom::PolynomialSegment poly_segment(line_msg->coefficients, line_msg->x_min, line_msg->x_max);
    sb_geom::Spline spline(poly_segment);

    // TODO: Checks for required header bits on msg?

    // Transform the line into the map frame and add it to the map
    try {
        // Wait a second to see if we get the tf (in case the obstacles are
        // publishing faster then the tf's are getting computed)
        tf_listener->waitForTransform(line_msg->header.frame_id, this->occ_grid_frame, line_msg->header.stamp, this->obstacle_tf_wait);
        
        // Transform the obstacle
        sb_geom::Spline transformed_spline = transformSpline(spline, line_msg->header.frame_id, this->occ_grid_frame, line_msg->header.stamp);

        // Add the obstacle to the map
        obstacle_manager.addObstacle(transformed_spline);
    } catch (tf::TransformException except) {
        ROS_WARN_STREAM("Could not transform line from \"" << line_msg->header.frame_id <<  "\" to \"" << this->occ_grid_frame << "\" : " << except.what());
    }

}

void ObstacleManagerNode::publishGeneratedOccupancyGrid(const ros::TimerEvent &timer_event) {
    // TODO: THis obstacle removal process should be it's own function
    // Get our most recent position
    sb_geom::Point2D current_position;
    try {
        tf::StampedTransform transform;
        tf_listener->lookupTransform(this->occ_grid_frame, this->robot_base_frame, ros::Time(0), transform);
        current_position.x() = transform.getOrigin().x();
        current_position.y() = transform.getOrigin().y();
    } catch (tf::TransformException except) {
        ROS_WARN_STREAM(except.what());
        return;
    }

    // Remove obstacles far away from us
    obstacle_manager.pruneObstaclesOutsideCircle(current_position, obstacle_pruning_radius);

    // Generate the occupancy grid
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

void ObstacleManagerNode::publishObstacleMarkers(const ros::TimerEvent& timer_event) {
    // TODO: Cones and lines should be two different functions
    // Publish Cones
    std::vector<geometry_msgs::Point> cone_points;
    for (mapping_igvc::ConeObstacle cone_obstacle : this->obstacle_manager.getConeObstacles()){
        geometry_msgs::Point cone_point;
        // TODO: Cone scale should reflect actual size of cone
        cone_point.x = cone_obstacle.center.x;
        cone_point.y = cone_obstacle.center.y;
        cone_point.z = 0;
        cone_points.emplace_back(cone_point);
    }

    // TODO: Make scale and color a param
    visualization_msgs::Marker::_color_type color =
            snowbots::RvizUtils::createMarkerColor(1.0, 0.0, 1.0, 1.0);
    visualization_msgs::Marker::_scale_type scale =
            snowbots::RvizUtils::createrMarkerScale(0.4, 0.4, 0.4);

    // TODO: SHould be a CYLINDER MarkerArray instead
    visualization_msgs::Marker cone_marker = snowbots::RvizUtils::createMarker(
                                      cone_points,
                                      color,
                                      scale,
                                      this->occ_grid_frame,
                                      "debug",
                                      visualization_msgs::Marker::POINTS);

    cone_marker_publisher.publish(cone_marker);

    // Publish lines
    std::vector<std::vector<geometry_msgs::Point>> lines_points;
    for (sb_geom::Spline spline : this->obstacle_manager.getLineObstacles()){
        // Decompose the spline into a lot of points to make a marker from
        // TODO: Make number of points in param
        int num_points = spline.approxLength() * line_marker_resolution;
        std::vector<geometry_msgs::Point> line_points;
        for (int i = 0; i < num_points; i++){
            sb_geom::Point2D spline_point = spline(1.0/(double)num_points * (double)i);
            geometry_msgs::Point point;
            point.x = spline_point.x();
            point.y = spline_point.y();
            point.z = 0;
            line_points.emplace_back(point);
        }
        lines_points.emplace_back(line_points);
    }

    // TODO: Make scale and color a param
    color =
            snowbots::RvizUtils::createMarkerColor(1.0, 0.0, 1.0, 1.0);
    scale =
            snowbots::RvizUtils::createrMarkerScale(0.05, 0.05, 0.05);

    visualization_msgs::MarkerArray line_marker_array = snowbots::RvizUtils::createMarkerArray(
                                      lines_points,
                                      color,
                                      scale,
                                      this->occ_grid_frame,
                                      "debug",
                                      visualization_msgs::Marker::LINE_STRIP);

    line_marker_publisher.publish(line_marker_array);
}

