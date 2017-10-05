/*
 * Created By: Gareth Ellis
 * Created On: July ??, 2017
 * Description: A node that runs a drag race outlined with cones,
 *              starting when the green light is detected (a boolean topic)
 *              stopping before the wall at the end of the drag strip
 */

#include <DragRaceNode.h>

DragRaceNode::DragRaceNode(int argc, char** argv, std::string node_name)
  : green_count_recognised(0) {
    // Setup NodeHandles
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    // Setup Subscriber(s)
    std::string scan_topic = "/scan";
    uint32_t queue_size    = 1;

    scan_subscriber =
    nh.subscribe(scan_topic, queue_size, &DragRaceNode::scanCallBack, this);

    // TODO: make sure this is the right topic name
    std::string traffic_light_topic = "/robot/vision/activity_detected";

    traffic_light_subscriber = nh.subscribe(
    traffic_light_topic, queue_size, &DragRaceNode::greenLightCallBack, this);

    // Setup Publisher(s)
    std::string twist_topic = nh.resolveName("cmd_vel");
    twist_publisher =
    nh.advertise<geometry_msgs::Twist>(twist_topic, queue_size);
    std::string cone_debug_topic = private_nh.resolveName("debug/cone");
    cone_debug_publisher = private_nh.advertise<visualization_msgs::Marker>(
    cone_debug_topic, queue_size);
    std::string cone_lines_debug_topic =
    private_nh.resolveName("debug/cone_lines");
    cone_line_debug_publisher =
    private_nh.advertise<visualization_msgs::Marker>(cone_lines_debug_topic,
                                                     queue_size);
    std::string best_line_debug_topic =
    private_nh.resolveName("debug/best_line");
    best_line_debug_publisher =
    private_nh.advertise<visualization_msgs::Marker>(best_line_debug_topic,
                                                     queue_size);

    // Get Params
    SB_getParam(private_nh, "target_distance", target_distance, 1.0);
    SB_getParam(private_nh, "angular_vel_cap", angular_vel_cap, 1.0);
    SB_getParam(private_nh, "linear_vel_cap", linear_vel_cap, 1.0);
    SB_getParam(
    private_nh, "theta_scaling_multiplier", theta_scaling_multiplier, 1.0);
    SB_getParam(
    private_nh, "angular_speed_multiplier", angular_speed_multiplier, 1.0);
    SB_getParam(
    private_nh, "linear_speed_multiplier", linear_speed_multiplier, 1.0);
    SB_getParam(private_nh, "line_to_the_right", line_to_the_right, true);
    double max_obstacle_merging_distance, cone_grouping_tolerance,
    min_wall_length;
    SB_getParam(private_nh,
                "max_obstacle_merging_distance",
                max_obstacle_merging_distance,
                0.3);
    SB_getParam(
    private_nh, "cone_grouping_tolerance", cone_grouping_tolerance, 1.8);
    SB_getParam(private_nh,
                "max_distance_from_robot_accepted",
                max_distance_from_robot_accepted,
                2.0);
    SB_getParam(private_nh, "min_wall_length", min_wall_length, 0.4);
    SB_getParam(private_nh,
                "minimum_green_count_recognised",
                minimum_green_recognised_count,
                10);
    SB_getParam(
    private_nh, "obstacle_ticks_threshold", obstacle_ticks_threshold, 10);
    SB_getParam(private_nh, "collision_distance", collision_distance, 3.0);

    // NOW IN RADIANS
    SB_getParam(private_nh, "front_angle", front_angle, M_PI_2);
    SB_getParam(
    private_nh, "front_collision_only", front_collision_only, false);
    SB_getParam(private_nh, "side_angle_max", side_angle_max, M_PI_2);
    SB_getParam(
    private_nh, "side_angle_min", side_angle_min, M_PI_2 - M_PI_4 / 2);
    SB_getParam(
    private_nh, "region_fill_percentage", region_fill_percentage, 0.5);

    // Setup drag race controller with given params
    drag_race_controller = DragRaceController(target_distance,
                                              line_to_the_right,
                                              theta_scaling_multiplier,
                                              angular_speed_multiplier,
                                              linear_speed_multiplier,
                                              angular_vel_cap,
                                              linear_vel_cap);

    // Setup the obstacle manager with given params
    obstacle_manager = LidarObstacleManager(max_obstacle_merging_distance,
                                            cone_grouping_tolerance,
                                            max_distance_from_robot_accepted,
                                            min_wall_length,
                                            collision_distance,
                                            front_angle,
                                            side_angle_max,
                                            side_angle_min,
                                            region_fill_percentage,
                                            front_collision_only);

    end_of_course           = false;
    incoming_obstacle_ticks = 0;
}

void DragRaceNode::greenLightCallBack(
const std_msgs::Bool& green_light_detected) {
    if (green_light_detected.data) { green_count_recognised++; }
}

void DragRaceNode::scanCallBack(const sensor_msgs::LaserScan::ConstPtr& scan) {
    // Clear any obstacles we already have
    obstacle_manager.clearObstacles();

    // Insert the scan we just received
    obstacle_manager.addLaserScan(*scan);

    bool no_line_on_expected_side = false;

    // TODO: Option 1
    if (obstacle_manager.collisionDetected()) {
        incoming_obstacle_ticks++;
    } else {
        // False alarm
        incoming_obstacle_ticks = 0;

        // This is maybe better? Experiment
        // incoming_obstacle_ticks--;
        // if (incoming_obstacle_ticks < 0) incoming_obstacle_ticks = 0;
    }

    if (incoming_obstacle_ticks > obstacle_ticks_threshold) {
        if (!end_of_course) {
            end_of_course = true;
            ROS_INFO("END OF COURSE DETECTED");
        }
    }

    // Get the best line for us
    LineOfBestFit best_line = obstacle_manager.getBestLine(line_to_the_right);

    // If no good lines, initiate plan B and use lines on the other side.
    if (best_line.correlation == 0) {
        best_line = obstacle_manager.getBestLine(!line_to_the_right);
        no_line_on_expected_side = true;
    }

    // Avoid the line given while staying within the boundaries
    geometry_msgs::Twist twist = drag_race_controller.determineDesiredMotion(
    best_line, no_line_on_expected_side);

    // If no green light has been detected stop.
    if (green_count_recognised < minimum_green_recognised_count) {
        twist.angular.z = 0;
        twist.linear.x  = 0;
    }

    if (end_of_course) {
        twist.linear.x = twist.linear.y = twist.linear.z = 0;
        twist.angular.x = twist.angular.y = twist.angular.z = 0;
    }

    // Publish our desired twist message
    twist_publisher.publish(twist);

    // TODO: have a debug param for this
    // Broadcast a visualisable representation so we can see obstacles in RViz
    cone_debug_publisher.publish(obstacle_manager.getConeRVizMarker());
    cone_line_debug_publisher.publish(
    obstacle_manager.getConeLinesRVizMarker());
    best_line_debug_publisher.publish(
    obstacle_manager.getBestConeLineRVizMarker(line_to_the_right));
}
