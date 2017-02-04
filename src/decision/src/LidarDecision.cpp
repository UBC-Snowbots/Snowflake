/*
 * Created By: Gareth Ellis
 * Created On: January 26, 2016
 * Description: The Lidar decision node, takes in a raw Lidar scan
 *              and broadcasts a recommended Twist message
 */

#include <LidarDecision.h>


// The constructor
LidarDecision::LidarDecision(int argc, char **argv, std::string node_name) {
    // Setup NodeHandles
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;
    ros::NodeHandle public_nh("~");

    // Setup Subscriber(s)
    std::string laserscan_topic_name = "/robot/scan";
    uint32_t refresh_rate = 10;
    scan_subscriber = public_nh.subscribe(laserscan_topic_name, refresh_rate,
                                          &LidarDecision::scanCallBack, this);

    // Setup Publisher(s)
    std::string twist_topic = public_nh.resolveName("command");
    uint32_t queue_size = 10;
    twist_publisher = nh.advertise<geometry_msgs::Twist>(twist_topic, queue_size);

    // Get Param(s)
    if (!public_nh.param<float>("max_obstacle_angle_diff", max_obstacle_angle_diff, (float)M_PI/30))
        ROS_WARN("max_obstacle_angle_diff not set, defaulting to %f", max_obstacle_angle_diff);
    if (!public_nh.param<distance_t>("max_obstacle_danger_distance", max_obstacle_danger_distance, 10))
        ROS_WARN("max_obstacle_danger_distance not set, defaulting to %f", max_obstacle_angle_diff);
    if (!public_nh.param<float>("twist_turn_rate", twist_turn_rate, 10))
        ROS_WARN("twist_turn_rate not set, defaulting to %f", twist_turn_rate);
    if (!public_nh.param<float>("twist_velocity", twist_velocity, 10))
        ROS_WARN("twist_velocity not set, defaulting to %f", twist_velocity);
}

// This is called whenever a new message is received
void LidarDecision::scanCallBack(const sensor_msgs::LaserScan::ConstPtr& raw_scan) {
    // Deal with new messages here
    twist_publisher.publish(generate_twist_message(raw_scan));
}

geometry_msgs::Twist LidarDecision::generate_twist_message(const sensor_msgs::LaserScan::ConstPtr &raw_scan) {

    // Find all the obstacles
    std::vector<LidarObstacle> obstacles = findObstacles(*raw_scan, max_obstacle_angle_diff);

    // Choose the most dangerous obstacle
    LidarObstacle most_dangerous_obstacle = mostDangerousObstacle(obstacles);

    // Create and return a twist message based on the most dangerous obstacle
    return twist_message_from_obstacle(most_dangerous_obstacle, max_obstacle_danger_distance,
                                       0, twist_velocity, twist_turn_rate);
}

std::vector<LidarObstacle> LidarDecision::findObstacles(const sensor_msgs::LaserScan &scan,
                                                        float max_obstacle_angle_diff) {
    // Get the raw scan data
    std::vector<float> scan_data = scan.ranges;

    // Find all ranges (lidar hits) that could be obstacles (initially each laser hit is an obstacle)
    std::vector<LidarObstacle> obstacles;
    for (int i = 0; i < scan_data.size(); i++) {
        // Check that obstacle is within valid range
        if (scan_data[i] < scan.range_max && scan_data[i] > scan.range_min) {
            // If so, add it to the obstacles
            obstacles.emplace_back(LidarObstacle(scan.angle_increment * i + scan.angle_min,
                                                 scan_data[i]));
        }
    }

    // Merge together obstacles that could be the same obstacle
    mergeSimilarObstacles(obstacles, max_obstacle_angle_diff);

    return obstacles;
}

LidarObstacle LidarDecision::mostDangerousObstacle(const std::vector<LidarObstacle> obstacles) {
    // Return obstacle with the greatest danger score
    return *std::max_element(obstacles.begin(), obstacles.end(),
                              [&] (LidarObstacle obs1, LidarObstacle obs2){
                                  return obs1.dangerScore() < obs2.dangerScore();
                              });
}

void LidarDecision::mergeSimilarObstacles(std::vector<LidarObstacle>& obstacles, float max_angle_diff) {
    // Ensure the list of obstacles is sorted in order of ascending angle
    std::sort(obstacles.begin(), obstacles.end(),
        [&] (LidarObstacle l1, LidarObstacle l2){
            return l1.getAvgAngle() < l2.getAvgAngle();
        });

    // Merge similar obstacles
    int i = 0;
    while(i < obstacles.size() - 1){
        // Check if angle difference between two consecutive scans is less then max_angle_diff
        if (abs(obstacles[i+1].getMinAngle() - obstacles[i].getMaxAngle()) < max_angle_diff){
            // Merge next obstacle into current one
            obstacles[i].mergeInLidarObstacle(obstacles[i+1]);
            obstacles.erase(obstacles.begin()+i+1);
        } else {
            i++;
        }
    }
}

geometry_msgs::Twist LidarDecision::twist_message_from_obstacle(LidarObstacle obstacle,
                                                               distance_t danger_distance,
                                                               angle_t danger_angle,
                                                               float linear_vel_multiplier,
                                                               float angular_vel_multiplier) {
    geometry_msgs::Twist twist;
    twist.linear.x = 0;
    twist.linear.y = 0;
    twist.linear.z = 0;
    twist.angular.x = 0;
    twist.angular.y = 0;
    twist.angular.z = 0;
    if (obstacle.getAvgDistance() < danger_distance && abs(obstacle.getAvgAngle()) < danger_angle) {
        // TODO: Improve the algorithm here to at least use some sort of scale  (linear or otherwise) dependent on obstacle distance and angle
        twist.linear.x = linear_vel_multiplier;
        twist.angular.z = angular_vel_multiplier;
    }
    return twist;
}
