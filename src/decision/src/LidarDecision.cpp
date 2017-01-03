/*
 * Created By: Jimmy Yeh
 * Created On: November 26, 2016
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
    std::string laserscan_topic_name = "/elsa/scan";
    int refresh_rate = 10;
    scan_subscriber = public_nh.subscribe(laserscan_topic_name, refresh_rate, &LidarDecision::scanCallBack, this);

    // Setup Publisher(s)
    std::string twist_topic = public_nh.resolveName("command");
    uint32_t queue_size = 10;
    twist_publisher = nh.advertise<geometry_msgs::Twist>(twist_topic, queue_size);

}

// This is called whenever a new message is received
void LidarDecision::scanCallBack(const sensor_msgs::LaserScan::ConstPtr& raw_scan) {
    // Deal with new messages here
    twist_publisher.publish(manage_twist(raw_scan));
}

bool LidarDecision::obstacle_in_range(double min_angle, double max_angle, float max_obstacle_distance,
                                      const sensor_msgs::LaserScan::ConstPtr& raw_scan) {
    int init_pos_vec, end_pos_vec;

    init_pos_vec = static_cast<int>((min_angle - raw_scan->angle_min) / raw_scan->angle_increment);
    end_pos_vec = static_cast<int>((max_angle - raw_scan->angle_min) / raw_scan->angle_increment);

    return std::any_of(
             raw_scan->ranges.begin()+init_pos_vec, raw_scan->ranges.begin()+end_pos_vec,
             [&](auto const& v){
                 return v < max_obstacle_distance;
             });

}

int LidarDecision::linear_speed(bool in_near, bool in_mid){
    if (in_near) return 0;
    else if (in_mid) return 10;
    else return 20;
}

int LidarDecision::angular_speed(bool in_near, bool in_mid, bool in_far){
    if (in_near) return 20;
    else if (in_mid) return 10;
    else if (in_far) return 5;
    else return 0;
}

geometry_msgs::Twist LidarDecision::manage_twist(const sensor_msgs::LaserScan::ConstPtr& raw_scan) {

    geometry_msgs::Twist vel_msg;

    //the scan is from right to left
    //the following corresponds to distance value readings in vector ranges
    float dis_near= 5;
    float dis_mid = 10;
    float dis_far = 20;
    //the following corresponds to the element sequence inside the vector ranges
    int angle_side_near = 20;
    int angle_side_mid = 10;
    int angle_side_far = 5;

    //step 1 obstacle in near, mid or far range (inside impact region);
    bool in_near = obstacle_in_range(raw_scan->angle_min + angle_side_near*raw_scan->angle_increment,
                                     raw_scan->angle_max - angle_side_near*raw_scan->angle_increment, dis_near, raw_scan);
    bool in_mid = obstacle_in_range(raw_scan->angle_min + angle_side_mid*raw_scan->angle_increment,
                                    raw_scan->angle_max - angle_side_mid*raw_scan->angle_increment, dis_mid, raw_scan);
    bool in_far = obstacle_in_range(raw_scan->angle_min + angle_side_far*raw_scan->angle_increment,
                                    raw_scan->angle_max - angle_side_far*raw_scan->angle_increment, dis_far, raw_scan);
    double half_angle = (raw_scan->angle_max+raw_scan->angle_min)/2;

    bool on_right = obstacle_in_range(raw_scan->angle_min, half_angle, dis_far, raw_scan);

    //step 2 determine turn

    vel_msg.linear.x = linear_speed(in_near, in_mid);
    vel_msg.angular.z = (on_right ? 1 : -1) * angular_speed(in_near,in_mid,in_far);
    vel_msg.linear.y = 0;
    vel_msg.linear.z = 0;
    vel_msg.angular.x = 0;
    vel_msg.angular.y = 0;

    return vel_msg;
}

