/*
 * Created By: Gareth Ellis
 * Created On: September 22, 2016
 * Description: The Lidar decision node, takes in a raw Lidar scan
 *              and broadcasts a recommended Twist message
 */

#include <LidarDecision.h>
using namespace std;

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
}





double pi = 3.1415926;


bool LidarDecision::obstacle_in_range(double min_angle, double max_angle, float obstacle_distance,
                                      const sensor_msgs::LaserScan::ConstPtr& raw_scan) {
    int i;
    int init_pos_vec, end_pos_vec;

    init_pos_vec = static_cast<int>((min_angle - raw_scan->angle_min) / raw_scan->angle_increment);
    end_pos_vec = static_cast<int>((max_angle - raw_scan->angle_min) / raw_scan->angle_increment);

    /*bool exist_obstacle = false;
    for (i = init_pos_vec; i <= end_pos_vec; i++) {
        if (raw_scan->ranges[i] < obstacle_distance) exist_obstacle = true;
    }
    if (exist_obstacle) return true;
    else return false;*/
    return std::any_of(
             raw_scan->ranges.begin()+init_pos_vec, raw_scan->ranges.begin()+end_pos_vec,
             [&](auto const& v){
                 return v < obstacle_distance;
             });

}

//the twist message to be sent out will be modified throught the following functions


// v1. very crude function for turning, will not be used
geometry_msgs::Twist LidarDecision::manage_twist_1(float obstacle_distance, float forward_speed,  float angular_speed,
                                                 const sensor_msgs::LaserScan::ConstPtr& raw_scan){
    geometry_msgs::Twist vel_msg;
    bool left_block = LidarDecision::obstacle_in_range(0.0, pi/2, obstacle_distance, raw_scan);
    bool right_block = LidarDecision::obstacle_in_range(pi/2, pi, obstacle_distance, raw_scan);
    vel_msg.angular.x = forward_speed;
    vel_msg.angular.y = 0;
    vel_msg.linear.x = 0;
    vel_msg.linear.y = 0;
    vel_msg.linear.z = 0;
    if (!left_block and !right_block) vel_msg.angular.z = 0;
    if (left_block) vel_msg.angular.z = - angular_speed;
    if (right_block) vel_msg.angular.z = angular_speed;
    if (right_block and left_block) {
        vel_msg.linear.x = 0;
        vel_msg.angular.z = angular_speed;
    }
    return vel_msg;
}

// v2. three distance grid decisioning


int LidarDecision::turn_left(const sensor_msgs::LaserScan::ConstPtr& raw_scan, float distance_used){
    int left_side_value = 0;
    int right_side_value = 0;
    int angle_increment_steps;
    angle_increment_steps = static_cast<int>((raw_scan->angle_max - raw_scan->angle_min) / raw_scan->angle_increment);

    //ranges start at 0?
    while ((raw_scan->ranges[left_side_value]>distance_used)||(left_side_value < angle_increment_steps/2)){
        left_side_value++;
    }
    std::cout << "left_side " << left_side_value << std::endl;
    while((raw_scan->ranges[angle_increment_steps - right_side_value] > distance_used)||(right_side_value < angle_increment_steps/2)){
        right_side_value++;
    }
    std::cout << "right_side " << right_side_value << std::endl;
    if (left_side_value > right_side_value) return 1;
    else return (-1);

}


geometry_msgs::Twist LidarDecision::manage_twist_2(const sensor_msgs::LaserScan::ConstPtr& raw_scan) {

    geometry_msgs::Twist vel_msg;
    float dis_near= 5;
    float dis_mid = 10;
    float dis_far = 20;
    int angle_increment_steps;
    angle_increment_steps = static_cast<int>((raw_scan->angle_max - raw_scan->angle_min) / raw_scan->angle_increment);
    int angle_value_exact_front;
    angle_value_exact_front = angle_increment_steps / 2;
    int angle_side_near = 1; //the area that would not be obstacle to motion
    int angle_side_mid = 1;
    int angle_side_far = 1;

    //step 1 obstacle in near, mid or far range (inside impact region);
    bool in_near = obstacle_in_range(raw_scan->angle_min + angle_side_near, raw_scan->angle_max - angle_side_near,
                                     dis_near, raw_scan);
    bool in_mid = obstacle_in_range(raw_scan->angle_min + angle_side_mid, raw_scan->angle_max - angle_side_mid, dis_mid,
                                    raw_scan);
    bool in_far = obstacle_in_range(raw_scan->angle_min + angle_side_far, raw_scan->angle_max - angle_side_far, dis_far,
                                    raw_scan);

    //step 2 determine turn
    //scan from side to see either left or right has more space, for certain distance
    float distance_used;
    float linear_speed = 13;
    float angular_speed = 13;
    //determine distance, velocity for different case
    if (in_far) {
        //certain linear;
        vel_msg.linear.x = linear_speed;
        vel_msg.angular.z = turn_left(raw_scan, dis_far) * angular_speed;
    } else if (in_mid) {
        //certain linear;
        vel_msg.linear.x = linear_speed;
        vel_msg.angular.z = turn_left(raw_scan, dis_mid)*angular_speed;
    } else if (in_near) {
        //certain linear;
        vel_msg.linear.x = linear_speed;
        vel_msg.angular.z = turn_left(raw_scan, dis_near)*angular_speed;
    } else {
        // no obstacle;
        //certain linear;
        vel_msg.linear.x = linear_speed;
        vel_msg.angular.z = 0;
    }
    vel_msg.linear.y = 0;
    vel_msg.linear.z = 0;
    vel_msg.angular.x = 0;
    vel_msg.angular.y = 0;

    return vel_msg;
}

void LidarDecision::publishTwist(geometry_msgs::Twist twist){
    twist_publisher.publish(twist);
}


// double range_immediate_front; //the angle range that coresponds to right infront of the robot
// double a = range_immediate_front;
//  double safe_distance;
//bool front_block = LidarDecision::obstacle_in_range(pi/2-a, pi/2+a, safe_distance, raw_scan);
