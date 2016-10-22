/*
 * Created By: Gareth Ellis
 * Created On: September 22, 2016
 * Description: The Lidar decision node, takes in a raw Lidar scan
 *              and broadcasts a recommended Twist message
 */

#include <LidarDecision.h>

#include <algorithm>
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

void LidarDecision::publishTwist(geometry_msgs::Twist twist){
    twist_publisher.publish(twist);
}





bool LidarDecision::obstacle_in_range(double min_angle, double max_angle, float obstacle_distance,
                                      const sensor_msgs::LaserScan::ConstPtr& raw_scan){
    int i;
    int init_pos_vec, end_pos_vec;

    init_pos_vec = static_cast<int>((min_angle - raw_scan->angle_min)/raw_scan->angle_increment);
    end_pos_vec = static_cast<int>((max_angle - raw_scan->angle_min)/ raw_scan->angle_increment);
    /*bool exist_obstacle = false;
    for (i=init_pos_vec;i<=end_pos_vec;i++){
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

geometry_msgs::Twist LidarDecision::name(const sensor_msgs::LaserScan::ConstPtr& raw_scan){


}

