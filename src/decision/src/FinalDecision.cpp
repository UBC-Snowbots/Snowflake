/*
 * Created By: YOUR NAME HERE
 * Created On: September 22, 2016
 * Description: The Decision Node for GPS, takes in a point relative to
 *              the robots location and heading and broadcasts a
 *              recommended twist message
 */

#include <FinalDecision.h>


FinalDecision::FinalDecision(int argc, char **argv, std::string node_name) {
    // Setup NodeHandles
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;
    ros::NodeHandle public_nh("~");

    // Setup Subscriber(s)
    std::string lidar_decision_topic_name = "/lidar_decision/command";
    std::string vision_decision_topic_name = "/vision_decision/command";
    std::string gps_decision_topic_name = "/gps_decision/command";

    int refresh_rate = 10;
    lidar_subscriber = public_nh.subscribe(lidar_decision_topic_name, refresh_rate, &FinalDecision::lidarCallBack, this);
    vision_subscriber = public_nh.subscribe(vision_decision_topic_name, refresh_rate, &FinalDecision::visionCallBack, this);
    gps_subscriber = public_nh.subscribe(gps_decision_topic_name, refresh_rate, &FinalDecision::gpsCallBack, this);

    // Setup Publisher(s)
    std::string twist_topic = public_nh.resolveName("command");
    uint32_t queue_size = 10;
    twist_publisher = nh.advertise<geometry_msgs::Twist>(twist_topic, queue_size);

}


void FinalDecision::lidarCallBack(const geometry_msgs::Twist::ConstPtr& lidar_decision) {
    // Deal with new messages here
    recent_lidar = *lidar_decision;
    arbitrator(recent_lidar,recent_vision,recent_gps);
}


void FinalDecision::visionCallBack(const geometry_msgs::Twist::ConstPtr& vision_decision) {
    // Deal with new messages here
    recent_vision = *vision_decision;
    arbitrator(recent_lidar,recent_vision,recent_gps);
}


void FinalDecision::gpsCallBack(const geometry_msgs::Twist::ConstPtr& gps_decision) {
    // Deal with new messages here
    recent_gps = *gps_decision;
    arbitrator(recent_lidar,recent_vision,recent_gps);
}

void FinalDecision::publishTwist(geometry_msgs::Twist twist){
    twist_publisher.publish(twist);
}


geometry_msgs::Twist FinalDecision::arbitrator(geometry_msgs::Twist recent_lidar, geometry_msgs::Twist recent_vision, geometry_msgs::Twist recent_gps){
    if(recent_lidar.angular.z != 0)
        return recent_lidar;
    else if(recent_vision.angular.z != 0)
        return recent_vision;
    else
        return recent_gps;
}

bool FinalDecision::turning(geometry_msgs::Twist twist){
    return (twist.angular.z != 0);
}