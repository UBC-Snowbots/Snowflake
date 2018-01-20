/*
 * Created By: Kevin Luo
 * Modified by: Valerian Ratu
 * Created On: September 22, 2016
 * Description: The Final Decision node, arbitrates between the decisions of
 * other nodes.
 */

#include <FinalDecision.h>

FinalDecision::FinalDecision(int argc, char** argv, std::string node_name) {
    // Setup NodeHandles
    ros::init(argc, argv, node_name);
    ros::NodeHandle public_nh;
    ros::NodeHandle private_nh("~");

    // Setup Subscriber(s)
    std::string lidar_decision_topic_name  = "/lidar_decision/twist";
    std::string vision_decision_topic_name = "/vision_decision/twist";
    std::string gps_decision_topic_name    = "/gps_decision/twist";

    uint32_t queue_size = 1;
    lidar_subscriber    = public_nh.subscribe(
    lidar_decision_topic_name, queue_size, &FinalDecision::lidarCallBack, this);
    vision_subscriber = public_nh.subscribe(vision_decision_topic_name,
                                            queue_size,
                                            &FinalDecision::visionCallBack,
                                            this);
    gps_subscriber = public_nh.subscribe(
    gps_decision_topic_name, queue_size, &FinalDecision::gpsCallBack, this);

    // Setup Publisher(s)
    std::string twist_topic = "/cmd_vel";
    twist_publisher =
    public_nh.advertise<geometry_msgs::Twist>(twist_topic, queue_size);
}

void FinalDecision::lidarCallBack(
const geometry_msgs::Twist::ConstPtr& lidar_decision) {
    recent_lidar = *lidar_decision;
    publishTwist(arbitrator(recent_lidar, recent_vision, recent_gps));
}

void FinalDecision::visionCallBack(
const geometry_msgs::Twist::ConstPtr& vision_decision) {
    recent_vision = *vision_decision;
    publishTwist(arbitrator(recent_lidar, recent_vision, recent_gps));
}

void FinalDecision::gpsCallBack(
const geometry_msgs::Twist::ConstPtr& gps_decision) {
    recent_gps = *gps_decision;
    publishTwist(arbitrator(recent_lidar, recent_vision, recent_gps));
}

void FinalDecision::publishTwist(geometry_msgs::Twist twist) {
    twist_publisher.publish(twist);
}

geometry_msgs::Twist
FinalDecision::arbitrator(geometry_msgs::Twist recent_lidar,
                          geometry_msgs::Twist recent_vision,
                          geometry_msgs::Twist recent_gps) {
    if (recent_lidar.angular.z != 0) {
        return recent_lidar;
    } else if (fabs(recent_vision.angular.z) != 0) {
        return recent_vision;
    } else {
        return recent_gps;
    }
}
