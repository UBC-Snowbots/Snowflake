/**
 * Created by William Gu on 24/03/18
 * Implementation for a Cone Extractor Node that identifies cones from a laser msg
 */

#include <ConeExtractorNode.h>

ConeExtractorNode::ConeExtractorNode(int argc, char **argv, std::string node_name) {
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    uint32_t queue_size = 1;
    int refresh_rate = 10;

    /* TODO: Get ros params here */
    cone_dist_tol = 1; //CHANGE

    std::string subscribe_topic = "/scan"; // Setup subscriber to laserscan
    laser_subscriber = nh.subscribe(subscribe_topic, refresh_rate, &ConeExtractorNode::laserCallBack, this);

    std::string publish_topic = "output_cone_obstacle"; //Placeholder
    cone_publisher = private_nh.advertise<mapping_igvc::ConeObstacle>(
            publish_topic, queue_size
    );
}

void ConeExtractorNode::laserCallBack(const sensor_msgs::LaserScan::ConstPtr& ptr){
    sensor_msgs::LaserScan laser_msg = *ptr;
    std::vector<mapping_igvc::ConeObstacle> cones = ConeIdentification::identifyCones(laser_msg, cone_dist_tol);
    for (int i=0; i<cones.size(); i++){ //Publish cones individually
        cone_publisher.publish(cones[i]);
    }
};