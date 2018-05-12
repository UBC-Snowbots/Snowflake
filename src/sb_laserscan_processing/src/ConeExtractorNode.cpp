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

    /* Get ros params */
    std::string cone_dist_tol_param = "cone_dist_tol";
    double default_cone_dist_tol = 0.01;
    SB_getParam(private_nh,
                cone_dist_tol_param,
                cone_dist_tol,
                default_cone_dist_tol);

    std::string cone_rad_exp_param = "cone_rad_exp";
    double default_cone_rad_exp = 0.1;
    SB_getParam(private_nh,
                cone_rad_exp_param,
                cone_rad_exp,
                default_cone_rad_exp);

    std::string cone_rad_tol_param = "cone_rad_tol";
    double default_cone_rad_tol = 0.15; //Change later
    SB_getParam(private_nh,
                cone_rad_tol_param,
                cone_rad_tol,
                default_cone_rad_tol);

    std::string min_points_in_cone_param = "min_points_in_cone";
    int default_min_points_in_cone = 5; //Change later
    SB_getParam(private_nh,
                min_points_in_cone_param,
                min_points_in_cone,
                default_min_points_in_cone);

    std::string ang_threshold_param = "ang_threshold";
    double default_ang_threshold = 2.3; //Change later
    SB_getParam(private_nh,
                ang_threshold_param,
                ang_threshold,
                default_ang_threshold);

    std::string subscribe_topic = "/scan"; // Setup subscriber to laserscan (Placeholder)
    laser_subscriber = nh.subscribe(subscribe_topic, refresh_rate, &ConeExtractorNode::laserCallBack, this);

    std::string publish_topic = "output_cone_obstacle"; //Placeholder
    cone_publisher = private_nh.advertise<mapping_igvc::ConeObstacle>(
            publish_topic, queue_size
    );
}


void ConeExtractorNode::laserCallBack(const sensor_msgs::LaserScan::ConstPtr& ptr){
    sensor_msgs::LaserScan laser_msg = *ptr;

    std::vector<mapping_igvc::ConeObstacle> cones = ConeIdentification::identifyCones(laser_msg, cone_dist_tol, cone_rad_exp, cone_rad_tol, min_points_in_cone, ang_threshold);
    for (int i=0; i<cones.size(); i++){ //Publish cones individually
        cone_publisher.publish(cones[i]);

        /*
        std::cout<<"Cone #:"<<i<<std::endl;
        std::cout<<"X: "<<cones[i].center.x<<std::endl;
        std::cout<<"Y: "<<cones[i].center.y<<std::endl;
        std::cout<<"R: "<<cones[i].radius<<std::endl;
        std::cout<<std::endl;*/
    }
}
