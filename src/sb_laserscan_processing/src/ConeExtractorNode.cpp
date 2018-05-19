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

    /* Get ros params */
    std::string cone_dist_tol_param = "cone_dist_tol";
    double default_cone_dist_tol = 0.01;
    SB_getParam(private_nh,
                cone_dist_tol_param,
                cone_dist_tol,
                default_cone_dist_tol);

    std::string cone_rad_exp_param = "cone_rad_exp";
    double default_cone_rad_exp = 0.3;
    SB_getParam(private_nh,
                cone_rad_exp_param,
                cone_rad_exp,
                default_cone_rad_exp);

    std::string cone_rad_tol_param = "cone_rad_tol";
    double default_cone_rad_tol = 0.1; //Change later
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
    laser_subscriber = nh.subscribe(subscribe_topic, queue_size, &ConeExtractorNode::laserCallBack, this);

    std::string output_cone_topic = "output_cone_obstacle"; //Placeholder
    cone_publisher = private_nh.advertise<mapping_igvc::ConeObstacle>(
            output_cone_topic, queue_size
    );

    std::string marker_topic = "markers"; //Placeholder
    rviz_publisher = private_nh.advertise<visualization_msgs::Marker>(
            marker_topic, queue_size
    );
}


void ConeExtractorNode::laserCallBack(const sensor_msgs::LaserScan::ConstPtr& ptr){
    sensor_msgs::LaserScan laser_msg = *ptr;

    std::vector<mapping_igvc::ConeObstacle> cones = ConeIdentification::identifyCones(laser_msg, cone_dist_tol, cone_rad_exp, cone_rad_tol, min_points_in_cone, ang_threshold);
    for (int i=0; i<cones.size(); i++){ //Publish cones individually
        cone_publisher.publish(cones[i]);

        std::cout<<"Cone #:"<<i<<std::endl;
        std::cout<<"X: "<<cones[i].center.x<<std::endl;
        std::cout<<"Y: "<<cones[i].center.y<<std::endl;
        std::cout<<"R: "<<cones[i].radius<<std::endl;
        std::cout<<std::endl;
    }
    std::cout<<"================="<<std::endl;
    publishMarkers(cones);
}


void ConeExtractorNode::publishMarkers(std::vector<mapping_igvc::ConeObstacle> cones){
    if (cones.empty())
        return;

    visualization_msgs::Marker marker;

    marker.id = 0;
    marker.header.frame_id = cones[0].header.frame_id;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;

    marker.type = visualization_msgs::Marker::POINTS;

    // POINTS markers use x and y scale for width/height respectively
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;

    // Points are green
    marker.color.g = 1.0f;
    marker.color.a = 1.0;

    for (int i=0; i<cones.size(); i++){
        geometry_msgs::Point coneCenter;
        coneCenter.x = cones[i].center.x;
        coneCenter.y = cones[i].center.y;
        marker.points.push_back(coneCenter);
    }

    rviz_publisher.publish(marker);

}