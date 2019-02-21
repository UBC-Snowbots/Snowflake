/*
 * Created By: Robyn Castro
 * Created On: October 14, 2018
 * Description: Takes in a point cloud and creates a regional assessment.
 *
 */

#include "RiskAnalysisNode.h"

RiskAnalysisNode::RiskAnalysisNode(int argc,
                                   char** argv,
                                   std::string node_name) {
    // Setup NodeHandles
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    // Initialise parameters
    float default_width = 10;
    SB_getParam(private_nh,
                "area_of_interest_width",
                area_of_interest_width,
                default_width);

    float default_height = 5;
    SB_getParam(private_nh,
                "area_of_interest_height",
                area_of_interest_height,
                default_height);

    int default_vertical_divisions = 10;
    SB_getParam(private_nh,
                "num_vertical_cell_div",
                num_vertical_cell_div,
                default_vertical_divisions);

    int default_horizontal_divisions = 30;
    SB_getParam(private_nh,
                "num_horizontal_cell_div",
                num_horizontal_cell_div,
                default_horizontal_divisions);

    int default_min_points = 100;
    SB_getParam(
    private_nh, "region_min_points", region_min_points, default_min_points);

    float default_risk_multiplier = 1;
    SB_getParam(private_nh, "risk_multiplier", risk_multiplier, default_risk_multiplier);

    // In the beginning we have not published any markers
    seq_count = 0;

    // Since risk goes from 0 - 10, we need 11 gradient values
    size_t gradient_size = 11;
    double gradient_step = 1.0 / gradient_size;

    gradient.reserve(gradient_size);

    for (int i = 0; i < gradient_size; i++) {
        gradient[i] = snowbots::RvizUtils::createMarkerColor(
        i * gradient_step, gradient_step * (gradient_size - i), 0, 1.0f);
    }

    // Initialise Subscribers
    std::string topic_to_subscribe_to = "input_pointcloud"; // dummy topic name
    int refresh_rate                  = 10;
    pcl_subscriber                    = nh.subscribe(
    topic_to_subscribe_to, refresh_rate, &RiskAnalysisNode::pclCallBack, this);

    // Initialise Publishers
    std::string risk_areas_topic = "output_risk_areas"; // dummy topic name
    uint32_t queue_size          = 1;
    risk_publisher = private_nh.advertise<mapping_msgs_urc::RiskAreaArray>(
    risk_areas_topic, queue_size);

    std::string risk_area_markers_topic =
    "output_risk_area_markers"; // dummy topic name

    risk_marker_publisher =
    private_nh.advertise<visualization_msgs::MarkerArray>(
    risk_area_markers_topic, queue_size);

    // Initialise risk_analysis with parameters
    risk_analysis = RiskAnalysis(area_of_interest_width,
                                 area_of_interest_height,
                                 num_vertical_cell_div,
                                 num_horizontal_cell_div,
                                 region_min_points,
                                 risk_multiplier
                                 );
}

void RiskAnalysisNode::pclCallBack(
const sensor_msgs::PointCloud2ConstPtr point_cloud) {
    pcl::PCLPointCloud2 pcl_pc2;

    // convert sensor_msgs::PointCloud2 to pcl::PCLPointCloud2
    pcl_conversions::toPCL(*point_cloud, pcl_pc2);

    // extract risk
    pcl_risk = risk_analysis.assessPointCloudRisk(pcl_pc2);

    publishMarkers(pcl_risk);

    visualization_msgs::MarkerArray risk_area_markers;
    std::string frame_id = "base_link";
    std::string ns       = "debug";

    // Create the rviz markers
    for (int i = 0; i < pcl_risk.areas.size(); i++) {
        visualization_msgs::Marker risk_area_marker =
        snowbots::RvizUtils::createPolygonMarker2D(
        pcl_risk.areas[i].area,
        convertRiskToColor(pcl_risk.areas[i].score.data),
        snowbots::RvizUtils::createMarkerScale(0.01, 0, 0),
        frame_id,
        ns,
        visualization_msgs::Marker::LINE_STRIP,
        i
        );

        risk_area_markers.markers.push_back(risk_area_marker);
    }

    // Visualise markers
    risk_marker_publisher.publish(risk_area_markers);
}

void RiskAnalysisNode::publishMarkers(
mapping_msgs_urc::RiskAreaArray risk_areas) {
    risk_areas.header.frame_id = "camera_color_optical_frame";

    risk_areas.header.seq = seq_count;
    seq_count++;

    risk_areas.header.stamp = ros::Time::now();

    risk_publisher.publish(risk_areas);
}

visualization_msgs::Marker::_color_type
RiskAnalysisNode::convertRiskToColor(float risk) {
    // Round down to the nearest integer to get risk color
    return gradient[(int) risk*10.0];
}