//
// Created by robyn on 20/10/18.
//

#ifndef RISKANALYSISNODE_H
#define RISKANALYSISNODE_H

// ROS
#include <ros/ros.h>

// Utilities
#include <RvizUtils.h>
#include <sb_utils.h>

// Messages
#include "mapping_msgs_urc/RiskArea.h"
#include "mapping_msgs_urc/RiskAreaArray.h"
#include <geometry_msgs/Polygon.h>
#include <sensor_msgs/PointCloud2.h>

// Point Cloud
#include <pcl/PCLPointCloud2.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <tr1/unordered_map>

// Risk Analysis
#include "RiskAnalysis.h"

class RiskAnalysisNode {
  public:
    /**
     * Constructor
     */
    RiskAnalysisNode(int argc, char** argv, std::string node_name);

  private:
    /*
     * The callback function is called whenever the node receives a
     * PointCloud message. It converts sensor_msgs PointCloud2 pointer
     * to PCL PointCloud pointer and then extracts risk areas from the
     * PointCloud
     *
     * @param point_cloud the received PointCloud message
     */
    void pclCallBack(const sensor_msgs::PointCloud2ConstPtr point_cloud);

    /**
     * Adds in all the header information for the message then publishes
     * the risk_areas
     *
     * @param risk_areas
     */
    void publishMarkers(mapping_msgs_urc::RiskAreaArray risk_areas);

    /**
     * Returns a color corresponding to the inputted risk
     *
     * @param risk
     * @return visualization_msgs::Marker::_color_type color corresponding to
     * inputted risk
     */
    visualization_msgs::Marker::_color_type convertRiskToColor(float risk);

    RiskAnalysis risk_analysis;

    float area_of_interest_width;
    float area_of_interest_height;
    float risk_multiplier;

    int num_vertical_cell_div;
    int num_horizontal_cell_div;

    int region_min_points;

    int seq_count;

    mapping_msgs_urc::RiskAreaArray pcl_risk;

    std::vector<visualization_msgs::Marker::_color_type> gradient;

    ros::Subscriber pcl_subscriber;
    ros::Publisher risk_publisher;
    ros::Publisher risk_marker_publisher;
};

#endif // RISKANALYSISNODE_H
