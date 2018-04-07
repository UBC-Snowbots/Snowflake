//
// Created by robyn on 08/03/18.
//

#ifndef SB_POINTCLOUD_PROCESSING_IGVCVISUALIZERNODE_H
#define SB_POINTCLOUD_PROCESSING_IGVCVISUALIZERNODE_H

#include <iostream>
#include <cstdlib>

// Pointcloud
#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_ros/filters/filter.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>

// ROS
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

// Colour Space Conversions
#include "ColourspaceConverter.h"

class IGVCVisualizerNode {
public:
    /**
     * Constructor
     */
    IGVCVisualizerNode(int argc, char** argv, std::string node_name);

private:
    /**
     * Callback for the raw pointcloud
     *
     * @param address of raw point cloud
     */
    void rawPCLCallBack(const sensor_msgs::PointCloud2::ConstPtr &input);

    /**
     * Initialization of the filter
     */
    boost::shared_ptr<pcl::visualization::PCLVisualizer> setUpPCLVisualizer();

    /**
     * Call back for whenever a point is clicked on
     */
    static void pointPickEventOccurred (const pcl::visualization::PointPickingEvent &event,
                                 void* viewer_void);

    static void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event, void* viewer_void);

    void updateVisualizer(const ros::TimerEvent& event);

    static void updateFilterParams(float h, float s, float v);
    static void setParameter(std::string node_name, std::string param_name, float val);

    ros::Subscriber image_sub;
    ros::Timer timer;
};


#endif //SB_POINTCLOUD_PROCESSING_IGVCVISUALIZERNODE_H
