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

// Snowbots
#include "sb_utils.h"

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
     * Whenever a timer event occurs, the image is updated
     * 
     * @param event timer based event based on image_update_rate
     */
    void updateVisualizerCallback(const ros::TimerEvent& event);

    /**
     * Initialise parameters for margin of errors and image update rate
     *
     * @param private_nh the private node handle
     */
    void retrieveVisualizerParameters(ros::NodeHandle private_nh);

    /**
     * Whenever a point is clicked, it will dynamically reconfigure filter values
     * for h, s, and v based on the point clicked.
     *
     * @param event the point picking event
     * @param viewer_void void pointer of the viewer
     */
    static void pointPickEventOccurred (const pcl::visualization::PointPickingEvent &event,
                                 void* viewer_void);

    /**
     * When "h" is pressed, help documentation is printed out.
     * When "r" is pressed, resets the viewer state to default.
     * When "p" is pressed, pauses the viewer updates.
     *
     * @param event the keyboard press event
     * @param viewer_void void pointer of the viewer
     */
    static void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event, void* viewer_void);


    /**
     * Dynamically reconfigures hue, saturation and value filter parameters.
     *
     * Min and maxes for the filter are determined by the passed in h, s, and v
     * values and the set margin of errors.
     *
     * @param h hue
     * @param s saturation
     * @param v value
     */
    static void updateFilterParams(float h, float s, float v);

    /**
     * Run command line arguments to dynamically reconfigure a node parameter
     *
     * @param node_name name of the node to reconfigure
     * @param param_name name of the parameter to reconfigure
     * @param val new value of the parameter
     */
    static void setParameter(std::string node_name, std::string param_name, float val);

    ros::Subscriber image_sub;

    // Timer Variables
    ros::Timer timer;
    float image_update_rate;

    // Margin of Error Variables
    static float h_margin_of_error;
    static float s_margin_of_error;
    static float v_margin_of_error;

    // Visualizer Variables
    static bool isPaused;
    static boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    static pcl::PointCloud<pcl::PointXYZRGB>::Ptr visualized_cloud;

};


#endif //SB_POINTCLOUD_PROCESSING_IGVCVISUALIZERNODE_H
