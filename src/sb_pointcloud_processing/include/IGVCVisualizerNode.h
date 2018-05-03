/**
 * Created by: Robyn Castro
 * Created on: 2018/4/29
 * Description: A class which visualizes point clouds, and allows
 *              dynamic reconfiguring of pointclouds.
 * References:
 *      Tutorial for pcl_visualizer -
 *          http://pointclouds.org/documentation/tutorials/pcl_visualizer.php
 *      Dynamic Reconfigure for C++ -
 *          http://wiki.ros.org/hokuyo_node/Tutorials/UsingDynparamToChangeHokuyoLaserParameters#PythonAPI
 */
#ifndef SB_POINTCLOUD_PROCESSING_IGVCVISUALIZERNODE_H
#define SB_POINTCLOUD_PROCESSING_IGVCVISUALIZERNODE_H

#include <cstdlib>
#include <iostream>

// Pointcloud
#include <pcl/common/common_headers.h>
#include <pcl/filters/filter.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_ros/filters/filter.h>

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
    void rawPCLCallBack(const sensor_msgs::PointCloud2::ConstPtr& input);

    /**
     * Callback for the filtered pointcloud
     *
     * @param address of filtered point cloud
     */
    void filteredPCLCallBack(const sensor_msgs::PointCloud2::ConstPtr& input);

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
     * Whenever a point is clicked, it will dynamically reconfigure filter
     * values
     * for h, s, and v based on the point clicked.
     *
     * @param event the point picking event
     * @param viewer_void void pointer of the viewer
     */
    static void
    pointPickEventOccurred(const pcl::visualization::PointPickingEvent& event,
                           void* viewer_void);

    /**
     * When "h" is pressed, help documentation is printed out.
     * When "r" is pressed, resets the viewer state to default.
     * When "p" is pressed, pauses the viewer updates.
     *
     * @param event the keyboard press event
     * @param viewer_void void pointer of the viewer
     */
    static void
    keyboardEventOccurred(const pcl::visualization::KeyboardEvent& event,
                          void* viewer_void);

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
    static void
    setParameter(std::string node_name, std::string param_name, float val);

    ros::Subscriber raw_pcl_sub;
    ros::Subscriber filtered_pcl_sub;

    // Viewport Channels
    int raw_channel;
    int filtered_channel;

    // Timer Variables
    ros::Timer timer;
    float image_update_rate;

    // Margin of Error Variables
    static float h_margin_of_error;
    static float s_margin_of_error;
    static float v_margin_of_error;

    // Determines whether or not viewer should be updated
    static bool isPaused;

    // The viewer object that controls the pcl visualizer
    static boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;

    // The point cloud to be displayed onto the raw_channel
    static pcl::PointCloud<pcl::PointXYZRGB>::Ptr raw_visualized_cloud;
};

#endif // SB_POINTCLOUD_PROCESSING_IGVCVISUALIZERNODE_H
