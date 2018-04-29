//
// Created by robyn on 08/03/18.
//

#include "IGVCVisualizerNode.h"


// Allocation of memory for static member variables
float IGVCVisualizerNode::h_margin_of_error;
float IGVCVisualizerNode::s_margin_of_error;
float IGVCVisualizerNode::v_margin_of_error;

boost::shared_ptr<pcl::visualization::PCLVisualizer> IGVCVisualizerNode::viewer;
pcl::PointCloud<pcl::PointXYZRGB>::Ptr IGVCVisualizerNode::visualized_cloud;
bool IGVCVisualizerNode::isPaused;

IGVCVisualizerNode::IGVCVisualizerNode(int argc, char **argv, std::string node_name) {
    // ROS setup
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    // Always start resumed
    isPaused = false;

    // Initialise visualiser settings
    viewer = setUpPCLVisualizer();

    // Initialise margin of error and image update rate
    retrieveVisualizerParameters(private_nh);

    // Setup subscriber
    std::string pcl_topic = "/zed/point_cloud/cloud_registered";
    uint32_t queue_size = 1;
    image_sub = nh.subscribe<sensor_msgs::PointCloud2>(pcl_topic, queue_size, &IGVCVisualizerNode::rawPCLCallBack,
                                                       this);

    // Setup timer callback for updating the camera
    timer = nh.createTimer(ros::Duration(image_update_rate), &IGVCVisualizerNode::updateVisualizerCallback, this);
}

void IGVCVisualizerNode::rawPCLCallBack(const sensor_msgs::PointCloud2::ConstPtr &input) {
    // Only update point cloud if system is not paused
    if (!isPaused) {
        // Obtain the ROS pointcloud and convert into PCL Pointcloud2
        pcl::PCLPointCloud2::Ptr pcl_input(new pcl::PCLPointCloud2);
        pcl_conversions::toPCL(*(input), *(pcl_input));

        // Converts from the Pointcloud2 format to PointcloudRGB
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_rgb(
                new pcl::PointCloud<pcl::PointXYZRGB>());
        pcl::fromPCLPointCloud2(*pcl_input, *pcl_rgb);

        // Remove NaNs from point cloud
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*pcl_rgb, *pcl_rgb, indices);

        // Renew pcl point data
        visualized_cloud = pcl_rgb;

        // Remove all old point clouds from the view
        viewer->removeAllPointClouds(0);

        // Show the new point cloud retrieved from the camera
        viewer->addPointCloud(visualized_cloud, "visualized_cloud");

        // Setup how big the points are in the point cloud
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1.0,
                                                 "visualized_cloud");
    }
}

void IGVCVisualizerNode::updateVisualizerCallback(const ros::TimerEvent &event) {
    // Update the viewer
    viewer->spinOnce(100);
    boost::this_thread::sleep(boost::posix_time::microseconds(100000));
}

boost::shared_ptr<pcl::visualization::PCLVisualizer> IGVCVisualizerNode::setUpPCLVisualizer() {
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("PCL Viewer"));
    viewer->registerPointPickingCallback(pointPickEventOccurred, (void *) viewer.get());
    viewer->registerKeyboardCallback(keyboardEventOccurred, (void *) viewer.get());
    viewer->initCameraParameters();

    return viewer;
}

void IGVCVisualizerNode::retrieveVisualizerParameters(ros::NodeHandle private_nh) {
    float default_h_margin_of_error = 10;
    SB_getParam(private_nh, "h_margin_of_error", h_margin_of_error, default_h_margin_of_error);

    float default_s_margin_of_error = 0.1;
    SB_getParam(private_nh, "s_margin_of_error", s_margin_of_error, default_s_margin_of_error);

    float default_v_margin_of_error = 0.1;
    SB_getParam(private_nh, "v_margin_of_error", v_margin_of_error, default_v_margin_of_error);

    float default_image_update_rate = 0.1;
    SB_getParam(private_nh, "image_update_rate", image_update_rate, default_image_update_rate);
}

void IGVCVisualizerNode::updateFilterParams(float h, float s, float v) {
    setParameter("hue_filter", "filter_limit_min", h - h_margin_of_error);
    setParameter("hue_filter", "filter_limit_max", h + h_margin_of_error);
    setParameter("saturation_filter", "filter_limit_min", s - s_margin_of_error);
    setParameter("saturation_filter", "filter_limit_max", s + s_margin_of_error);
    setParameter("value_filter", "filter_limit_min", v - v_margin_of_error);
    setParameter("value_filter", "filter_limit_max", v + v_margin_of_error);
}

void IGVCVisualizerNode::setParameter(std::string node_name, std::string param_name, float val) {
    // Run command line arguments to dynamically reconfigure parameter
    std::string set_command = std::string("rosrun dynamic_reconfigure dynparam set");
    system((set_command + " " + node_name + " " + param_name + " " + std::to_string(val)).c_str());
    // More detailed info can be found here http://wiki.ros.org/dynamic_reconfigure
}

void IGVCVisualizerNode::pointPickEventOccurred(const pcl::visualization::PointPickingEvent &event, void *viewer_void) {
    // Retrieve the RGB point that was selected
    pcl::PointXYZRGB rgb_point = visualized_cloud->points.at(event.getPointIndex());
    pcl::PointXYZHSV hsv_point;

    // Convert retrieved RGB point to HSV
    ColourspaceConverter colourConverter;
    colourConverter.PointXYZRGBAtoXYZHSV(rgb_point, hsv_point);

    // Remove old selected point highlights
    viewer->removeAllShapes(0);

    // Highlight current selected point
    viewer->addSphere(rgb_point, 0.05, 0, 0, 255, "rgb_point", 0);

    std::cout << "POINT PICKED!" << std::endl;

    // Dynamically reconfigure filter parameters
    updateFilterParams(hsv_point.h, hsv_point.s, hsv_point.z);
}

void IGVCVisualizerNode::keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event, void *viewer_void) {
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *>(viewer_void);
    if (event.getKeySym() == "r" && event.keyDown())
        std::cout << "'r' was pressed" << std::endl;
    if (event.getKeySym() == "h" && event.keyDown())
        std::cout << "'h' was pressed" << std::endl;
    if (event.getKeySym() == "p" && event.keyDown()) {
        isPaused = !isPaused;
        if (isPaused)
            std::cout << "System Paused." << std::endl;
        else
            std::cout << "System Resumed." << std::endl;
    }
}