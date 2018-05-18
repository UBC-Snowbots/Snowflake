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

#include "IGVCVisualizerNode.h"

// Allocation of memory for static member variables
float IGVCVisualizerNode::h_margin_of_error;
float IGVCVisualizerNode::s_margin_of_error;
float IGVCVisualizerNode::v_margin_of_error;

boost::shared_ptr<pcl::visualization::PCLVisualizer> IGVCVisualizerNode::viewer;
pcl::PointCloud<pcl::PointXYZRGB>::Ptr IGVCVisualizerNode::raw_visualized_cloud;
bool IGVCVisualizerNode::isPaused;

IGVCVisualizerNode::IGVCVisualizerNode(int argc,
                                       char** argv,
                                       std::string node_name) {
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

    // Setup raw pcl subscriber
    std::string raw_pcl_topic = "/input_cloud";
    uint32_t queue_size       = 1;
    raw_pcl_sub               = nh.subscribe<sensor_msgs::PointCloud2>(
    raw_pcl_topic, queue_size, &IGVCVisualizerNode::rawPCLCallBack, this);

    // Setup filtered pcl subscriber
    std::string filtered_pcl_topic = "/height_filter/output";
    filtered_pcl_sub               = nh.subscribe<sensor_msgs::PointCloud2>(
    filtered_pcl_topic,
    queue_size,
    &IGVCVisualizerNode::filteredPCLCallBack,
    this);

    // Setup timer callback for updating the camera
    timer = nh.createTimer(ros::Duration(image_update_rate),
                           &IGVCVisualizerNode::updateVisualizerCallback,
                           this);
}

void IGVCVisualizerNode::rawPCLCallBack(
const sensor_msgs::PointCloud2::ConstPtr& input) {
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
        raw_visualized_cloud = pcl_rgb;

        // Remove all old point clouds from the view
        viewer->removeAllPointClouds(raw_channel);

        // Show the new point cloud retrieved from the camera
        viewer->addPointCloud(
        raw_visualized_cloud, "raw_visualized_cloud", raw_channel);

        // Setup how big the points are in the point cloud
        viewer->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
        1.0,
        "raw_visualized_cloud");
    }
}

void IGVCVisualizerNode::filteredPCLCallBack(
const sensor_msgs::PointCloud2::ConstPtr& input) {
    // Only update point cloud if system is not paused
    if (!isPaused) {
        // Obtain the ROS pointcloud and convert into PCL Pointcloud2
        pcl::PCLPointCloud2::Ptr pcl_input(new pcl::PCLPointCloud2);
        pcl_conversions::toPCL(*(input), *(pcl_input));

        // Converts from the Pointcloud2 format to PointcloudRGB
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_pcl(
        new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromPCLPointCloud2(*pcl_input, *filtered_pcl);

        // Remove all old point clouds from the view
        viewer->removeAllPointClouds(filtered_channel);

        // Show the new filtered cloud retrieved from the camera
        viewer->addPointCloud(
        filtered_pcl, "filtered_pcl_rgb", filtered_channel);

        // Setup how big the points are in the point cloud
        viewer->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1.0, "filtered_pcl_rgb");
    }
}

void IGVCVisualizerNode::updateVisualizerCallback(
const ros::TimerEvent& event) {
    // Update the viewer
    viewer->spinOnce(100);
    // Should sleep so keyboard and mouse callbacks can occur.
    boost::this_thread::sleep(boost::posix_time::microseconds(100000));
}

boost::shared_ptr<pcl::visualization::PCLVisualizer>
IGVCVisualizerNode::setUpPCLVisualizer() {
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(
    new pcl::visualization::PCLVisualizer("PCL Viewer"));
    viewer->registerPointPickingCallback(pointPickEventOccurred,
                                         (void*) viewer.get());
    viewer->registerKeyboardCallback(keyboardEventOccurred,
                                     (void*) viewer.get());
    viewer->initCameraParameters();

    viewer->createViewPort(0.0, 0.0, 0.5, 1.0, raw_channel);
    viewer->addText("Raw Point Cloud", 10, 10, "raw_channel text", raw_channel);

    viewer->createViewPort(0.5, 0.0, 1.0, 1.0, filtered_channel);
    viewer->addText(
    "Filtered Point Cloud", 10, 10, "filtered_channel text", filtered_channel);

    return viewer;
}

void IGVCVisualizerNode::retrieveVisualizerParameters(
ros::NodeHandle private_nh) {
    float default_h_margin_of_error = 10;
    SB_getParam(private_nh,
                "h_margin_of_error",
                h_margin_of_error,
                default_h_margin_of_error);

    float default_s_margin_of_error = 0.1;
    SB_getParam(private_nh,
                "s_margin_of_error",
                s_margin_of_error,
                default_s_margin_of_error);

    float default_v_margin_of_error = 0.1;
    SB_getParam(private_nh,
                "v_margin_of_error",
                v_margin_of_error,
                default_v_margin_of_error);

    float default_image_update_rate = 0.1;
    SB_getParam(private_nh,
                "image_update_rate",
                image_update_rate,
                default_image_update_rate);
}

void IGVCVisualizerNode::updateFilterParams(float h, float s, float v) {
    setParameter("hue_filter", "filter_limit_min", h - h_margin_of_error);
    setParameter("hue_filter", "filter_limit_max", h + h_margin_of_error);
    setParameter(
    "saturation_filter", "filter_limit_min", s - s_margin_of_error);
    setParameter(
    "saturation_filter", "filter_limit_max", s + s_margin_of_error);
    setParameter("value_filter", "filter_limit_min", v - v_margin_of_error);
    setParameter("value_filter", "filter_limit_max", v + v_margin_of_error);
}

void IGVCVisualizerNode::setParameter(std::string node_name,
                                      std::string param_name,
                                      float val) {
    // Run command line arguments to dynamically reconfigure parameter
    std::string set_command =
    std::string("rosrun dynamic_reconfigure dynparam set");
    int err = system((set_command + " " + node_name + " " + param_name + " " +
                      std::to_string(val))
                     .c_str());
    // More detailed info can be found here
    // http://wiki.ros.org/dynamic_reconfigure

    if (err) {
        std::cout << "Error occured when setting" << param_name << "for"
                  << node_name << std::endl;
    }
}

void IGVCVisualizerNode::pointPickEventOccurred(
const pcl::visualization::PointPickingEvent& event, void* viewer_void) {
    // Pause any updates to the visualizer
    isPaused = true;

    // Retrieve the RGB point that was selected
    pcl::PointXYZRGB rgb_point =
    raw_visualized_cloud->points.at(event.getPointIndex());
    pcl::PointXYZHSV hsv_point;

    // Convert retrieved RGB point to HSV
    ColourspaceConverter colourConverter;
    colourConverter.PointXYZRGBAtoXYZHSV(rgb_point, hsv_point);

    // Remove old selected point highlights
    viewer->removeAllShapes(0);

    // Highlight current selected point
    viewer->addSphere(rgb_point, 0.05, 0, 0, 255, "rgb_point", 0);

    // Print point information
    std::cout << "Point picked with values..." << std::endl;
    std::cout << "R: " << (int) rgb_point.r << ", G: " << (int) rgb_point.g
              << ", B: " << (int) rgb_point.b << std::endl;
    std::cout << "H: " << hsv_point.h << ", S: " << hsv_point.s
              << ", V: " << hsv_point.v << std::endl;
    std::cout << std::endl;

    std::cout << "Updating filter parameters..." << std::endl;
    // Dynamically reconfigure filter parameters
    updateFilterParams(hsv_point.h, hsv_point.s, hsv_point.z);
    std::cout << "Finished updating filter parameters." << std::endl;

    // Resume any updates to the visualizer
    isPaused = false;
}

void IGVCVisualizerNode::keyboardEventOccurred(
const pcl::visualization::KeyboardEvent& event, void* viewer_void) {
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer =
    *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer>*>(
    viewer_void);
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
