//
// Created by robyn on 08/03/18.
//

#include "IGVCVisualizerNode.h"

boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
pcl::PointCloud<pcl::PointXYZRGB>::Ptr visualized_cloud;
bool isPaused;

IGVCVisualizerNode::IGVCVisualizerNode(int argc, char** argv, std::string node_name) {
    // ROS
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    // Set topics
    std::string pcl_topic  = "/zed/point_cloud/cloud_registered";

    viewer = setUpPCLVisualizer();

    isPaused = false;

    // Setup subscriber
    uint32_t queue_size = 1;
    image_sub = nh.subscribe<sensor_msgs::PointCloud2>(pcl_topic, queue_size, &IGVCVisualizerNode::rawPCLCallBack, this);

    // Setup timer callback
    timer = nh.createTimer(ros::Duration(0.1), &IGVCVisualizerNode::updateVisualizer, this);
}

void IGVCVisualizerNode::pointPickEventOccurred (const pcl::visualization::PointPickingEvent &event,
                                                 void* viewer_void) {

    pcl::PointXYZRGB actual_point = visualized_cloud->points.at(event.getPointIndex());
    pcl::PointXYZHSV hsv_point;

    ColourspaceConverter colourConverter;
    colourConverter.PointXYZRGBAtoXYZHSV(actual_point, hsv_point);

    viewer->removeAllShapes(0);
    viewer->addSphere(actual_point, 0.05, 0, 0, 255, "actual_point", 0);

    std::cout << "R: " << (int) actual_point.r << " G: " << (int) actual_point.g << " B: " << (int) actual_point.b << std::endl;
    std::cout << "H: " << hsv_point.h << " S: " << hsv_point.s << " V: " << hsv_point.v << std::endl;
    std::cout << std::endl;

    updateFilterParams(hsv_point.h, hsv_point.s, hsv_point.z);
}

void IGVCVisualizerNode::keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event, void* viewer_void)
{
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

void IGVCVisualizerNode::rawPCLCallBack(const sensor_msgs::PointCloud2::ConstPtr &input) {
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

        visualized_cloud = pcl_rgb;
        viewer->removeAllPointClouds(0);
        viewer->addPointCloud(visualized_cloud, "visualized_cloud");
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1.0,
                                                 "visualized_cloud");
        isPaused = true;
    }
}

void IGVCVisualizerNode::updateVisualizer(const ros::TimerEvent& event) {
    viewer->spinOnce(100);
    boost::this_thread::sleep(boost::posix_time::microseconds(100000));
}

void IGVCVisualizerNode::updateFilterParams(float h, float s, float v) {
    float h_margin_of_error = 10;
    float s_margin_of_error = 0.1;
    float v_margin_of_error = 0.1;

    setParameter("hue_filter", "filter_limit_min", h - h_margin_of_error);
    setParameter("hue_filter", "filter_limit_max", h + h_margin_of_error);
    setParameter("saturation_filter", "filter_limit_min", s - s_margin_of_error);
    setParameter("saturation_filter", "filter_limit_max", s + s_margin_of_error);
    setParameter("value_filter", "filter_limit_min", v - v_margin_of_error);
    setParameter("value_filter", "filter_limit_max", v + v_margin_of_error);
}

void IGVCVisualizerNode::setParameter(std::string node_name, std::string param_name, float val) {
    std::string set_command = std::string("rosrun dynamic_reconfigure dynparam set");
    std::cout << set_command + " " + node_name + " " + param_name + " " + std::to_string(val) << std::endl;
    // system((set_command + " " + node_name + " " + param_name + " " + std::to_string(val)).c_str());
}

boost::shared_ptr<pcl::visualization::PCLVisualizer> IGVCVisualizerNode::setUpPCLVisualizer() {
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->registerPointPickingCallback(pointPickEventOccurred, (void *) viewer.get());
    viewer->registerKeyboardCallback(keyboardEventOccurred, (void *) viewer.get());
    viewer->initCameraParameters ();

    return viewer;
}

