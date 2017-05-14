//
// Created by sb on 25/03/17.
//

#include <ZedFilter.h>

// The constructor
ZedFilter::ZedFilter(int argc, char **argv, std::string node_name)
{
    ros::init(argc, argv, node_name);

    // Setup NodeHandles
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    // Setup Subscriber(s)
    std::string camera_image_topic_name = "/zed/point_cloud/cloud_registered";
    int queue_size = 1;
    raw_image_subscriber = nh.subscribe(camera_image_topic_name, queue_size, &ZedFilter::imageCallBack, this);
    
    // Setup Publisher(s)
    std::string filtered_image_topic_name = "/zed_filter/filtered_point_cloud";
    filtered_image_publisher = nh.advertise<PointCloudRGB>(filtered_image_topic_name, queue_size);

    base_link_name = "base_link";

    PointCloudFilter::FilterValues filter_values;
    // Obtain filter value parameters
    SB_getParam(private_nh, "h_min", filter_values.h_min, (float) 1.0);
    SB_getParam(private_nh, "h_max", filter_values.h_max, (float) 360.0);
    SB_getParam(private_nh, "s_min", filter_values.s_min, (float) 0);
    SB_getParam(private_nh, "s_max", filter_values.s_max, (float) 1);
    SB_getParam(private_nh, "v_min", filter_values.v_min, (float) 0);
    SB_getParam(private_nh, "v_max", filter_values.v_max, (float) 1);

    filter = PointCloudFilter(filter_values);

}

void ZedFilter::imageCallBack(const sensor_msgs::PointCloud2::ConstPtr& zed_camera_output) {
    sensor_msgs::PointCloud2 transformed_input;
    SB_doTransform(*zed_camera_output, transformed_input, base_link_name);

    // Conversion to PCL datatype
    pcl::PCLPointCloud2 temp;
    pcl_conversions::toPCL(*zed_camera_output, temp);
    PointCloudRGB::Ptr point_cloud_RGB(new PointCloudRGB);
    pcl::fromPCLPointCloud2(temp, *point_cloud_RGB);

    // Filter Values
    PointCloudRGB::Ptr output_cloud(new PointCloudRGB);
    filter.filterCloud(point_cloud_RGB, output_cloud);
    output_cloud->header.frame_id = "/zed_current_frame";
    // Publish output
    filtered_image_publisher.publish(transformed_input);
}
