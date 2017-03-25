//
// Created by sb on 25/03/17.
//

#include "ZedFilter.h"

// The constructor
ZedFilter::ZedFilter(int argc, char **argv, std::string node_name) {
    ros::init(argc, argv, node_name);

    // Setup NodeHandles
    ros::NodeHandle nh;
    ros::NodeHandle public_nh("~");

    // Setup Subscriber(s)
    std::string camera_image_topic_name = "/zed/point_cloud/cloud_registered";
    int refresh_rate = 10;
    raw_image_subscriber = public_nh.subcribe(camera_image_topic_name, refresh_rate, &ZedFilter::imageCallBack, this);

    // Setup Publisher(s)
    std::string filtered_image_topic_name = "/zed_filter/filtered_point_cloud";
    uint32_t queue_size = 10;
    filtered_image_publisher = nh.advertise<sensor_msgs::PointCloud2::ConstPtr>(filtered_image_topic_name, queue_size);
}

void imageCallBack(const sensor_msgs::PointCloud2::ConstPtr& zed_camera_output) {
    sensor_msgs::PointCloud2::ConstPtr& filtered_point_cloud;

    // TODO: Map all points to z = 0 plane.
    // TODO: Delete all points that are not close to white.
    // TODO: All other points are mapped to white.

    publishFilteredImage(filtered_point_cloud);
}

// TODO: See if there are any better point cloud data structures.
void publishFilteredImage(const sensor_msgs::PointCloud2::ConstPtr& filtered_point_cloud) {
    filtered_image_publisher.publish(filtered_point_cloud);
}