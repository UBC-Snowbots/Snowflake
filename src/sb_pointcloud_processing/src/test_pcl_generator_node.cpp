//
// Created by min on 10/03/18.
//

#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include "../test/TestUtils.h"

sensor_msgs::PointCloud2 generatePclMessage();

int main(int argc, char** argv) {
    ros::init(argc, argv, "test_pcl_generator_node");
    ros::NodeHandle nh;

    ros::Rate loop_rate = 1;
    ros::Publisher publisher = nh.advertise<sensor_msgs::PointCloud2>("input_pointcloud", 1);

    sensor_msgs::PointCloud2 msg_to_publish = generatePclMessage();
    msg_to_publish.header.frame_id = "line_extractor_test";

    while(ros::ok()) {
        publisher.publish(msg_to_publish);

        ros::spinOnce();
        loop_rate.sleep();
    }
}

sensor_msgs::PointCloud2 generatePclMessage() {
    float x_min   = 0;
    float x_max   = 99;
    float x_delta = 1;

    // coefficients is the same as the one in LineObstacle message
    std::vector<float> coefficients = {1000, 7, -0.7, 0.007};
    LineExtractor::TestUtils::LineArgs args(
            coefficients, x_min, x_max, x_delta);

    float max_noise_x = 1;
    float max_noise_y = 1;

    // Generate a single PointCloud with noise
    pcl::PointCloud<pcl::PointXYZ> pcl;
    LineExtractor::TestUtils::addLineToPointCloud(
            args, pcl, max_noise_x, max_noise_y);

    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(pcl, msg);

    return msg;
}