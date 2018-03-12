//
// Created by min on 10/03/18.
//

#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include "../test/TestUtils.h"

std::vector<float> first_line;
std::vector<float> second_line;

float x_min;
float x_max;
float x_delta;
float max_noise_x;
float max_noise_y;

sensor_msgs::PointCloud2 generatePclMessage();

int main(int argc, char** argv) {
    ros::init(argc, argv, "test_pcl_generator_node");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    ros::Rate loop_rate = 1;
    ros::Publisher publisher = nh.advertise<sensor_msgs::PointCloud2>("input_pointcloud", 1);

    std::string first_line_param = "first_line";
    std::vector<float> default_first_line = {50, 0, -0.01};
    private_nh.param(first_line_param, first_line, default_first_line);

    std::string second_line_param = "second_line";
    std::vector<float> default_second_line = {0, 0, -0.01};
    private_nh.param(second_line_param, second_line, default_second_line);

    std::string x_min_param = "x_min";
    float default_x_min = -50;
    private_nh.param(x_min_param, x_min, default_x_min);

    std::string x_max_param = "x_max";
    float default_x_max = 50;
    private_nh.param(x_max_param, x_max, default_x_max);

    std::string x_delta_param = "x_delta";
    float default_x_delta = 0.5;
    private_nh.param(x_delta_param, x_delta, default_x_delta);

    std::string max_noise_x_param = "max_noise_x";
    float default_max_noise_x = 5;
    private_nh.param(max_noise_x_param, max_noise_x, default_max_noise_x);

    std::string max_noise_y_param = "max_noise_y";
    float default_max_noise_y = 5;
    private_nh.param(max_noise_y_param, max_noise_y, default_max_noise_y);

    sensor_msgs::PointCloud2 msg_to_publish = generatePclMessage();
    msg_to_publish.header.frame_id = "line_extractor_test";

    while(ros::ok()) {
        publisher.publish(msg_to_publish);

        ros::spinOnce();
        loop_rate.sleep();
    }
}

sensor_msgs::PointCloud2 generatePclMessage() {
    // coefficients is the same as the one in LineObstacle message
    LineExtractor::TestUtils::LineArgs args(
            first_line, x_min, x_max, x_delta);

    // Generate a single PointCloud with noise
    pcl::PointCloud<pcl::PointXYZ> pcl;
    LineExtractor::TestUtils::addLineToPointCloud(
            args, pcl, max_noise_x, max_noise_y);

    // Add second line to the pointcloud
    args.coefficients = second_line;
    LineExtractor::TestUtils::addLineToPointCloud(
            args, pcl, max_noise_x, max_noise_y);

    // convert pointcloud to sensor msgs
    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(pcl, msg);

    return msg;
}