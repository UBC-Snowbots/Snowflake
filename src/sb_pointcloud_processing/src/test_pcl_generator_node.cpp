/*
 * Created by: Min Gyo Kim
 * Created On: March 11th 2018
 * Description: Generates a point cloud with two lines and an optional outlier line
 *              and publishes it to "/input/pointcloud"
 */

#include "../test/TestUtils.h"
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sb_utils.h>

std::vector<float> first_line;
std::vector<float> second_line;

float x_min;
float x_max;
float x_delta;
float max_noise_x;
float max_noise_y;

float outlier_x_delta;
std::vector<float> outlier_line;

sensor_msgs::PointCloud2 generatePclMessage(bool include_outlier);

int main(int argc, char** argv) {
    ros::init(argc, argv, "test_pcl_generator_node");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    ros::Rate loop_rate = 0.75;
    ros::Publisher publisher =
    nh.advertise<sensor_msgs::PointCloud2>("input_pointcloud", 1);

    std::string first_line_param          = "first_line";
    std::vector<float> default_first_line = {50, 0, -0.01};
    private_nh.param(first_line_param, first_line, default_first_line);

    std::string second_line_param          = "second_line";
    std::vector<float> default_second_line = {0, 0, -0.01};
    private_nh.param(second_line_param, second_line, default_second_line);

    std::string x_min_param = "x_min";
    float default_x_min     = -50;
    SB_getParam(private_nh, x_min_param, x_min, default_x_min);

    std::string x_max_param = "x_max";
    float default_x_max     = 50;
    SB_getParam(private_nh, x_max_param, x_max, default_x_max);

    std::string x_delta_param = "x_delta";
    float default_x_delta     = 0.5;
    SB_getParam(private_nh, x_delta_param, x_delta, default_x_delta);

    std::string max_noise_x_param = "max_noise_x";
    float default_max_noise_x     = 5;
    SB_getParam(private_nh, max_noise_x_param, max_noise_x, default_max_noise_x);

    std::string max_noise_y_param = "max_noise_y";
    float default_max_noise_y     = 5;
    SB_getParam(private_nh, max_noise_y_param, max_noise_y, default_max_noise_y);

    std::string outlier_param = "outlier";
    bool default_outlier = false;
    bool outlier;
    SB_getParam(private_nh, outlier_param, outlier, default_outlier);

    if (outlier) {
        std::string outlier_line_param = "outlier_line";
        std::vector<float> default_outlier_line = {0};
        private_nh.param(outlier_line_param, outlier_line, default_outlier_line);

        std::string outlier_x_delta_param = "outlier_x_delta";
        float default_outlier_x_delta     = 1;
        private_nh.param(outlier_x_delta_param, outlier_x_delta, default_outlier_x_delta);
    }

    sensor_msgs::PointCloud2 msg_to_publish = generatePclMessage(outlier);
    msg_to_publish.header.frame_id          = "line_extractor_test";

    while (ros::ok()) {
        publisher.publish(msg_to_publish);

        ros::spinOnce();
        loop_rate.sleep();
    }
}

sensor_msgs::PointCloud2 generatePclMessage(bool include_outlier) {
    // coefficients is the same as the one in LineObstacle message
    LineExtractor::TestUtils::LineArgs args(first_line, x_min, x_max, x_delta);

    // Generate a single PointCloud with noise
    pcl::PointCloud<pcl::PointXYZ> pcl;
    LineExtractor::TestUtils::addLineToPointCloud(
    args, pcl, max_noise_x, max_noise_y);

    // Add second line to the pointcloud
    args.coefficients = second_line;
    LineExtractor::TestUtils::addLineToPointCloud(
    args, pcl, max_noise_x, max_noise_y);

    // Add outlier if wanted
    if (include_outlier) {
        args.coefficients = outlier_line;
        args.x_delta = outlier_x_delta;

        LineExtractor::TestUtils::addLineToPointCloud(
                args, pcl, max_noise_x, max_noise_y);
    }

    // convert pointcloud to sensor msgs
    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(pcl, msg);

    return msg;
}