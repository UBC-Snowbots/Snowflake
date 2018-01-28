//
// Created by valerian on 13/01/18.
//

#include <rgb_to_hsv.h>

#include <pluginlib/class_list_macros.h>

using namespace sb_pointcloud_processing;

PLUGINLIB_EXPORT_CLASS(RGBtoHSV, nodelet::Nodelet)

RGBtoHSV::RGBtoHSV(){}

void RGBtoHSV::onInit()
{
    NODELET_DEBUG("Initializing nodelet...");
    ros::NodeHandle& private_nh = getPrivateNodeHandle();

    pub = private_nh.advertise<PointCloud<PointXYZRGB>::Ptr >("rgb_to_hsv_out", 1);
    sub = private_nh.subscribe("rgb_to_hsv_in", 1, &RGBtoHSV::callback, this);
}

void RGBtoHSV::callback(PointCloud<PointXYZRGB>::ConstPtr& input){
    PointCloud<PointXYZHSV>::Ptr output;
    output = converter.Process(input);
    pub.publish(output);
}

