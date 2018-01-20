//
// Created by valerian on 13/01/18.
//

#include <rgb_to_hsv.h>

#include <pluginlib/class_list_macros.h>

using namespace sb_pointcloud_processing;

PLUGINLIB_EXPORT_CLASS(RGBtoHSV, nodelet::Nodelet)

void RGBtoHSV::onInit()
{
    NODELET_DEBUG("Initializing nodelet...");
}

RGBtoHSV::RGBtoHSV(){}

