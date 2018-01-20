//
// Created by valerian on 20/01/18.
//

#include <frame_transform.h>
#include <pluginlib/class_list_macros.h>

using namespace sb_pointcloud_processing;

PLUGINLIB_EXPORT_CLASS(FrameTransform, nodelet::Nodelet)

void FrameTransform::onInit()
{
    NODELET_DEBUG("Initializing nodelet...");
}

FrameTransform::FrameTransform(){}
