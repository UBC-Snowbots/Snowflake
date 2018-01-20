//
// Created by valerian on 13/01/18.
//

#include <PreprocessingNode.h>

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(PreprocessingNode, nodelet::Nodelet)

void PreprocessingNode::onInit()
{
    NODELET_DEBUG("Initializing nodelet...");
}

