//
// Created by valerian on 13/01/18.
//
// rgb_to_hsv
// TFTransform

#ifndef PROJECT_PREPROCESSINGNODE_H
#define PROJECT_PREPROCESSINGNODE_H

// ROS Includes
#include <nodelet/nodelet.h>
#include <ros/ros.h>

// PCL Includes
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/filters/filter.h>

#include <ColourspaceConverter.h>

using namespace pcl;
using namespace pcl_ros;

namespace sb_pointcloud_processing {

    class RGBtoHSV : public nodelet::Nodelet {
    public:
        RGBtoHSV();
    private:

        virtual void onInit();
        void callback(PointCloud<PointXYZRGB>::ConstPtr& input);
        ColourspaceConverter converter;

        ros::Publisher pub;
        ros::Subscriber sub;
    };

}

#endif //PROJECT_PREPROCESSINGNODE_H
