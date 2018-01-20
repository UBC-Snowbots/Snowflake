//
// Created by valerian on 13/01/18.
//
// rgb_to_hsv
// TFTransform

#ifndef PROJECT_PREPROCESSINGNODE_H
#define PROJECT_PREPROCESSINGNODE_H

#include <nodelet/nodelet.h>
#include <ColourspaceConverter.h>

namespace sb_pointcloud_processing {

    class RGBtoHSV : public nodelet::Nodelet {
    public:
        virtual void onInit();

        RGBtoHSV();
    };

}

#endif //PROJECT_PREPROCESSINGNODE_H
