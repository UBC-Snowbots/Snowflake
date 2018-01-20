//
// Created by valerian on 20/01/18.
//

#ifndef PROJECT_FRAME_TRANSFORM_H
#define PROJECT_FRAME_TRANSFORM_H

#include <nodelet/nodelet.h>

namespace sb_pointcloud_processing {

    class FrameTransform : public nodelet::Nodelet {
    public:
        virtual void onInit();
        FrameTransform();
    };

}

#endif //PROJECT_FRAME_TRANSFORM_H
