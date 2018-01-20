//
// Created by valerian on 13/01/18.
//

#ifndef PROJECT_PREPROCESSFILTER_H
#define PROJECT_PREPROCESSFILTER_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

using namespace pcl;

class ColourspaceConverter {

public:

    ColourspaceConverter();

    static PointCloud<PointXYZHSV>::Ptr Process(PointCloud<PointXYZRGB>::Ptr input);

};


#endif //PROJECT_PREPROCESSFILTER_H
