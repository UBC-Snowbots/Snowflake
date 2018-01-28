/**
 * Created by: Valerian Ratu
 * Created on: 2018/01/13
 * Description: A class which pre-processed pointcloud input from the zed
 *              into the right frame and colourspace.
 *              Used in the rgb_to_hsv nodelet
 */


#ifndef PROJECT_PREPROCESSFILTER_H
#define PROJECT_PREPROCESSFILTER_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

using namespace pcl;

class ColourspaceConverter {

public:

    ColourspaceConverter();

    static PointCloud<PointXYZHSV>::Ptr Process(PointCloud<PointXYZRGB>::ConstPtr& input);
    static void PointXYZRGBAtoXYZHSV(const PointXYZRGB& in, PointXYZHSV& out);

private:

};


#endif //PROJECT_PREPROCESSFILTER_H
