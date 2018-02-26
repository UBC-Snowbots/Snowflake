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
#include <pcl/PCLPointCloud2.h>

using namespace pcl;

class ColourspaceConverter {

public:

    ColourspaceConverter();

    void filter(PointCloud<PointXYZHSV> &output);

    void setInputCloud(PointCloud<PointXYZRGB>::Ptr input);

private:
    PointCloud<PointXYZRGB>::Ptr cloud_;
    void PointXYZRGBAtoXYZHSV(const PointXYZRGB& in, PointXYZHSV& out);
};


#endif //PROJECT_PREPROCESSFILTER_H
