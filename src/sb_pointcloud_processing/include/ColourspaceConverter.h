/**
 * Created by: Valerian Ratu
 * Created on: 2018/01/13
 * Description: A class which converts RGB pointclouds to the HSV colourspace.
 *              Used in the rgb_to_hsv nodelet.
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

    /**
     * Converts the pointcloud given by setInputCloud to an
     * HSV colourspace
     * @param output memory allocated to store hte output cloud
     */
    void filter(PointCloud<PointXYZHSV> &output);

    /**
     * Sets the input cloud to be processed
     * @param input
     */
    void setInputCloud(PointCloud<PointXYZRGB>::Ptr input);

private:
    PointCloud<PointXYZRGB>::Ptr cloud_;

    /**
     * Converts an RGB point to the HSV colourspace
     * @param in a PointXYZRGB
     * @param out the corresponding point in HSV colourspace
     */
    void PointXYZRGBAtoXYZHSV(const PointXYZRGB& in, PointXYZHSV& out);
};


#endif //PROJECT_PREPROCESSFILTER_H
