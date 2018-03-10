/**
 * Created by: Valerian Ratu
 * Created on: 2018/01/13
 * Description: A class which converts RGB pointclouds to the HSV colourspace.
 *              Used in the rgb_to_hsv nodelet.
 */

#ifndef PROJECT_PREPROCESSFILTER_H
#define PROJECT_PREPROCESSFILTER_H

#include <pcl/PCLPointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class ColourspaceConverter {
  public:
    ColourspaceConverter();

    /**
     * Converts the pointcloud given by setInputCloud to an
     * HSV colourspace
     * @param output memory allocated to store hte output cloud
     */
    void convert(pcl::PointCloud<pcl::PointXYZHSV>& output);

    /**
     * Sets the input cloud to be processed
     * @param input
     */
    void setInputCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input);

  private:
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_;

    /**
     * Converts an RGB point to the HSV colourspace
     * Implementation taken from PCL
     * @see pcl/point_types/conversion.h
     * @param in a PointXYZRGB
     * @param out the corresponding point in HSV colourspace
     */
    void PointXYZRGBAtoXYZHSV(const pcl::PointXYZRGB& in, pcl::PointXYZHSV& out);
};

#endif // PROJECT_PREPROCESSFILTER_H
