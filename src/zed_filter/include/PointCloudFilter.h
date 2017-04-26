//
// Created by valerian on 23/04/17.
//

#ifndef PROJECT_POINTCLOUDFILTER_H
#define PROJECT_POINTCLOUDFILTER_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/point_types.h>
#include <pcl/point_types_conversion.h>

class PointCloudFilter {

public:

    typedef pcl::PointXYZRGB PointRGB;
    typedef pcl::PointXYZHSV PointHSV;
    typedef pcl::PointCloud<PointRGB> PointCloudRGB;
    typedef pcl::PointCloud<PointHSV> PointCloudHSV;

    struct FilterValues {
        float h_min, h_max, s_min, s_max, v_min, v_max;
    };
    FilterValues filter_values;

    PointCloudFilter();
    PointCloudFilter(FilterValues values);
    bool filterCloud(PointCloudRGB::Ptr& input, PointCloudRGB::Ptr& output);
};


#endif //PROJECT_POINTCLOUDFILTER_H
