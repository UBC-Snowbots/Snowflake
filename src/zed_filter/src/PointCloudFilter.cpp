//
// Created by valerian on 23/04/17.
//

#include "../include/PointCloudFilter.h"

PointCloudFilter::PointCloudFilter(){

    filter_values.h_min = 0;
    filter_values.h_max = 360;
    filter_values.s_min = 0;
    filter_values.s_max = 1;
    filter_values.v_min = 0;
    filter_values.v_max = 1;
}

PointCloudFilter::PointCloudFilter(FilterValues values){
    filter_values = values;
}

PointCloudFilter::PointCloudFilter(float h_min, float h_max, float s_min, float s_max, float v_min, float v_max){
    filter_values.h_min = h_min;
    filter_values.h_max = h_max;
    filter_values.s_min = s_min;
    filter_values.s_max = s_max;
    filter_values.v_min = v_min;
    filter_values.v_max = v_max;
}


bool PointCloudFilter::filterCloud(PointCloudRGB::Ptr& input, PointCloudRGB::Ptr& output){
    PointCloudHSV::Ptr point_cloud_HSV(new PointCloudHSV);
    pcl::PointCloudXYZRGBtoXYZHSV(*input, *point_cloud_HSV);

    // Filter out non-white points from the point cloud
    PointCloudHSV::Ptr filtered_point_cloud(new PointCloudHSV);
    // Create the filtering object;
    pcl::PassThrough<PointHSV> pass;
    pass.setInputCloud(point_cloud_HSV);

    // Filter Hue
    pass.setFilterFieldName("h");
    pass.setFilterLimits(filter_values.h_min, filter_values.h_max);
    pass.filter(*filtered_point_cloud);
    // Filter Saturation
    pass.setInputCloud(filtered_point_cloud);
    pass.setFilterFieldName("s");
    pass.setFilterLimits(filter_values.s_min, filter_values.s_max);
    pass.filter(*filtered_point_cloud);
    // Filter Value
    pass.setInputCloud(filtered_point_cloud);
    pass.setFilterFieldName("v");
    pass.setFilterLimits(filter_values.v_min, filter_values.v_max);
    pass.filter(*filtered_point_cloud);

    // Clear the output point cloud and populates it
    output->points.clear();
    PointCloudHSV::const_iterator it;
    for(it = filtered_point_cloud->points.begin(); it != filtered_point_cloud->points.end(); it++) {
        PointRGB current_point;

        // Map all points to z = 0 plane.
        current_point.z = 0;

        // All remaining points are mapped to white.
        current_point.r = 1;
        current_point.g = 255;
        current_point.b = 255;

        // Retrieve the x and z values
        current_point.x = it->x;
        current_point.y = it->y;

        output->points.push_back(current_point);
    }
    return true;
}