//
// Created by valerian on 23/04/17.
//

#include "../include/PointCloudFilter.h"

PointCloudFilter::PointCloudFilter(){
    setFilter(0, 360, 0, 1, 0 ,1);
}

PointCloudFilter::PointCloudFilter(FilterValues values){
    setFilter(values);
}

PointCloudFilter::PointCloudFilter(float h_min, float h_max, float s_min, float s_max, float v_min, float v_max){
    setFilter(h_min, h_max, s_min, s_max, v_min, v_max);
}

void PointCloudFilter::setFilter(float h_min, float h_max, float s_min, float s_max, float v_min, float v_max) {
    FilterValues values;
    values.h_min = h_min;
    values.h_max = h_max;
    values.s_min = s_min;
    values.s_max = s_max;
    values.v_min = v_min;
    values.v_max = v_max;
    setFilter(values);
}

void PointCloudFilter::setFilter(FilterValues values) {
    filter_values = values;
}

void PointCloudFilter::PointCloudRGBtoPointCloudHSV(PointCloudRGB::Ptr in,
                          PointCloudHSV::Ptr out)
{
    out->width   = in->width;
    out->height  = in->height;

    PointCloudRGB::iterator it;
    for (it = in->points.begin(); it != in->points.end(); it++){
        PointHSV p;
        PointRGBtoPointHSV(*it, p);
        out->points.push_back(p);
    }
}

void PointCloudFilter::PointRGBtoPointHSV(PointRGB& in, PointHSV& out){
    out.x = in.x; out.y = in.y; out.z = in.z;

    const unsigned char max = std::max (in.r, std::max (in.g, in.b));
    const unsigned char min = std::min (in.r, std::min (in.g, in.b));

    out.v = static_cast <float> (max) / 255.f;

    if (max == 0) // division by zero
    {
        out.s = 0.f;
        out.h = 0.f; // h = -1.f;
        return;
    }

    const float diff = static_cast <float> (max - min);
    out.s = diff / static_cast <float> (max);

    if (min == max) // diff == 0 -> division by zero
    {
        out.h = 0;
        return;
    }

    if      (max == in.r) out.h = 60.f * (      static_cast <float> (in.g - in.b) / diff);
    else if (max == in.g) out.h = 60.f * (2.f + static_cast <float> (in.b - in.r) / diff);
    else                  out.h = 60.f * (4.f + static_cast <float> (in.r - in.g) / diff); // max == b

    if (out.h < 0.f) out.h += 360.f;
}


bool PointCloudFilter::filterCloud(PointCloudRGB::Ptr input, PointCloudRGB::Ptr output){
    PointCloudHSV::Ptr point_cloud_HSV(new PointCloudHSV);
    PointCloudRGBtoPointCloudHSV(input, point_cloud_HSV);
    ROS_INFO("Incoming points: %u", input->points.size());
    ROS_INFO("Incoming points: %u", point_cloud_HSV->points.size());

    // Filter out non-white points from the point cloud
    PointCloudHSV::Ptr filtered_point_cloud(new PointCloudHSV);
    // Create the filtering object;
    pcl::PassThrough<PointHSV> pass;
    pass.setInputCloud(point_cloud_HSV);
//
//    // Filter Hue
    pass.setFilterFieldName("h");
    pass.setFilterLimits(filter_values.h_min, filter_values.h_max);
    pass.filter(*filtered_point_cloud);
    ROS_INFO("Points after h: %u", filtered_point_cloud->points.size());
//    // Filter Saturation
    pass.setInputCloud(filtered_point_cloud);
    pass.setFilterFieldName("s");
    pass.setFilterLimits(filter_values.s_min, filter_values.s_max);
    pass.filter(*filtered_point_cloud);
    ROS_INFO("Points after s: %u", filtered_point_cloud->points.size());
//
//    // Filter Value
    pass.setInputCloud(filtered_point_cloud);
    pass.setFilterFieldName("v");
    pass.setFilterLimits(filter_values.v_min, filter_values.v_max);
    pass.filter(*filtered_point_cloud);
    ROS_INFO("Points after v: %u", filtered_point_cloud->points.size());

    // Clear the output point cloud and populates it
    output->points.clear();
    PointCloudHSV::const_iterator it;
//    output->height = filtered_point_cloud->height;
//    output->width = filtered_point_cloud->width;
    for(it = filtered_point_cloud->points.begin(); it != filtered_point_cloud->points.end(); it++) {
        PointRGB current_point;

        // Map all points to z = 0 plane.
        current_point.z = 0;

        // All remaining points are mapped to white.
        current_point.r = 255;
        current_point.g = 255;
        current_point.b = 255;

        // Retrieve the x and z values
        current_point.x = it->x;
        current_point.y = it->y;

        output->points.push_back(current_point);
    }
    return true;
}