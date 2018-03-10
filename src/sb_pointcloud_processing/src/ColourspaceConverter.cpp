/**
 * Created by: Valerian Ratu
 * Created on: 2018/01/13
 * Description: A class which converts RGB pointclouds to the HSV colourspace.
 */

#include <ColourspaceConverter.h>

using namespace pcl;

ColourspaceConverter::ColourspaceConverter() {}

void ColourspaceConverter::setInputCloud(PointCloud<PointXYZRGB>::Ptr input) {
    cloud_ = input;
}

void ColourspaceConverter::convert(PointCloud<PointXYZHSV>& output) {
    output.width  = cloud_->width;
    output.height = cloud_->height;
    output.header = cloud_->header;
    for (size_t i = 0; i < cloud_->size(); i++) {
        PointXYZHSV p;
        ColourspaceConverter::PointXYZRGBAtoXYZHSV(cloud_->points[i], p);
        output.points.push_back(p);
    }
}

void ColourspaceConverter::PointXYZRGBAtoXYZHSV(const PointXYZRGB& in,
                                                PointXYZHSV& out) {
    out.x = in.x;
    out.y = in.y;
    out.z = in.z;

    const unsigned char max = std::max(in.r, std::max(in.g, in.b));
    const unsigned char min = std::min(in.r, std::min(in.g, in.b));

    out.v = static_cast<float>(max) / 255.f;

    if (max == 0) // rgb values of zero maps to hsv 0, 0, 0
    {
        out.s = 0.f;
        out.h = 0.f;
        return;
    }

    const float diff = static_cast<float>(max - min);
    out.s            = diff / static_cast<float>(max);

    if (min == max) // diff == 0 -> division by zero, set h to zero
    {
        out.h = 0;
        return;
    }

    if (max == in.r)
        out.h = 60.f * (static_cast<float>(in.g - in.b) / diff);
    else if (max == in.g)
        out.h = 60.f * (2.f + static_cast<float>(in.b - in.r) / diff);
    else
        out.h =
        60.f * (4.f + static_cast<float>(in.r - in.g) / diff); // max == b

    if (out.h < 0.f) out.h += 360.f;
}
