/**
 * Created by: Valerian Ratu
 * Created on: 2018/01/13
 * Description: A node which pre-processed pointcloud input from the zed
 *              into the right frame and colourspace
 */

#include <ColourspaceConverter.h>
#include <pcl/point_types_conversion.h>

ColourspaceConverter::ColourspaceConverter()
{

}

PointCloud<PointXYZHSV>::Ptr ColourspaceConverter::Process(PointCloud<PointXYZRGB>::Ptr input) {
    PointCloud<PointXYZHSV>::Ptr output(new PointCloud<PointXYZHSV>());
    PointCloudXYZRGBtoXYZHSV(*input, *output);
}
