/*
 * Created By: Valerian Ratu
 * Created On: April 25 2017
 * Description: Tests for PointCloudFilter
 */


#include <PointCloudFilter.h>
#include <gtest/gtest.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

typedef pcl::PointXYZRGB PointRGB;
typedef pcl::PointXYZHSV PointHSV;
typedef pcl::PointCloud<PointRGB> PointCloudRGB;
typedef pcl::PointCloud<PointHSV> PointCloudHSV;

TEST(PointCloudFilterTest, output_black_and_white){
    PointCloudRGB::Ptr pcl_to_populate(new PointCloudRGB);
    pcl_to_populate->height = 1; // Unorganized.
    pcl_to_populate->width  = 255;   // 255 data points.
    pcl_to_populate->is_dense = false; // there may be invalid points
    pcl_to_populate->points.resize(pcl_to_populate->width * pcl_to_populate->height);
    for(size_t i = 0; i < pcl_to_populate->points.size(); i++) {
        pcl_to_populate->points[i].x = i;
        pcl_to_populate->points[i].y = i;
        pcl_to_populate->points[i].z = i;
        pcl_to_populate->points[i].r = i;
        pcl_to_populate->points[i].g = i;
        pcl_to_populate->points[i].b = i;
    }
    PointCloudFilter filter = PointCloudFilter();
    PointCloudRGB::Ptr filtered_point_cloud(new PointCloudRGB);
    filter.filterCloud(pcl_to_populate, filtered_point_cloud);

    pcl::PointCloud<pcl::PointXYZRGB>::const_iterator it;
    for(it = filtered_point_cloud->points.begin(); it != filtered_point_cloud->points.end(); it++) {
        EXPECT_EQ(0, it->z);
        EXPECT_EQ(255, it->r);
        EXPECT_EQ(255, it->g);
        EXPECT_EQ(255, it->b);
    }
}

int main(int pointCloudTests, char** argv) {
    testing::InitGoogleTest(&pointCloudTests, argv);
    return RUN_ALL_TESTS();
}