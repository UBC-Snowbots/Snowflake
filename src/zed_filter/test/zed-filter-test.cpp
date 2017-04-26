///*
// * Created By: Robyn Castro
// * Created On: April 2, 2017
// * Description: Tests for VisionDecision
// */
//
//#include <ZedFilter.h>
//#include <sensor_msgs/PointCloud2.h>
//#include <sensor_msgs/point_cloud2_iterator.h>
//#include <gtest/gtest.h>
//
//using namespace sensor_msgs;
//
//TEST(pclTest, random) {
//
//    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_to_populate (new pcl::PointCloud<pcl::PointXYZRGB>);
//
//    pcl_to_populate->height = 1; // Unorganized.
//    pcl_to_populate->width  = 255;   // 255 data points.
//    pcl_to_populate->is_dense = false; // there may be invalid points
//    pcl_to_populate->points.resize(pcl_to_populate->width * pcl_to_populate->height);
//    for(size_t i = 0; i < pcl_to_populate->points.size(); i++) {
//        pcl_to_populate->points[i].x = i;
//        pcl_to_populate->points[i].y = i;
//        pcl_to_populate->points[i].z = i;
//        pcl_to_populate->points[i].r = i;
//        pcl_to_populate->points[i].g = i;
//        pcl_to_populate->points[i].b = i;
//    }
//
//    sensor_msgs::PointCloud2::Ptr points_msg(new sensor_msgs::PointCloud2);
//    pcl::toROSMsg(*pcl_to_populate, *points_msg);
//
//    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_point_cloud;
//    filtered_point_cloud = ZedFilter::filterImage(points_msg);
//
//    pcl::PointCloud<pcl::PointXYZRGB>::const_iterator it;
//    for(it = filtered_point_cloud->points.begin(); it != filtered_point_cloud->points.end(); it++) {
//        EXPECT_EQ(0, it->z);
//        EXPECT_EQ(255, it->r);
//        EXPECT_EQ(255, it->g);
//        EXPECT_EQ(255, it->b);
//    }
//}
//
//int main(int pclTests, char **argv) {
//    testing::InitGoogleTest(&pclTests, argv);
//    return RUN_ALL_TESTS();
//}
//
