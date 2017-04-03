/*
 * Created By: Robyn Castro
 * Created On: April 2, 2017
 * Description: Tests for VisionDecision
 */

#include <ZedFilter.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

using namespace sensor_msgs;

TEST(pclTest, random){
    PointCloud2Ptr points_msg = boost::make_shared<PointCloud2>();
    points_msg->height = 1; // Unorganized.
    points_msg->width  = 255;   // 255 data points.
    points_msg->is_bigendian = false;
    points_msg->is_dense = false; // there may be invalid points

    sensor_msgs::PointCloud2Modifier pcd_modifier(*points_msg);
    // this call also resizes the data structure according to the given width, height and fields
    pcd_modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");

    sensor_msgs::PointCloud2Iterator<float> iter_x(*points_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(*points_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(*points_msg, "z");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(*points_msg, "r");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(*points_msg, "g");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(*points_msg, "b");

    int i;
    for (i = 0;; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++iter_r, ++iter_g, ++iter_b, ++i)
    {
        // TODO fill in x, y, z, r, g, b local variables
        *iter_x = i;
        *iter_y = i;
        *iter_z = i;
        *iter_r = i;
        *iter_g = i;
        *iter_b = i;
    }

}

int main(int pclTests, char **argv) {
    testing::InitGoogleTest(&pclTests, argv);
    return RUN_ALL_TESTS();
}

