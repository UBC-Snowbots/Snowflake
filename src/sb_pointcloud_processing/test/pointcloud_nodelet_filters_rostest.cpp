#include "rgb_to_hsv.h"
#include <gtest/gtest.h>
#include <ros/ros.h>

class PointCloudProcessingTest : public testing::Test {
  protected:
    virtual void SetUp() {
        test_publisher =
        nh_.advertise<pcl::PointCloud<pcl::PointXYZRGB>>("test_sub", 1);
        test_subscriber = nh_.subscribe(
        "/z/output", 1, &PointCloudProcessingTest::callback, this);

        // Let the publishers and subscribers set itself up timely
        ros::Rate loop_rate(1);
        loop_rate.sleep();
    }

    ros::NodeHandle nh_;
    pcl::PointCloud<pcl::PointXYZHSV> message_output;
    ros::Publisher test_publisher;
    ros::Subscriber test_subscriber;

  public:
    /**
     * Helper function to compare floats. Defines the equality to be
     * based on FLT_EPSILON: the value of the difference between one and
     * the smallest value smallest from one in float representation.
     * @param a
     * @param b
     * @return True if a and b very close in float representation,
     *           false otherwise.
     */
    bool almostEquals(float a, float b) { return fabs(a - b) < FLT_EPSILON; }

    void callback(const pcl::PointCloud<pcl::PointXYZHSV>::Ptr msg) {
        message_output = *msg;
    }
};

/*
 * Tests whether pointclouds in one frame are transformed
 * to the output frame properly.
 * @see the static transform publisher in the rostest
 *
 * The initial cloud should be:
 *
 *      z
 *      ^
 *      |    * (1,3)
 *      |
 *      |
 *      | (0,0)
 *      * -----------> y
 *
 * And after the transform (point kept in the same place in figure):
 * 2 units in the y direction followed by a 45 deg positive rotation on the
 * x-axis
 *
 *                ^ y
 *                |
 *           * (3,1)
 *                |
 *                |
 * z      (0,2)   |
 * <----*---------
 *
 *
 */
TEST_F(PointCloudProcessingTest, transformsCloud) {
    ros::Duration(5).sleep();

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr input(
    new pcl::PointCloud<pcl::PointXYZRGB>());
    input->height = 1;
    input->width  = 2;
    input->points.resize(input->height * input->width);

    pcl::PointXYZRGB p1;
    p1.x = 0;
    p1.y = 0;
    p1.z = 0;
    p1.r = 45;
    p1.g = 174;
    p1.b = 45;

    input->points[0] = p1;

    pcl::PointXYZRGB p2;
    p2.x = 0;
    p2.y = 1;
    p2.z = 3;
    p2.r = 45;
    p2.g = 174;
    p2.b = 45;

    input->points[1] = p2;

    input->header.frame_id = "input";

    test_publisher.publish(input);
    ros::Rate loop_rate(1);
    loop_rate.sleep();
    ros::spinOnce();

    EXPECT_EQ(2, message_output.points.size());

    // Expect points to be converted from (y, z)
    //    - x = 0 for all points
    // Not quite exact due to floating point maths
    // (0,0) -> (0,2)
    // (1,3) -> (3,1)
    int num_comparisons = 0;
    for (auto& point : message_output.points) {
        // std::cerr << "Point: y = " << point.y << " z = " << point.z <<
        // std::endl;
        if (almostEquals(point.y, 0.f)) {
            ASSERT_NEAR(2, point.z, 0.001);
            num_comparisons++;
        } else if (almostEquals(point.y, 3.f)) {
            ASSERT_NEAR(1, point.z, 0.001);
            num_comparisons++;
        }
    }
    EXPECT_EQ(2, num_comparisons);
}

/*
 * Filters for points of interests which are points within
 * a certain boundary with a certain colour (GREEN)
 */
TEST_F(PointCloudProcessingTest, filtersPointsOfInterest) {
    ros::Duration(5).sleep();

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr input(
    new pcl::PointCloud<pcl::PointXYZRGB>());
    input->height = 1;
    input->width  = 9;
    input->points.resize(input->height * input->width);
    int point_ind = 0;
    for (int i = 1; i <= 5; i++) {
        pcl::PointXYZRGB p;
        p.x = 0;
        p.y = 1;
        p.z = i;
        if (i > 1 && i < 5) {
            // These are the points of interest
            p.r = 45;
            p.g = 174;
            p.b = 45;
        }
        input->points[point_ind++] = p;
    }
    for (int i = 2; i <= 5; i++) {
        pcl::PointXYZRGB p;
        p.x = 0;
        p.y = i;
        p.z = 1;
        if (i > 1 && i < 5) {
            // Should be filtered out by the z filter
            p.r = 45;
            p.g = 174;
            p.b = 45;
        }
        input->points[point_ind++] = p;
    }

    input->header.frame_id = "input";

    test_publisher.publish(input);
    ros::Rate loop_rate(1);
    loop_rate.sleep();
    ros::spinOnce();

    EXPECT_EQ(3, message_output.points.size());
    int num_comparisons = 0;
    for (auto& point : message_output.points) {
        // std::cerr << "Point: y = " << point.y << " z = " << point.z <<
        // std::endl;
        if (almostEquals(point.y, 2.f)) {
            ASSERT_NEAR(1, point.z, 0.001);
            num_comparisons++;
        } else if (almostEquals(point.y, 3.f)) {
            ASSERT_NEAR(1, point.z, 0.001);
            num_comparisons++;
        } else if (almostEquals(point.y, 4.f)) {
            ASSERT_NEAR(1, point.z, 0.001);
            num_comparisons++;
        }
    }
    EXPECT_EQ(3, num_comparisons);
}

int main(int argc, char** argv) {
    // !! Don't forget to initialize ROS, since this is a test within the ros
    // framework !!
    ros::init(argc, argv, "pointcloud_filters_rostest");
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}