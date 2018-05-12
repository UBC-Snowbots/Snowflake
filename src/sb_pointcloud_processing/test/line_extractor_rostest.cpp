/*
 * Created By: Min Gyo Kim
 * Created On: January 20, 2018
 * Description: Ros tests for Line Extractor Node
 */

#include "./TestUtils.h"
#include <gtest/gtest.h>
#include <mapping_igvc/LineObstacle.h>
#include <pcl_conversions/pcl_conversions.h>
#include <std_msgs/Float32.h>

/**
 * This is the helper class which will publish and subscribe messages which will
 * test the node being instantiated
 * It contains at the minimum:
 *      publisher - publishes the input to the node
 *      subscriber - publishes the output of the node
 *      callback function - the callback function which corresponds to the
 * subscriber
 *      getter function - to provide a way for gtest to check for equality of
 * the message recieved
 */
class LineExtractorRosTest : public testing::Test {
  protected:
    virtual void SetUp() {
        test_publisher =
        nh_.advertise<sensor_msgs::PointCloud2>("/input_pointcloud", 1);
        test_subscriber =
        nh_.subscribe("/line_extractor_node/output_line_obstacle",
                      1,
                      &LineExtractorRosTest::callback,
                      this);

        // Let the publishers and subscribers set itself up timely
        ros::Rate loop_rate(1);
        loop_rate.sleep();
    }

    ros::NodeHandle nh_;
    mapping_igvc::LineObstacle lineObstacle;
    ros::Publisher test_publisher;
    ros::Subscriber test_subscriber;

  public:
    void callback(const mapping_igvc::LineObstacle& line) {
        lineObstacle = line;
    }
};

TEST_F(LineExtractorRosTest, TestTwoNonLinearLinesWithNoise) {
    float x_min   = 0;
    float x_max   = 99;
    float x_delta = 1;

    // coefficients is the same as the one in LineObstacle message
    std::vector<float> coefficients = {1000, 7, -0.7, 0.007};
    LineExtractor::TestUtils::LineArgs args(
    coefficients, x_min, x_max, x_delta);

    float max_noise_x = 1;
    float max_noise_y = 1;

    // Generate a single PointCloud with noise
    pcl::PointCloud<pcl::PointXYZ> pcl;
    LineExtractor::TestUtils::addLineToPointCloud(
    args, pcl, max_noise_x, max_noise_y);

    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(pcl, msg);

    test_publisher.publish(msg);
    ros::Rate loop_rate(1);

    // Wait for the message to get passed around
    loop_rate.sleep();

    // spinOnce allows ros to actually process your callbacks
    // for the curious:
    // http://answers.ros.org/question/11887/significance-of-rosspinonce/
    ros::spinOnce();

    ASSERT_EQ(lineObstacle.coefficients.size(), coefficients.size());

    for (unsigned int i = 0; i < lineObstacle.coefficients.size(); i++) {
        float tol;

        // logic to allow more tolerance for y-intercept
        if (i) {
            tol = 5;
        } else {
            tol = 10;
        }

        EXPECT_NEAR(lineObstacle.coefficients[i], coefficients[i], tol);
    }

    EXPECT_NEAR(lineObstacle.x_min, x_min, 1);
    EXPECT_NEAR(lineObstacle.x_max, x_max, 1);

    float true_min, true_max;
    LineExtractor::TestUtils::getMinAndMaxOfPointCloud(true_min, true_max, pcl);

    EXPECT_FLOAT_EQ(lineObstacle.x_min, true_min);
    EXPECT_FLOAT_EQ(lineObstacle.x_max, true_max);
}

int main(int argc, char** argv) {
    // !! Don't forget to initialize ROS, since this is a test within the ros
    // framework !!
    ros::init(argc, argv, "line_extractor_rostest");
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
