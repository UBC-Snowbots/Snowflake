/*
 * Created By: Min Gyo Kim
 * Created On: January 20, 2018
 * Description: Ros tests for Line Extractor Node
 */

#include "./TestUtils.h"
#include <gtest/gtest.h>
#include <pcl_conversions/pcl_conversions.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Point.h>

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
class BallExtractorRosTest : public testing::Test {
  protected:
    virtual void SetUp() {
        test_publisher =
        nh_.advertise<sensor_msgs::PointCloud2>("/input_pointcloud", 1);
        test_subscriber =
        nh_.subscribe("/ball_extractor_node/output_ball_obstacle",
                      1,
                      &BallExtractorRosTest::callback,
                      this);

        // Let the publishers and subscribers set itself up timely
        ros::Rate loop_rate(1);
        loop_rate.sleep();
    }

    ros::NodeHandle nh_;
    geometry_msgs::Point ballObstacle;
    ros::Publisher test_publisher;
    ros::Subscriber test_subscriber;

  public:
    void callback(const geometry_msgs::Point& ball) {
        ballObstacle = ball;
    }
};

TEST_F(BallExtractorRosTest, TestTwoNonLinearLinesWithNoise) {\
    pcl::PointCloud<pcl::PointXYZ> pcl;

    // radius
    float r = 0.03;
    float r_pow_2 = pow(r, 2);
    float x = 5.92;
    float y_center = 99;
    float z_center = -50;
    float y_delta = 0.0005;
    std::vector<float> ys;
    std::vector<float> zs;

    for (float y = -r; y <= r; y += y_delta) {
        float y_pow_2 = pow(y, 2);

        float z1 = sqrt(r_pow_2 - y_pow_2);
        pcl.push_back(pcl::PointXYZ(x, y + y_center, z1 + z_center));
        ys.push_back(y + y_center);
        zs.push_back(z1 + z_center);

        float z2 = -sqrt(r_pow_2 - y_pow_2);
        pcl.push_back(pcl::PointXYZ(x, y + y_center, z2 + z_center));
        ys.push_back(y + y_center);
        zs.push_back(z2 + z_center);
    }

    // add noise
    pcl.push_back(pcl::PointXYZ(429, 13843, -9358));
    pcl.push_back(pcl::PointXYZ(48297, -438973, 8794));

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

    EXPECT_FLOAT_EQ(x, ballObstacle.x);
    float y_min = *std::min_element(ys.begin(), ys.end());
    float y_max = *std::max_element(ys.begin(), ys.end());
    EXPECT_FLOAT_EQ((y_min + y_max)/2, ballObstacle.y);

    float z_min = *std::min_element(zs.begin(), zs.end());
    float z_max = *std::max_element(zs.begin(), zs.end());
    EXPECT_FLOAT_EQ((z_min + z_max)/2, ballObstacle.z);
}

int main(int argc, char** argv) {
    // !! Don't forget to initialize ROS, since this is a test within the ros
    // framework !!
    ros::init(argc, argv, "ball_extractor_rostest");
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
