/*
 * Created By: Robyn Castro
 * Created On: September 22, 2016
 * Description: Tests for VisionDecision
 */

#include <VisionDecision.h>
#include <gtest/gtest.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

using namespace cv;
using namespace std;

sensor_msgs::Image convertToSensorMsg(Mat cvMatImage);

TEST(imageTest, moveAwayFromLineAndTurnRight){
    string filename = "imageTests/testStraightImage.jpg";
    Mat image = imread(filename, CV_LOAD_IMAGE_GRAYSCALE);

    sensor_msgs::Image sensorMsg = convertToSensorMsg(image);

    sensor_msgs::ImageConstPtr testImageScan(new sensor_msgs::Image(sensorMsg));

    EXPECT_NEAR(45, VisionDecision::getDesiredAngle(testImageScan->height / 8.0, testImageScan, 0.25), 10);
}

TEST(imageTest, angleLeft){
    string filename = "imageTests/testLeftImage.jpg";
    Mat image = imread(filename, CV_LOAD_IMAGE_GRAYSCALE);

    sensor_msgs::Image sensorMsg = convertToSensorMsg(image);

    sensor_msgs::ImageConstPtr testImageScan(new sensor_msgs::Image(sensorMsg));

    EXPECT_NEAR(-30, VisionDecision::getDesiredAngle(testImageScan->height / 8.0, testImageScan, 0.25), 20);
}

TEST(imageTest, angleRight){
    string filename = "imageTests/testNoisyRightImage.jpg";
    Mat image = imread(filename, CV_LOAD_IMAGE_GRAYSCALE);

    sensor_msgs::Image sensorMsg = convertToSensorMsg(image);

    sensor_msgs::ImageConstPtr testImageScan(new sensor_msgs::Image(sensorMsg));

    EXPECT_NEAR(30, VisionDecision::getDesiredAngle(testImageScan->height / 8.0, testImageScan, 0.25), 20);
}

TEST(imageTest, moveAwayFromLineAndTurnRightTwo){
    string filename = "imageTests/testVeryNoisyStraightImage.jpg";
    Mat image = imread(filename, CV_LOAD_IMAGE_GRAYSCALE);

    sensor_msgs::Image sensorMsg = convertToSensorMsg(image);

    sensor_msgs::ImageConstPtr testImageScan(new sensor_msgs::Image(sensorMsg));

    EXPECT_NEAR(45, VisionDecision::getDesiredAngle(testImageScan->height / 8.0, testImageScan, 0.25), 15);
}

TEST(imageTest, moveAwayFromLineAndTurnLeft){
    string filename = "imageTests/testStraightOnRightSide.jpg";
    Mat image = imread(filename, CV_LOAD_IMAGE_GRAYSCALE);

    sensor_msgs::Image sensorMsg = convertToSensorMsg(image);

    sensor_msgs::ImageConstPtr testImageScan(new sensor_msgs::Image(sensorMsg));

    EXPECT_NEAR(-45, VisionDecision::getDesiredAngle(testImageScan->height / 8.0, testImageScan, 0.25), 15);
}

TEST(imageTest, noisyLeft){
    string filename = "imageTests/testVeryNoisyLeftImage.jpg";
    Mat image = imread(filename, CV_LOAD_IMAGE_GRAYSCALE);

    sensor_msgs::Image sensorMsg = convertToSensorMsg(image);

    sensor_msgs::ImageConstPtr testImageScan(new sensor_msgs::Image(sensorMsg));

    EXPECT_NEAR(-30, VisionDecision::getDesiredAngle(testImageScan->height / 8.0, testImageScan, 0.25), 20);
}

/**
 *  Tests for line that signals to turn left while starting higher than the bottom
 *  of the image.
 */
TEST(imageTest, elevatedLeftLine){
    string filename = "imageTests/testElevatedLeftLine.jpg";
    Mat image = imread(filename, CV_LOAD_IMAGE_GRAYSCALE);

    sensor_msgs::Image sensorMsg = convertToSensorMsg(image);

    sensor_msgs::ImageConstPtr testImageScan(new sensor_msgs::Image(sensorMsg));

    EXPECT_NEAR(-30, VisionDecision::getDesiredAngle(testImageScan->height / 8.0, testImageScan, 0.25), 20);
}

/**
 *  Tests for line that signals to turn right while starting higher than the bottom
 *  of the image.
 */
TEST(imageTest, elevatedRightLine){
    string filename = "imageTests/testElevatedRightLine.jpg";
    Mat image = imread(filename, CV_LOAD_IMAGE_GRAYSCALE);

    sensor_msgs::Image sensorMsg = convertToSensorMsg(image);

    sensor_msgs::ImageConstPtr testImageScan(new sensor_msgs::Image(sensorMsg));

    EXPECT_NEAR(30, VisionDecision::getDesiredAngle(testImageScan->height / 8.0, testImageScan, 0.25), 20);
}

TEST(imageTest, straightButLineNearEdge){
    string filename = "imageTests/testStraightButLineNearEdge.jpg";
    Mat image = imread(filename, CV_LOAD_IMAGE_GRAYSCALE);

    sensor_msgs::Image sensorMsg = convertToSensorMsg(image);

    sensor_msgs::ImageConstPtr testImageScan(new sensor_msgs::Image(sensorMsg));

    EXPECT_EQ(STOP_SIGNAL_ANGLE, VisionDecision::getDesiredAngle(testImageScan->height / 8.0, testImageScan, 0.25));
}

TEST(imageTest, perpendicular){
    string filename = "imageTests/testPerpendicular.jpg";
    Mat image = imread(filename, CV_LOAD_IMAGE_GRAYSCALE);

    sensor_msgs::Image sensorMsg = convertToSensorMsg(image);

    sensor_msgs::ImageConstPtr testImageScan(new sensor_msgs::Image(sensorMsg));

    EXPECT_EQ(STOP_SIGNAL_ANGLE, VisionDecision::getDesiredAngle(testImageScan->height / 8.0, testImageScan, 0.25));
}

TEST(imageTest, curved){
    string filename = "imageTests/testCurvedLine.jpg";
    Mat image = imread(filename, CV_LOAD_IMAGE_GRAYSCALE);

    sensor_msgs::Image sensorMsg = convertToSensorMsg(image);

    sensor_msgs::ImageConstPtr testImageScan(new sensor_msgs::Image(sensorMsg));

    EXPECT_NEAR(-30, VisionDecision::getDesiredAngle(testImageScan->height / 8.0, testImageScan, 0.25), 20);
}

TEST(imageTest, multiLineLeft){
    string filename = "imageTests/testMultipleLinesLeft.jpg";
    Mat image = imread(filename, CV_LOAD_IMAGE_GRAYSCALE);

    sensor_msgs::Image sensorMsg = convertToSensorMsg(image);

    sensor_msgs::ImageConstPtr testImageScan(new sensor_msgs::Image(sensorMsg));

    EXPECT_NEAR(-30, VisionDecision::getDesiredAngle(testImageScan->height / 8.0, testImageScan, 0.25), 20);
}

TEST(imageTest, multiLineRight){
    string filename = "imageTests/testMultipleLinesRight.jpg";
    Mat image = imread(filename, CV_LOAD_IMAGE_GRAYSCALE);

    sensor_msgs::Image sensorMsg = convertToSensorMsg(image);

    sensor_msgs::ImageConstPtr testImageScan(new sensor_msgs::Image(sensorMsg));

    EXPECT_NEAR(30, VisionDecision::getDesiredAngle(testImageScan->height / 8.0, testImageScan, 0.25), 20);
}

TEST(speedTest, angular){
    EXPECT_EQ(0, VisionDecision::getDesiredAngularSpeed(0));
    EXPECT_EQ(0, VisionDecision::getDesiredAngularSpeed(90));
    EXPECT_EQ(-1, VisionDecision::getDesiredAngularSpeed(-90));
    EXPECT_EQ(0.5, VisionDecision::getDesiredAngularSpeed(45));
}

TEST(speedTest, linear){
    EXPECT_EQ(1, VisionDecision::getDesiredLinearSpeed(0));
    EXPECT_EQ(0, VisionDecision::getDesiredLinearSpeed(90));
    EXPECT_EQ(0, VisionDecision::getDesiredLinearSpeed(-90));
    EXPECT_EQ(0.5, VisionDecision::getDesiredLinearSpeed(45));
}

/**
 * Helper function to turn a cvMatImage into a sensor image.
 */
sensor_msgs::Image convertToSensorMsg(Mat cvMatImage){
    sensor_msgs::Image img_msg; // >> message to be sent
    MatIterator_<uchar> it, end;

    img_msg.height           = cvMatImage.rows;
    img_msg.width            = cvMatImage.cols;
    img_msg.encoding         = "8UC1";
    img_msg.is_bigendian     = false;
    img_msg.step             = (int) cvMatImage.step;

    for (it = cvMatImage.begin<uchar>(), end = cvMatImage.end<uchar>(); it != end; it++){
        if((int) *it == 0)
            img_msg.data.push_back(0);
        else
            img_msg.data.push_back(255);
    }


    return img_msg;
    }

int main(int aimageTests, char **argv) {
    std::cout << argv[0] << std::endl;
    testing::InitGoogleTest(&aimageTests, argv);
    return RUN_ALL_TESTS();
}