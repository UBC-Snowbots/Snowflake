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

sensor_msgs::Image convertToSensorMsg(Mat cvMatImage);

TEST(imageTest, angleStraight){
    String filename = "imageTests/testStraightImage.jpg";
    Mat image = imread(filename, CV_LOAD_IMAGE_GRAYSCALE);

    sensor_msgs::Image sensorMsg = convertToSensorMsg(image);

    sensor_msgs::ImageConstPtr testImageScan(new sensor_msgs::Image(sensorMsg));

    EXPECT_NEAR(0, VisionDecision::getDesiredAngle(testImageScan->height / 4.0, testImageScan, 0), 10);
}

TEST(imageTest, angleLeft){
    String filename = "imageTests/testLeftImage.jpg";
    Mat image = imread(filename, CV_LOAD_IMAGE_GRAYSCALE);

    sensor_msgs::Image sensorMsg = convertToSensorMsg(image);

    sensor_msgs::ImageConstPtr testImageScan(new sensor_msgs::Image(sensorMsg));

    EXPECT_NEAR(-30, VisionDecision::getDesiredAngle(testImageScan->height / 4.0, testImageScan, 0), 30);
}

TEST(imageTest, angleRight){
    String filename = "imageTests/testNoisyRightImage.jpg";
    Mat image = imread(filename, CV_LOAD_IMAGE_GRAYSCALE);

    sensor_msgs::Image sensorMsg = convertToSensorMsg(image);

    sensor_msgs::ImageConstPtr testImageScan(new sensor_msgs::Image(sensorMsg));

    EXPECT_NEAR(30, VisionDecision::getDesiredAngle(testImageScan->height / 4.0, testImageScan, 0), 30);
}

TEST(imageTest, noisyStraight){
    String filename = "imageTests/testVeryNoisyStraightImage.jpg";
    Mat image = imread(filename, CV_LOAD_IMAGE_GRAYSCALE);

    sensor_msgs::Image sensorMsg = convertToSensorMsg(image);

    sensor_msgs::ImageConstPtr testImageScan(new sensor_msgs::Image(sensorMsg));
    EXPECT_NEAR(0, VisionDecision::getDesiredAngle(testImageScan->height / 4.0, testImageScan, 0), 10);
}

TEST(imageTest, noisyLeft){
    String filename = "imageTests/testVeryNoisyLeftImage.jpg";
    Mat image = imread(filename, CV_LOAD_IMAGE_GRAYSCALE);

    sensor_msgs::Image sensorMsg = convertToSensorMsg(image);

    sensor_msgs::ImageConstPtr testImageScan(new sensor_msgs::Image(sensorMsg));

    EXPECT_NEAR(-30, VisionDecision::getDesiredAngle(testImageScan->height / 4.0, testImageScan, 0), 30);
}

/**
 *  Tests for line that signals to turn left while starting higher than the bottom
 *  of the image.
 */
TEST(imageTest, elevatedLeftLine){
    String filename = "imageTests/testElevatedLeftLine.jpg";
    Mat image = imread(filename, CV_LOAD_IMAGE_GRAYSCALE);

    sensor_msgs::Image sensorMsg = convertToSensorMsg(image);

    sensor_msgs::ImageConstPtr testImageScan(new sensor_msgs::Image(sensorMsg));

    EXPECT_NEAR(-30, VisionDecision::getDesiredAngle(testImageScan->height / 4.0, testImageScan, 0), 30);
}

/**
 *  Tests for line that signals to turn right while starting higher than the bottom
 *  of the image.
 */
TEST(imageTest, elevatedRightLine){
    String filename = "imageTests/testElevatedRightLine.jpg";
    Mat image = imread(filename, CV_LOAD_IMAGE_GRAYSCALE);

    sensor_msgs::Image sensorMsg = convertToSensorMsg(image);

    sensor_msgs::ImageConstPtr testImageScan(new sensor_msgs::Image(sensorMsg));

    EXPECT_NEAR(30, VisionDecision::getDesiredAngle(testImageScan->height / 4.0, testImageScan, 0), 30);
}

TEST(imageTest, straightButLineNearEdge){
    String filename = "imageTests/testStraightButLineNearEdge.jpg";
    Mat image = imread(filename, CV_LOAD_IMAGE_GRAYSCALE);

    sensor_msgs::Image sensorMsg = convertToSensorMsg(image);

    sensor_msgs::ImageConstPtr testImageScan(new sensor_msgs::Image(sensorMsg));

    EXPECT_NEAR(STOP_SIGNAL_ANGLE, VisionDecision::getDesiredAngle(testImageScan->height / 4.0, testImageScan, 0), 20);
}

TEST(imageTest, perpendicular){
    String filename = "imageTests/testPerpendicular.jpg";
    Mat image = imread(filename, CV_LOAD_IMAGE_GRAYSCALE);

    sensor_msgs::Image sensorMsg = convertToSensorMsg(image);

    sensor_msgs::ImageConstPtr testImageScan(new sensor_msgs::Image(sensorMsg));

    EXPECT_EQ(STOP_SIGNAL_ANGLE, VisionDecision::getDesiredAngle(testImageScan->height / 4.0, testImageScan, 0));
}

TEST(imageTest, curved){
    String filename = "imageTests/testCurvedLine.jpg";
    Mat image = imread(filename, CV_LOAD_IMAGE_GRAYSCALE);

    sensor_msgs::Image sensorMsg = convertToSensorMsg(image);

    sensor_msgs::ImageConstPtr testImageScan(new sensor_msgs::Image(sensorMsg));

    EXPECT_NEAR(-30, VisionDecision::getDesiredAngle(testImageScan->height / 4.0, testImageScan, 0), 30);
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
    testing::InitGoogleTest(&aimageTests, argv);
    return RUN_ALL_TESTS();
}