/*
 * Created By: Gareth Ellis
 * Created On: September 22, 2016
 * Description: Tests for VisionDecision
 */

#include <VisionDecision.h>
#include <gtest/gtest.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <tiff.h>

using namespace cv;

sensor_msgs::Image convertToSensorMsg(Mat cvMatImage);

TEST(imageTest, angleStraight){
    String filename = "imageTests/testStraightImage.jpg";
    Mat image = imread(filename, CV_LOAD_IMAGE_GRAYSCALE);

    sensor_msgs::Image sensorMsg = convertToSensorMsg(image);

    sensor_msgs::ImageConstPtr testImageScan(new sensor_msgs::Image(sensorMsg));

    EXPECT_EQ(0, VisionDecision::getDesiredAngle(300, testImageScan));
}

TEST(imageTest, angleLeft){
    String filename = "imageTests/testLeftImage.jpg";
    Mat image = imread(filename, CV_LOAD_IMAGE_GRAYSCALE);

    sensor_msgs::Image sensorMsg = convertToSensorMsg(image);

    sensor_msgs::ImageConstPtr testImageScan(new sensor_msgs::Image(sensorMsg));

    EXPECT_EQ(-37, VisionDecision::getDesiredAngle(300, testImageScan));
}

TEST(imageTest, angleRight){
    String filename = "imageTests/testNoisyRightImage.jpg";
    Mat image = imread(filename, CV_LOAD_IMAGE_GRAYSCALE);

    sensor_msgs::Image sensorMsg = convertToSensorMsg(image);

    sensor_msgs::ImageConstPtr testImageScan(new sensor_msgs::Image(sensorMsg));

    EXPECT_EQ(33, VisionDecision::getDesiredAngle(300, testImageScan));
}

TEST(imageTest, noisyStraight){
    String filename = "imageTests/testVeryNoisyStraightImage.jpg";
    Mat image = imread(filename, CV_LOAD_IMAGE_GRAYSCALE);

    sensor_msgs::Image sensorMsg = convertToSensorMsg(image);

    sensor_msgs::ImageConstPtr testImageScan(new sensor_msgs::Image(sensorMsg));

    EXPECT_EQ(-5, VisionDecision::getDesiredAngle(300, testImageScan));
}

TEST(imageTest, noisyLeft){
    String filename = "imageTests/testVeryNoisyLeftImage.jpg";
    Mat image = imread(filename, CV_LOAD_IMAGE_GRAYSCALE);

    sensor_msgs::Image sensorMsg = convertToSensorMsg(image);

    sensor_msgs::ImageConstPtr testImageScan(new sensor_msgs::Image(sensorMsg));

    EXPECT_EQ(-37, VisionDecision::getDesiredAngle(300, testImageScan));
}

TEST(imageTest, splitLines){
    String filename = "imageTests/testSplitLines.jpg";
    Mat image = imread(filename, CV_LOAD_IMAGE_GRAYSCALE);

    sensor_msgs::Image sensorMsg = convertToSensorMsg(image);

    sensor_msgs::ImageConstPtr testImageScan(new sensor_msgs::Image(sensorMsg));

    EXPECT_EQ(-59, VisionDecision::getDesiredAngle(300, testImageScan));
}

TEST(imageTest, splitLinesRight){
    String filename = "imageTests/testSplitLinesRight.jpg";
    Mat image = imread(filename, CV_LOAD_IMAGE_GRAYSCALE);

    sensor_msgs::Image sensorMsg = convertToSensorMsg(image);

    sensor_msgs::ImageConstPtr testImageScan(new sensor_msgs::Image(sensorMsg));

    EXPECT_EQ(59, VisionDecision::getDesiredAngle(300, testImageScan));
}

TEST(imageTest, smallRight){
    String filename = "imageTests/testSmallRight.jpg";
    Mat image = imread(filename, CV_LOAD_IMAGE_GRAYSCALE);

    sensor_msgs::Image sensorMsg = convertToSensorMsg(image);

    sensor_msgs::ImageConstPtr testImageScan(new sensor_msgs::Image(sensorMsg));

    EXPECT_EQ(48, VisionDecision::getDesiredAngle(300, testImageScan));
}

TEST(imageTest, perpendicular){
    String filename = "imageTests/testPerpendicular.jpg";
    Mat image = imread(filename, CV_LOAD_IMAGE_GRAYSCALE);

    sensor_msgs::Image sensorMsg = convertToSensorMsg(image);

    sensor_msgs::ImageConstPtr testImageScan(new sensor_msgs::Image(sensorMsg));

    EXPECT_EQ(91, VisionDecision::getDesiredAngle(300, testImageScan));
}

TEST(speedTest, angular){
    EXPECT_EQ(0, VisionDecision::getDesiredAngularSpeed(0));
    EXPECT_EQ(100, VisionDecision::getDesiredAngularSpeed(90));
    EXPECT_EQ(100, VisionDecision::getDesiredAngularSpeed(-90));
    EXPECT_EQ(50, VisionDecision::getDesiredAngularSpeed(45));
}

TEST(speedTest, linear){
    EXPECT_EQ(100, VisionDecision::getDesiredSpeed(0));
    EXPECT_EQ(0, VisionDecision::getDesiredSpeed(90));
    EXPECT_EQ(0, VisionDecision::getDesiredSpeed(-90));
    EXPECT_EQ(50, VisionDecision::getDesiredSpeed(45));
}


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
