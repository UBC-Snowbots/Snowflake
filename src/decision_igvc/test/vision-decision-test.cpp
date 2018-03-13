/*
 * Created By: Robyn Castro
 * Created On: September 22, 2016
 * Description: Tests for VisionDecision
 */

#include <VisionDecision.h>
#include <cv_bridge/cv_bridge.h>
#include <gtest/gtest.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;
using namespace std;

sensor_msgs::Image convertToSensorMsg(Mat cvMatImage);

double confidence_value;

// Dictates how much new samples will influence the current
// average (Smaller value means less influence).
const double rolling_average_constant = 0.25;

// Dictates how much of the image samples need to be valid.
const double percent_of_samples_needed = 0.125;

// Dictates how much of the image is sampled
const double percent_of_image_sampled = 0.25;

// Dicates the angle to just ignore angle and turn away from the line
const double move_away_threshold = 20;

// Dictates how confident vision has to be to move
const double confidence_threshold = 60;

// Dictates how much white needs to be onscreen to be confident
const double percent_of_white_needed = 0.05;

// Dictates the amount of error allowed
const double error_margin = 20;

TEST(imageTest, moveAwayFromLineAndTurnRight) {
    string filename = "imageTests/testStraightImage.jpg";
    Mat image       = imread(filename, CV_LOAD_IMAGE_GRAYSCALE);

    sensor_msgs::Image sensorMsg = convertToSensorMsg(image);

    sensor_msgs::ImageConstPtr testImageScan(new sensor_msgs::Image(sensorMsg));

    EXPECT_NEAR(45,
                VisionDecision::getDesiredAngle(testImageScan->height / 8.0,
                                                testImageScan,
                                                rolling_average_constant,
                                                percent_of_image_sampled,
                                                percent_of_samples_needed,
                                                confidence_value,
                                                move_away_threshold,
                                                percent_of_white_needed),
                error_margin);
}

TEST(imageTest, angleLeft) {
    string filename = "imageTests/testLeftImage.jpg";
    Mat image       = imread(filename, CV_LOAD_IMAGE_GRAYSCALE);

    sensor_msgs::Image sensorMsg = convertToSensorMsg(image);

    sensor_msgs::ImageConstPtr testImageScan(new sensor_msgs::Image(sensorMsg));

    EXPECT_NEAR(-30,
                VisionDecision::getDesiredAngle(testImageScan->height / 8.0,
                                                testImageScan,
                                                rolling_average_constant,
                                                percent_of_image_sampled,
                                                percent_of_samples_needed,
                                                confidence_value,
                                                move_away_threshold,
                                                percent_of_white_needed),
                error_margin);
}

TEST(imageTest, angleRight) {
    string filename = "imageTests/testNoisyRightImage.jpg";
    Mat image       = imread(filename, CV_LOAD_IMAGE_GRAYSCALE);

    sensor_msgs::Image sensorMsg = convertToSensorMsg(image);

    sensor_msgs::ImageConstPtr testImageScan(new sensor_msgs::Image(sensorMsg));

    EXPECT_NEAR(30,
                VisionDecision::getDesiredAngle(testImageScan->height / 8.0,
                                                testImageScan,
                                                rolling_average_constant,
                                                percent_of_image_sampled,
                                                percent_of_samples_needed,
                                                confidence_value,
                                                move_away_threshold,
                                                percent_of_white_needed),
                error_margin);
}

TEST(imageTest, moveAwayFromLineAndTurnRightTwo) {
    string filename = "imageTests/testVeryNoisyStraightImage.jpg";
    Mat image       = imread(filename, CV_LOAD_IMAGE_GRAYSCALE);

    sensor_msgs::Image sensorMsg = convertToSensorMsg(image);

    sensor_msgs::ImageConstPtr testImageScan(new sensor_msgs::Image(sensorMsg));

    EXPECT_NEAR(45,
                VisionDecision::getDesiredAngle(testImageScan->height / 8.0,
                                                testImageScan,
                                                rolling_average_constant,
                                                percent_of_image_sampled,
                                                percent_of_samples_needed,
                                                confidence_value,
                                                move_away_threshold,
                                                percent_of_white_needed),
                error_margin);
}

TEST(imageTest, moveAwayFromLineAndTurnLeft) {
    string filename = "imageTests/testStraightOnRightSide.jpg";
    Mat image       = imread(filename, CV_LOAD_IMAGE_GRAYSCALE);

    sensor_msgs::Image sensorMsg = convertToSensorMsg(image);

    sensor_msgs::ImageConstPtr testImageScan(new sensor_msgs::Image(sensorMsg));

    EXPECT_NEAR(-45,
                VisionDecision::getDesiredAngle(testImageScan->height / 8.0,
                                                testImageScan,
                                                rolling_average_constant,
                                                percent_of_image_sampled,
                                                percent_of_samples_needed,
                                                confidence_value,
                                                move_away_threshold,
                                                percent_of_white_needed),
                error_margin);
}

TEST(imageTest, noisyLeft) {
    string filename = "imageTests/testVeryNoisyLeftImage.jpg";
    Mat image       = imread(filename, CV_LOAD_IMAGE_GRAYSCALE);

    sensor_msgs::Image sensorMsg = convertToSensorMsg(image);

    sensor_msgs::ImageConstPtr testImageScan(new sensor_msgs::Image(sensorMsg));

    EXPECT_NEAR(45,
                VisionDecision::getDesiredAngle(testImageScan->height / 8.0,
                                                testImageScan,
                                                rolling_average_constant,
                                                percent_of_image_sampled,
                                                percent_of_samples_needed,
                                                confidence_value,
                                                move_away_threshold,
                                                percent_of_white_needed),
                error_margin);
}

/**
 *  Tests for line that signals to turn left while starting higher than the
 * bottom
 *  of the image.
 */
TEST(imageTest, elevatedLeftLine) {
    string filename = "imageTests/testElevatedLeftLine.jpg";
    Mat image       = imread(filename, CV_LOAD_IMAGE_GRAYSCALE);

    sensor_msgs::Image sensorMsg = convertToSensorMsg(image);

    sensor_msgs::ImageConstPtr testImageScan(new sensor_msgs::Image(sensorMsg));

    EXPECT_NEAR(-30,
                VisionDecision::getDesiredAngle(testImageScan->height / 8.0,
                                                testImageScan,
                                                rolling_average_constant,
                                                percent_of_image_sampled,
                                                percent_of_samples_needed,
                                                confidence_value,
                                                move_away_threshold,
                                                percent_of_white_needed),
                error_margin);
}

/**
 *  Tests for line that signals to turn right while starting higher than the
 * bottom
 *  of the image.
 */
TEST(imageTest, elevatedRightLine) {
    string filename = "imageTests/testElevatedRightLine.jpg";
    Mat image       = imread(filename, CV_LOAD_IMAGE_GRAYSCALE);

    sensor_msgs::Image sensorMsg = convertToSensorMsg(image);

    sensor_msgs::ImageConstPtr testImageScan(new sensor_msgs::Image(sensorMsg));

    EXPECT_NEAR(30,
                VisionDecision::getDesiredAngle(testImageScan->height / 8.0,
                                                testImageScan,
                                                rolling_average_constant,
                                                percent_of_image_sampled,
                                                percent_of_samples_needed,
                                                confidence_value,
                                                move_away_threshold,
                                                percent_of_white_needed),
                error_margin);
}

TEST(imageTest, straightButLineNearEdge) {
    string filename = "imageTests/testStraightButLineNearEdge.jpg";
    Mat image       = imread(filename, CV_LOAD_IMAGE_GRAYSCALE);

    sensor_msgs::Image sensorMsg = convertToSensorMsg(image);

    sensor_msgs::ImageConstPtr testImageScan(new sensor_msgs::Image(sensorMsg));

    EXPECT_NEAR(-45,
                VisionDecision::getDesiredAngle(testImageScan->height / 8.0,
                                                testImageScan,
                                                rolling_average_constant,
                                                percent_of_image_sampled,
                                                percent_of_samples_needed,
                                                confidence_value,
                                                move_away_threshold,
                                                percent_of_white_needed),
                error_margin);
}

TEST(imageTest, perpendicular) {
    string filename = "imageTests/testPerpendicular.jpg";
    Mat image       = imread(filename, CV_LOAD_IMAGE_GRAYSCALE);

    sensor_msgs::Image sensorMsg = convertToSensorMsg(image);

    sensor_msgs::ImageConstPtr testImageScan(new sensor_msgs::Image(sensorMsg));

    VisionDecision::getDesiredAngle(testImageScan->height / 8.0,
                                    testImageScan,
                                    rolling_average_constant,
                                    percent_of_image_sampled,
                                    percent_of_samples_needed,
                                    confidence_value,
                                    move_away_threshold,
                                    percent_of_white_needed);

    EXPECT_EQ(0, confidence_value);
}

TEST(imageTest, curved) {
    string filename = "imageTests/testCurvedLine.jpg";
    Mat image       = imread(filename, CV_LOAD_IMAGE_GRAYSCALE);

    sensor_msgs::Image sensorMsg = convertToSensorMsg(image);

    sensor_msgs::ImageConstPtr testImageScan(new sensor_msgs::Image(sensorMsg));

    EXPECT_NEAR(-45,
                VisionDecision::getDesiredAngle(testImageScan->height / 8.0,
                                                testImageScan,
                                                rolling_average_constant,
                                                percent_of_image_sampled,
                                                percent_of_samples_needed,
                                                confidence_value,
                                                move_away_threshold,
                                                percent_of_white_needed),
                error_margin);
}

TEST(imageTest, multiLineLeft) {
    string filename = "imageTests/testMultipleLinesLeft.jpg";
    Mat image       = imread(filename, CV_LOAD_IMAGE_GRAYSCALE);

    sensor_msgs::Image sensorMsg = convertToSensorMsg(image);

    sensor_msgs::ImageConstPtr testImageScan(new sensor_msgs::Image(sensorMsg));

    EXPECT_NEAR(-30,
                VisionDecision::getDesiredAngle(testImageScan->height / 8.0,
                                                testImageScan,
                                                rolling_average_constant,
                                                percent_of_image_sampled,
                                                percent_of_samples_needed,
                                                confidence_value,
                                                move_away_threshold,
                                                percent_of_white_needed),
                error_margin);
}

TEST(imageTest, multiLineRight) {
    string filename = "imageTests/testMultipleLinesRight.jpg";
    Mat image       = imread(filename, CV_LOAD_IMAGE_GRAYSCALE);

    sensor_msgs::Image sensorMsg = convertToSensorMsg(image);

    sensor_msgs::ImageConstPtr testImageScan(new sensor_msgs::Image(sensorMsg));

    EXPECT_NEAR(30,
                VisionDecision::getDesiredAngle(testImageScan->height / 8.0,
                                                testImageScan,
                                                rolling_average_constant,
                                                percent_of_image_sampled,
                                                percent_of_samples_needed,
                                                confidence_value,
                                                move_away_threshold,
                                                percent_of_white_needed),
                error_margin);
}

TEST(imageTest, rightAngleLeft) {
    string filename = "imageTests/testRightAngleTurnLeft.jpg";
    Mat image       = imread(filename, CV_LOAD_IMAGE_GRAYSCALE);

    sensor_msgs::Image sensorMsg = convertToSensorMsg(image);

    sensor_msgs::ImageConstPtr testImageScan(new sensor_msgs::Image(sensorMsg));

    EXPECT_NEAR(-45,
                VisionDecision::getDesiredAngle(testImageScan->height / 8.0,
                                                testImageScan,
                                                rolling_average_constant,
                                                percent_of_image_sampled,
                                                percent_of_samples_needed,
                                                confidence_value,
                                                move_away_threshold,
                                                percent_of_white_needed),
                error_margin);
}

TEST(imageTest, rightAngleRight) {
    string filename = "imageTests/testRightAngleTurnRight.jpg";
    Mat image       = imread(filename, CV_LOAD_IMAGE_GRAYSCALE);

    sensor_msgs::Image sensorMsg = convertToSensorMsg(image);

    sensor_msgs::ImageConstPtr testImageScan(new sensor_msgs::Image(sensorMsg));

    EXPECT_NEAR(45,
                VisionDecision::getDesiredAngle(testImageScan->height / 8.0,
                                                testImageScan,
                                                rolling_average_constant,
                                                percent_of_image_sampled,
                                                percent_of_samples_needed,
                                                confidence_value,
                                                move_away_threshold,
                                                percent_of_white_needed),
                error_margin);
}

TEST(imageTest, cornerAngleRight) {
    string filename = "imageTests/testCornerTurnRight.jpg";
    Mat image       = imread(filename, CV_LOAD_IMAGE_GRAYSCALE);

    sensor_msgs::Image sensorMsg = convertToSensorMsg(image);

    sensor_msgs::ImageConstPtr testImageScan(new sensor_msgs::Image(sensorMsg));

    EXPECT_NEAR(45,
                VisionDecision::getDesiredAngle(testImageScan->height / 8.0,
                                                testImageScan,
                                                rolling_average_constant,
                                                percent_of_image_sampled,
                                                percent_of_samples_needed,
                                                confidence_value,
                                                move_away_threshold,
                                                percent_of_white_needed),
                error_margin);
}

TEST(imageTest, cornerBigAngleRight) {
    string filename = "imageTests/testCornerTurnRight.jpg";
    Mat image       = imread(filename, CV_LOAD_IMAGE_GRAYSCALE);

    sensor_msgs::Image sensorMsg = convertToSensorMsg(image);

    sensor_msgs::ImageConstPtr testImageScan(new sensor_msgs::Image(sensorMsg));

    EXPECT_NEAR(45,
                VisionDecision::getDesiredAngle(testImageScan->height / 8.0,
                                                testImageScan,
                                                rolling_average_constant,
                                                percent_of_image_sampled,
                                                percent_of_samples_needed,
                                                confidence_value,
                                                move_away_threshold,
                                                percent_of_white_needed),
                error_margin);
}

TEST(imageTest, cornerBigAngleLeft) {
    string filename = "imageTests/testBigCornerTurnLeft.jpg";
    Mat image       = imread(filename, CV_LOAD_IMAGE_GRAYSCALE);

    sensor_msgs::Image sensorMsg = convertToSensorMsg(image);

    sensor_msgs::ImageConstPtr testImageScan(new sensor_msgs::Image(sensorMsg));

    EXPECT_NEAR(-45,
                VisionDecision::getDesiredAngle(testImageScan->height / 8.0,
                                                testImageScan,
                                                rolling_average_constant,
                                                percent_of_image_sampled,
                                                percent_of_samples_needed,
                                                confidence_value,
                                                move_away_threshold,
                                                percent_of_white_needed),
                error_margin);
}

TEST(imageTest, cornerAngleLeft) {
    string filename = "imageTests/testCornerTurnLeft.jpg";
    Mat image       = imread(filename, CV_LOAD_IMAGE_GRAYSCALE);

    sensor_msgs::Image sensorMsg = convertToSensorMsg(image);

    sensor_msgs::ImageConstPtr testImageScan(new sensor_msgs::Image(sensorMsg));

    EXPECT_NEAR(-45,
                VisionDecision::getDesiredAngle(testImageScan->height / 8.0,
                                                testImageScan,
                                                rolling_average_constant,
                                                percent_of_image_sampled,
                                                percent_of_samples_needed,
                                                confidence_value,
                                                move_away_threshold,
                                                percent_of_white_needed),
                error_margin);
}

TEST(imageTest, coneAngleLeft) {
    string filename = "imageTests/testConeTurnLeft.jpg";
    Mat image       = imread(filename, CV_LOAD_IMAGE_GRAYSCALE);

    sensor_msgs::Image sensorMsg = convertToSensorMsg(image);

    sensor_msgs::ImageConstPtr testImageScan(new sensor_msgs::Image(sensorMsg));

    EXPECT_NEAR(-45,
                VisionDecision::getDesiredAngle(testImageScan->height / 8.0,
                                                testImageScan,
                                                rolling_average_constant,
                                                percent_of_image_sampled,
                                                percent_of_samples_needed,
                                                confidence_value,
                                                move_away_threshold,
                                                percent_of_white_needed),
                error_margin);
}

TEST(imageTest, coneAngleRight) {
    string filename = "imageTests/testConeTurnRight.jpg";
    Mat image       = imread(filename, CV_LOAD_IMAGE_GRAYSCALE);

    sensor_msgs::Image sensorMsg = convertToSensorMsg(image);

    sensor_msgs::ImageConstPtr testImageScan(new sensor_msgs::Image(sensorMsg));

    EXPECT_NEAR(45,
                VisionDecision::getDesiredAngle(testImageScan->height / 8.0,
                                                testImageScan,
                                                rolling_average_constant,
                                                percent_of_image_sampled,
                                                percent_of_samples_needed,
                                                confidence_value,
                                                move_away_threshold,
                                                percent_of_white_needed),
                error_margin);
}

TEST(imageTest, oppositeConeAngleRight) {
    string filename = "imageTests/testOppositeConeTurnRight.jpg";
    Mat image       = imread(filename, CV_LOAD_IMAGE_GRAYSCALE);

    sensor_msgs::Image sensorMsg = convertToSensorMsg(image);

    sensor_msgs::ImageConstPtr testImageScan(new sensor_msgs::Image(sensorMsg));

    EXPECT_NEAR(45,
                VisionDecision::getDesiredAngle(testImageScan->height / 8.0,
                                                testImageScan,
                                                rolling_average_constant,
                                                percent_of_image_sampled,
                                                percent_of_samples_needed,
                                                confidence_value,
                                                move_away_threshold,
                                                percent_of_white_needed),
                error_margin);
}

TEST(imageTest, avoidConeLeft) {
    string filename = "imageTests/testAvoidCone.jpg";
    Mat image       = imread(filename, CV_LOAD_IMAGE_GRAYSCALE);

    sensor_msgs::Image sensorMsg = convertToSensorMsg(image);

    sensor_msgs::ImageConstPtr testImageScan(new sensor_msgs::Image(sensorMsg));

    EXPECT_NEAR(45,
                VisionDecision::getDesiredAngle(testImageScan->height / 8.0,
                                                testImageScan,
                                                rolling_average_constant,
                                                percent_of_image_sampled,
                                                percent_of_samples_needed,
                                                confidence_value,
                                                move_away_threshold,
                                                percent_of_white_needed),
                error_margin);
}

TEST(imageTest, avoidConeRight) {
    string filename = "imageTests/testAvoidConeRight.jpg";
    Mat image       = imread(filename, CV_LOAD_IMAGE_GRAYSCALE);

    sensor_msgs::Image sensorMsg = convertToSensorMsg(image);

    sensor_msgs::ImageConstPtr testImageScan(new sensor_msgs::Image(sensorMsg));

    EXPECT_NEAR(45,
                VisionDecision::getDesiredAngle(testImageScan->height / 8.0,
                                                testImageScan,
                                                rolling_average_constant,
                                                percent_of_image_sampled,
                                                percent_of_samples_needed,
                                                confidence_value,
                                                move_away_threshold,
                                                percent_of_white_needed),
                error_margin);
}

TEST(speedTest, linear) {
    EXPECT_EQ(1, VisionDecision::getDesiredLinearSpeed(0));
    EXPECT_EQ(0, VisionDecision::getDesiredLinearSpeed(90));
    EXPECT_EQ(0, VisionDecision::getDesiredLinearSpeed(-90));
    EXPECT_EQ(0.5, VisionDecision::getDesiredLinearSpeed(45));
}

/**
 * Helper function to turn a cvMatImage into a sensor image.
 */
sensor_msgs::Image convertToSensorMsg(Mat cvMatImage) {
    sensor_msgs::Image img_msg; // >> message to be sent
    MatIterator_<uchar> it, end;

    img_msg.height       = cvMatImage.rows;
    img_msg.width        = cvMatImage.cols;
    img_msg.encoding     = "8UC1";
    img_msg.is_bigendian = false;
    img_msg.step         = (int) cvMatImage.step;

    for (it = cvMatImage.begin<uchar>(), end = cvMatImage.end<uchar>();
         it != end;
         it++) {
        if ((int) *it == 0)
            img_msg.data.push_back(0);
        else
            img_msg.data.push_back(255);
    }

    return img_msg;
}

int main(int aimageTests, char** argv) {
    std::cout << argv[0] << std::endl;
    testing::InitGoogleTest(&aimageTests, argv);
    return RUN_ALL_TESTS();
}