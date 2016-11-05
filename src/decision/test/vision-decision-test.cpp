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

TEST(imageRatioTest, zeroTozero){

    sensor_msgs::Image output_image;

    output_image.height           = 8;
    output_image.width            = 8;
    output_image.encoding         = "8UC1";
    output_image.is_bigendian     = false;
    output_image.step             = 8;

   for(int rowCount = 0; rowCount < 8; rowCount++)
       for (int columnCount = 0; columnCount < 8; columnCount++)
           output_image.data.push_back(255);


    sensor_msgs::ImageConstPtr testImageScan(new sensor_msgs::Image(output_image));

    EXPECT_EQ(0.0, VisionDecision::getHorizontalImageRatio(testImageScan));
}

TEST(imageRatioTest, oneToTwo){

    sensor_msgs::Image output_image;

    output_image.height           = 8;
    output_image.width            = 8;
    output_image.encoding         = "8UC1";
    output_image.is_bigendian     = false;
    output_image.step             = 8;

    for(int rowCount = 0; rowCount < output_image.height; rowCount++)
        for (int columnCount = 0; columnCount < output_image.width; columnCount++)
            output_image.data.push_back(0);

    output_image.data[0] = 255;
    output_image.data[3*8+5] = 255;
    output_image.data[6*8+7] = 255;

    sensor_msgs::ImageConstPtr testImageScan(new sensor_msgs::Image(output_image));

    EXPECT_EQ(1.0, VisionDecision::getHorizontalImageRatio(testImageScan));
}

TEST(desiredAngleTest, twentyBiasFifty){
    EXPECT_EQ(72, VisionDecision::getDesiredAngle(20, -50, 50));
}


TEST(imageTest, angleThirtySpeedQuick){
    String filename = "/home/robyncastro/IGVC-2017/src/decision/imageTests/testStraightImage.jpg";
    Mat image = imread(filename, CV_LOAD_IMAGE_GRAYSCALE);

    sensor_msgs::Image sensorMsg = convertToSensorMsg(image);

    sensor_msgs::ImageConstPtr testImageScan(new sensor_msgs::Image(sensorMsg));

    int maxBias = testImageScan->width * testImageScan->height / 4;

    int imageRatio = VisionDecision::getHorizontalImageRatio(testImageScan);
    
    EXPECT_EQ(0, VisionDecision::getDesiredAngle(imageRatio, -maxBias, maxBias));
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
int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
