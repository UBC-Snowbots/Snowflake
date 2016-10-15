/*
 * Created By: Gareth Ellis
 * Created On: September 22, 2016
 * Description: Tests for VisionDecision
 */

#include <VisionDecision.h>
#include <gtest/gtest.h>

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

    EXPECT_EQ(0.0, VisionDecision::getImageRatio(testImageScan));
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

    EXPECT_EQ(1.0, VisionDecision::getImageRatio(testImageScan));
 }

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
