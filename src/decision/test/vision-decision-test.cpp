/*
 * Created By: Gareth Ellis
 * Created On: September 22, 2016
 * Description: Tests for VisionDecision
 */

#include <VisionDecision.h>
#include <gtest/gtest.h>

TEST(imageRatioTest1, selfCreated){
    sensor_msgs::ImagePtr testImageScan;

    testImageScan -> width = 8;
    testImageScan -> height = 8;

    for(int rowCount = 0; rowCount < testImageScan->height; rowCount++){
        for(int columnCount = 0; columnCount < testImageScan->width; columnCount++){
            testImageScan->data[columnCount*rowCount] = 255;
        }
    }

    EXPECT_EQ(0, VisionDecision::getImageRatio(testImageScan));
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
