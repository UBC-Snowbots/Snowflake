#include <gtest/gtest.h>
#include <CircleDetection.h>
#include "../../vision/include/HSVFilterNode.h"

TEST(realImage, GreenLight) {
    std::string image_path = "images/GreenLight.jpg";

    cv::Mat bgr_image = imread(image_path);
    Mat filtered_image;

    HSVFilter testFilter = HSVFilter();
    testFilter.filterImage(bgr_image, filtered_image);

    CircleDetection *circleDetection = new CircleDetection();
    int numCircles = circleDetection->countCircles(filtered_image, false);

    EXPECT_EQ(1, numCircles);
}

TEST(realImage, noCircle) {
    std::string image_path = "images/RedLight.jpg";

    cv::Mat bgr_image = imread(image_path);
    Mat filtered_image;

    HSVFilter testFilter = HSVFilter();
    testFilter.filterImage(bgr_image, filtered_image);

    CircleDetection *circleDetection = new CircleDetection();
    int numCircles = circleDetection->countCircles(filtered_image, false);

    EXPECT_EQ(0, numCircles);
}

TEST(filteredImage, oneCircle) {
    std::string image_path = "images/binaryCircles.jpg";

    cv::Mat bgr_image = imread(image_path);
    
    CircleDetection *circleDetection = new CircleDetection();
    int numCircles = circleDetection->countCircles(bgr_image, false);

    EXPECT_EQ(1, numCircles);
}


int main(int aimageTests, char **argv) {
    testing::InitGoogleTest(&aimageTests, argv);
    return RUN_ALL_TESTS();
}
