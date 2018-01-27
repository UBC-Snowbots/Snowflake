#include <CircleDetection.h>
#include <HSVFilterNode.h>
#include <gtest/gtest.h>

TEST(realImage, GreenLight) {
    std::string image_path = "images/GreenLight.jpg";

    cv::Mat bgr_image = imread(image_path);
    Mat filtered_image;

    HSVFilter test_filter = HSVFilter();
    test_filter.filterImage(bgr_image, filtered_image);

    CircleDetection* circle_detection = new CircleDetection();
    int num_circles = circle_detection->countCircles(filtered_image, false);

    EXPECT_EQ(1, num_circles);
}

TEST(realImage, noCircle) {
    std::string image_path = "images/RedLight.jpg";

    cv::Mat bgr_image = imread(image_path);
    Mat filtered_image;

    HSVFilter testFilter = HSVFilter();
    testFilter.filterImage(bgr_image, filtered_image);

    CircleDetection* circle_detection = new CircleDetection();
    int num_circles = circle_detection->countCircles(filtered_image, false);

    EXPECT_EQ(0, num_circles);
}

TEST(filteredImage, oneCircle) {
    std::string image_path = "images/binaryCircles.jpg";

    cv::Mat bgr_image = imread(image_path);

    CircleDetection* circle_detection = new CircleDetection();
    int num_circles = circle_detection->countCircles(bgr_image, false);

    EXPECT_EQ(1, num_circles);
}

int main(int imageTests, char** argv) {
    testing::InitGoogleTest(&imageTests, argv);
    return RUN_ALL_TESTS();
}
