/*
 * Created By: Raad Khan
 * Created On: July 9, 2017
 * Description: Tests LaneFollow
 */

#include <LaneFollow.h>
#include <gtest/gtest.h>

using namespace cv;

TEST(LineDetect, straightLaneFollowTest) {

    std::string image_path = "images/straightImage.jpg";
    cv::Mat testColorImage = imread(image_path);
    cv::Mat testGrayImage;
    if (testColorImage.empty()) {
        printf("cannot access frame");
        return;
    }
    cv::cvtColor(testColorImage, testGrayImage, CV_BGR2GRAY);

    LineDetect testLineDetect;

}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
