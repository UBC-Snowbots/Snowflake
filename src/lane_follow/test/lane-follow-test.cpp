/*
 * Created By: Raad Khan
 * Created On: July 9, 2017
 * Description: Tests LaneFollow node
 */

#include <LaneFollow.h>
#include <gtest/gtest.h>

using namespace cv;

TEST(LineDetect, straightLaneFollowTest) {

    std::string image_path = "images/straightImage.jpg";
    cv::Mat testColor      = imread(image_path);
    cv::Mat testGray;
    cv::cvtColor(testColor, testGray, CV_BGR2GRAY);

    LineDetect testLineDetect;

    std::vector<Window> testBaseWindows =
    testLineDetect.getBaseWindows(testGray);
    std::vector<std::vector<cv::Point2d>> testFilteredLanePoints =
    testLineDetect.getLanePoints(testGray, testBaseWindows);
    std::vector<Polynomial> testFilteredLaneLines =
    testLineDetect.getLaneLines(testFilteredLanePoints);
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
