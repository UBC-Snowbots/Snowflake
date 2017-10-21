/*
 * Created By: Raad Khan
 * Created On: July 9, 2017
 * Description: Tests LaneFollow node
 */

#include <LaneFollow.h>
#include <LineDetect.h>
#include <gtest/gtest.h>
#include <opencv2/opencv.hpp>

using namespace cv;

TEST(LineDetect, straightLaneFollowTest) {

std::string image_path = "images/straightImage.jpg";
cv::Mat testColor = imread(image_path);
cv::Mat testGray;
cv::cvtColor(testColor, testGray, CV_BGR2GRAY);

LineDetect testLineDetect;
    
std::vector<Window> testBaseWindows = testLineDetect.getBaseWindows(testGray);
std::vector <std::vector<cv::Point2d>> testFilteredLanePoints = testLineDetect.getLanePoints(testGray, testBaseWindows);
std::vector<Polynomial> testFilteredLaneLines = testLineDetect.getLaneLines(testFilteredLanePoints);
// transform IPM, cartesian coordinates to real-world, ROS coordinates
// std::vector <std::vector<cv::Point2d>> testRealLanePoints = ...
// std::vector<Polynomial> testRealLaneLines = testLineDetect.getLaneLines(testRealLanePoints);
// cv::Point2d testIntersect = testLineDetect.getIntersection(testRealLaneLines[0], testRealLaneLines[1]);
// double testAngle = testLineDetect.getAngleFromOriginToPoint(testIntersect);

// EXPECT_NEAR(0, testAngle, 10);
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

