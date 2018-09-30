/*
 * Created By: Raad Khan
 * Created On: July 9, 2017
 * Description: Tests LineDetect
 */

#include "LineDetect.h"
#include <gtest/gtest.h>

using namespace cv;

TEST(LineDetect, getHistogramSmallSizeTest) {
    cv::Mat test_image(Size(4, 4), CV_8UC1, Scalar::all(0));

    test_image.at<uchar>(2, 1) = 255;

    LineDetect TestLineDetect;

    int_vec test_histogram     = TestLineDetect.getHistogram(test_image);
    int_vec expected_histogram = {0, 1, 0, 0};

    EXPECT_EQ(expected_histogram, test_histogram);
}

TEST(LineDetect, getHistogramLargeSizeTest) {
    cv::Mat test_image(Size(10, 10), CV_8UC1, Scalar::all(0));
    test_image.at<uchar>(0, 2) = 255;
    test_image.at<uchar>(4, 2) = 255;
    test_image.at<uchar>(8, 2) = 255;
    test_image.at<uchar>(7, 2) = 255;
    test_image.at<uchar>(3, 4) = 255;
    test_image.at<uchar>(4, 4) = 255;
    test_image.at<uchar>(9, 8) = 255;
    test_image.at<uchar>(2, 8) = 255;
    test_image.at<uchar>(3, 8) = 255;
    test_image.at<uchar>(7, 9) = 255;

    LineDetect TestLineDetect;

    int_vec test_histogram     = TestLineDetect.getHistogram(test_image);
    int_vec expected_histogram = {0, 0, 4, 0, 2, 0, 0, 0, 3, 1};

    EXPECT_EQ(expected_histogram, test_histogram);
}

TEST(LineDetect, getBaseHistogramPeakPositionSmallSizeTest) {
    int_vec test_histogram = {2, 1, 7, 6, 2, 1, 6, 6, 0};

    LineDetect TestLineDetect;

    std::pair<int, int> test_peak =
    TestLineDetect.getBaseHistogramPeakPositions(test_histogram);
    std::pair<int, int> expected_peak(2, 6);

    EXPECT_EQ(expected_peak, test_peak);
}

TEST(LineDetect, getBaseHistogramPeakPositionLargeSizeTest) {
    int_vec test_histogram = {0, 1, 3, 5, 2, 1, 1, 2, 6, 2, 0, 3};

    LineDetect TestLineDetect;

    std::pair<int, int> test_peak =
    TestLineDetect.getBaseHistogramPeakPositions(test_histogram);
    std::pair<int, int> expected_peak(3, 8);

    EXPECT_EQ(expected_peak, test_peak);
}

TEST(LineDetect, getWindowSliceLeftWindowTest) {
    cv::Mat test_image(Size(10, 10), CV_8UC1, Scalar::all(0));
    test_image.at<uchar>(0, 2) = 255;
    test_image.at<uchar>(4, 2) = 255;
    test_image.at<uchar>(8, 2) = 255;
    test_image.at<uchar>(7, 2) = 255;
    test_image.at<uchar>(3, 4) = 255;
    test_image.at<uchar>(4, 4) = 255;
    test_image.at<uchar>(9, 8) = 255;
    test_image.at<uchar>(2, 8) = 255;
    test_image.at<uchar>(3, 8) = 255;
    test_image.at<uchar>(7, 9) = 255;

    Window test_window{2, 2};

    int vertical_slice_index = 2;

    LineDetect TestLineDetect;

    cv::Mat test_window_slice = TestLineDetect.getWindowSlice(
    test_image, test_window, vertical_slice_index);
    cv::Mat expected_window_slice = test_image(Range(2, 3), Range(1, 3));
    // get a matrix with non-zero values at points where the two matrices have
    // different values
    cv::Mat diff = (test_window_slice != expected_window_slice);
    // equal if no matrix elements disagree
    bool equal = countNonZero(diff);

    EXPECT_EQ(false, equal);
}

TEST(LineDetect, getWindowSliceRightWindowTest) {
    cv::Mat test_image(Size(10, 10), CV_8UC1, Scalar::all(0));
    test_image.at<uchar>(0, 2) = 255;
    test_image.at<uchar>(4, 2) = 255;
    test_image.at<uchar>(8, 2) = 255;
    test_image.at<uchar>(7, 2) = 255;
    test_image.at<uchar>(3, 4) = 255;
    test_image.at<uchar>(4, 4) = 255;
    test_image.at<uchar>(9, 8) = 255;
    test_image.at<uchar>(2, 8) = 255;
    test_image.at<uchar>(3, 8) = 255;
    test_image.at<uchar>(7, 9) = 255;

    Window test_window{8, 2};

    int vertical_slice_index = 3;

    LineDetect TestLineDetect;

    cv::Mat test_window_slice = TestLineDetect.getWindowSlice(
    test_image, test_window, vertical_slice_index);
    cv::Mat expectedWindowSlice = test_image(Range(2, 3), Range(7, 9));
    // get a matrix with non-zero values at points where the two matrices have
    // different values
    cv::Mat diff = (test_window_slice != expectedWindowSlice);
    // equal if no matrix elements disagree
    bool equal = countNonZero(diff);

    EXPECT_EQ(false, equal);
}

// lane curving straight
TEST(LineDetect, fitPolyToLineLeftLineCurveStraightTest) {
    cv::Point2d test_point_1{1.0, 0.0};
    cv::Point2d test_point_2{1.0, 1.0};
    cv::Point2d test_point_3{2.0, 3.0};
    cv::Point2d test_point_4{2.5, 4.0};
    cv::Point2d test_point_5{3.0, 5.0};

    std::vector<cv::Point2d> test_points = {
    test_point_1, test_point_2, test_point_3, test_point_4, test_point_5};

    int test_order = 3;

    LineDetect TestLineDetect;

    Polynomial TestPolynomial =
    TestLineDetect.fitPolyToLine(test_points, test_order);

    Polynomial ExpectedPolynomial;
    ExpectedPolynomial.coefficients = {-3.50000, 5.08333, -1.25000, 0.16667};

    EXPECT_NEAR(ExpectedPolynomial.coefficients[3],
                TestPolynomial.coefficients[3],
                0.00001);
    EXPECT_NEAR(ExpectedPolynomial.coefficients[2],
                TestPolynomial.coefficients[2],
                0.00001);
    EXPECT_NEAR(ExpectedPolynomial.coefficients[1],
                TestPolynomial.coefficients[1],
                0.00001);
    EXPECT_NEAR(ExpectedPolynomial.coefficients[0],
                TestPolynomial.coefficients[0],
                0.00001);
}

TEST(LineDetect, fitPolyToLineRightLineCurveStraightTest) {
    cv::Point2d test_point_1{6.5, 0.0};
    cv::Point2d test_point_2{6.0, 1.0};
    cv::Point2d test_point_3{6.0, 2.0};
    cv::Point2d test_point_4{5.5, 3.0};
    cv::Point2d test_point_5{5.0, 5.0};

    std::vector<cv::Point2d> test_points = {
    test_point_1, test_point_2, test_point_3, test_point_4, test_point_5};

    int test_order = 3;

    LineDetect TestLineDetect;

    Polynomial TestPolynomial =
    TestLineDetect.fitPolyToLine(test_points, test_order);

    Polynomial ExpectedPolynomial;
    ExpectedPolynomial.coefficients = {
    162.50000, -74.83333, 12.00000, -0.66667};

    EXPECT_NEAR(ExpectedPolynomial.coefficients[3],
                TestPolynomial.coefficients[3],
                0.00001);
    EXPECT_NEAR(ExpectedPolynomial.coefficients[2],
                TestPolynomial.coefficients[2],
                0.00001);
    EXPECT_NEAR(ExpectedPolynomial.coefficients[1],
                TestPolynomial.coefficients[1],
                0.00001);
    EXPECT_NEAR(ExpectedPolynomial.coefficients[0],
                TestPolynomial.coefficients[0],
                0.00001);
}

TEST(LineDetect, getLaneIntersectPointCenterTest) {
    std::vector<Polynomial> test_lane_lines;

    Polynomial TestLeftLinePolynomial;
    Polynomial TestRightLinePolynomial;

    TestLeftLinePolynomial.coefficients = {
    -3.50000, 5.08333, -1.25000, 0.16667};
    TestRightLinePolynomial.coefficients = {
    162.50000, -74.83333, 12.00000, -0.66667};

    test_lane_lines = {TestLeftLinePolynomial, TestRightLinePolynomial};

    int test_order = 3;

    LineDetect TestLineDetect;

    cv::Point2d test_intersect_point =
            TestLineDetect.getLaneIntersectPoint(test_lane_lines, test_order);

    cv::Point2d expected_intersect_point = {8.83210, -4.39143};

    EXPECT_NEAR(
    expected_intersect_point.x, test_intersect_point.x, 0.00001);
    EXPECT_NEAR(
    expected_intersect_point.y, test_intersect_point.y, 0.00001);
}

// lane curving right
TEST(LineDetect, fitPolyToLineLeftLineCurveRightTest) {
    cv::Point2d test_point_1{5.0, 0.0};
    cv::Point2d test_point_2{5.2, 1.0};
    cv::Point2d test_point_3{6.0, 2.0};
    cv::Point2d test_point_4{6.5, 3.0};
    cv::Point2d test_point_5{7.4, 4.0};
    cv::Point2d test_point_6{8.7, 5.0};

    std::vector<cv::Point2d> test_points = {test_point_1,
                                            test_point_2,
                                            test_point_3,
                                            test_point_4,
                                            test_point_5,
                                            test_point_6};

    int test_order = 3;

    LineDetect TestLineDetect;

    Polynomial TestPolynomial =
    TestLineDetect.fitPolyToLine(test_points, test_order);

    Polynomial ExpectedPolynomial;
    ExpectedPolynomial.coefficients = {-24.04865, 8.06960, -0.77742, 0.02687};

    EXPECT_NEAR(ExpectedPolynomial.coefficients[3],
                TestPolynomial.coefficients[3],
                0.00001);
    EXPECT_NEAR(ExpectedPolynomial.coefficients[2],
                TestPolynomial.coefficients[2],
                0.00001);
    EXPECT_NEAR(ExpectedPolynomial.coefficients[1],
                TestPolynomial.coefficients[1],
                0.00001);
    EXPECT_NEAR(ExpectedPolynomial.coefficients[0],
                TestPolynomial.coefficients[0],
                0.00001);
}

TEST(LineDetect, fitPolyToLineRightLineCurveRightTest) {
    cv::Point2d test_point_1{12.3, 0.0};
    cv::Point2d test_point_2{11.9, 1.0};
    cv::Point2d test_point_3{11.6, 2.0};
    cv::Point2d test_point_4{11.3, 3.0};
    cv::Point2d test_point_5{11.2, 4.0};
    cv::Point2d test_point_6{11.1, 5.0};

    std::vector<cv::Point2d> test_points = {test_point_1,
                                            test_point_2,
                                            test_point_3,
                                            test_point_4,
                                            test_point_5,
                                            test_point_6};

    int test_order = 3;

    LineDetect TestLineDetect;

    Polynomial TestPolynomial =
    TestLineDetect.fitPolyToLine(test_points, test_order);

    Polynomial ExpectedPolynomial;
    ExpectedPolynomial.coefficients = {
    8234.90373, -2075.85198, 174.61119, -4.90033};

    EXPECT_NEAR(ExpectedPolynomial.coefficients[3],
                TestPolynomial.coefficients[3],
                0.00001);
    EXPECT_NEAR(ExpectedPolynomial.coefficients[2],
                TestPolynomial.coefficients[2],
                0.00001);
    EXPECT_NEAR(ExpectedPolynomial.coefficients[1],
                TestPolynomial.coefficients[1],
                0.00001);
    EXPECT_NEAR(ExpectedPolynomial.coefficients[0],
                TestPolynomial.coefficients[0],
                0.00001);
}

TEST(LineDetect, getLaneIntersectPointRightTest) {
    std::vector<Polynomial> test_lane_lines;

    Polynomial TestLeftLanePolynomial;
    Polynomial TestRightLanePolynomial;

    TestLeftLanePolynomial.coefficients = {
    -24.04865, 8.06960, -0.77742, 0.02687};
    TestRightLanePolynomial.coefficients = {
    -11930.72079, 3176.10932, -281.06283, 8.26972};

    test_lane_lines = {TestLeftLanePolynomial, TestRightLanePolynomial};

    int test_order = 3;

    LineDetect TestLineDetect;

    cv::Point2d test_intersect_point =
            TestLineDetect.getLaneIntersectPoint(test_lane_lines, test_order);

    cv::Point2d expected_intersect_point = {7.89850, -12.55519};

    EXPECT_NEAR(
    expected_intersect_point.x, test_intersect_point.x, 0.00001);
    EXPECT_NEAR(
    expected_intersect_point.y, test_intersect_point.y, 0.00001);
}

// lane curving left in normal perspective view
TEST(LineDetect, fitPolyToLineLeftLineCurveLeftTest) {
    cv::Point2d test_point_1{19.6, 0.0};
    cv::Point2d test_point_2{20.4, 2.0};
    cv::Point2d test_point_3{21.2, 4.0};
    cv::Point2d test_point_4{21.6, 6.0};
    cv::Point2d test_point_5{22.1, 8.0};
    cv::Point2d test_point_6{22.7, 12.0};
    cv::Point2d test_point_7{23.3, 16.0};
    cv::Point2d test_point_8{23.5, 18.0};

    std::vector<cv::Point2d> test_points = {test_point_1,
                                            test_point_2,
                                            test_point_3,
                                            test_point_4,
                                            test_point_5,
                                            test_point_6,
                                            test_point_7,
                                            test_point_8};

    int test_order = 3;

    LineDetect TestLineDetect;

    Polynomial TestPolynomial =
    TestLineDetect.fitPolyToLine(test_points, test_order);

    Polynomial ExpectedPolynomial;
    ExpectedPolynomial.coefficients = {
    -774.72144, 119.41203, -6.23463, 0.11015};

    EXPECT_NEAR(ExpectedPolynomial.coefficients[3],
                TestPolynomial.coefficients[3],
                0.00001);
    EXPECT_NEAR(ExpectedPolynomial.coefficients[2],
                TestPolynomial.coefficients[2],
                0.00001);
    EXPECT_NEAR(ExpectedPolynomial.coefficients[1],
                TestPolynomial.coefficients[1],
                0.00001);
    EXPECT_NEAR(ExpectedPolynomial.coefficients[0],
                TestPolynomial.coefficients[0],
                0.00001);
}

TEST(LineDetect, fitPolyToLineRightLineCurveLeftTest) {
    cv::Point2d test_point_1{34.7, 2.0};
    cv::Point2d test_point_2{34.3, 4.0};
    cv::Point2d test_point_3{32.8, 6.0};
    cv::Point2d test_point_4{32.0, 10.0};
    cv::Point2d test_point_5{31.0, 12.0};
    cv::Point2d test_point_6{30.0, 14.0};
    cv::Point2d test_point_7{28.0, 16.0};
    cv::Point2d test_point_8{27.0, 18.0};

    std::vector<cv::Point2d> test_points = {test_point_1,
                                            test_point_2,
                                            test_point_3,
                                            test_point_4,
                                            test_point_5,
                                            test_point_6,
                                            test_point_7,
                                            test_point_8};

    int test_order = 3;

    LineDetect TestLineDetect;

    Polynomial TestPolynomial =
    TestLineDetect.fitPolyToLine(test_points, test_order);

    // we know that a polynomial function of degree n will have at most n-1
    // relative extrema
    // and we observe in this test how this property can give an undesired best
    // fit polynomial
    Polynomial ExpectedPolynomial;
    ExpectedPolynomial.coefficients = {-371.68524, 37.32498, -1.14044, 0.01082};

    EXPECT_NEAR(ExpectedPolynomial.coefficients[3],
                TestPolynomial.coefficients[3],
                0.00001);
    EXPECT_NEAR(ExpectedPolynomial.coefficients[2],
                TestPolynomial.coefficients[2],
                0.00001);
    EXPECT_NEAR(ExpectedPolynomial.coefficients[1],
                TestPolynomial.coefficients[1],
                0.00001);
    EXPECT_NEAR(ExpectedPolynomial.coefficients[0],
                TestPolynomial.coefficients[0],
                0.00001);
}

TEST(LineDetect, getLaneIntersectPointLeftTest) {
    std::vector<Polynomial> test_lane_lines;

    Polynomial TestLeftLanePolynomial;
    Polynomial TestRightLanePolynomial;

    TestLeftLanePolynomial.coefficients = {
    -774.72144, 119.41203, -6.23463, 0.11015};
    TestRightLanePolynomial.coefficients = {
    -371.68524, 37.32498, -1.14044, 0.01082};

    test_lane_lines = {TestLeftLanePolynomial, TestRightLanePolynomial};

    int test_order = 3;

    LineDetect TestLineDetect;

    cv::Point2d test_intersect_point =
            TestLineDetect.getLaneIntersectPoint(test_lane_lines, test_order);

    cv::Point2d expected_intersect_point = {15.58456, -23.22649};

    EXPECT_NEAR(
    expected_intersect_point.x, test_intersect_point.x, 0.00001);
    EXPECT_NEAR(
    expected_intersect_point.y, test_intersect_point.y, 0.00001);
}

TEST(LineDetect, getLaneIntersectPointNoneTest) {
    std::vector<Polynomial> test_lane_lines;

    Polynomial TestLeftLanePolynomial;
    Polynomial TestRightLanePolynomial;

    TestLeftLanePolynomial.coefficients  = {0, 0, 0, 1};
    TestRightLanePolynomial.coefficients = {-27, 27, -9, 1};

    test_lane_lines = {TestLeftLanePolynomial, TestRightLanePolynomial};

    int test_order = 3;

    LineDetect TestLineDetect;

    try {
        cv::Point2d test_intersect_point =
                TestLineDetect.getLaneIntersectPoint(test_lane_lines, test_order);
    } catch (std::exception& error) {
        EXPECT_EQ(error.what(),
                  std::string("no lane intersects found - frame discarded"));
    }
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}