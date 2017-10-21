/*
 * Created by: Raad Khan
 * Created On: July 1, 2017
 * Description: Takes in an image feed and generates lane lines.
 * Usage: LaneFollow node instantiates this class to generate lane lines
 */

#include <LineDetect.h>
#include <Eigen/QR>
#include <opencv2/objdetect/objdetect.hpp>

using namespace cv;

LineDetect::LineDetect() : white(250),
                           numVerticalSlice(10),
                           degree(1) { }

std::vector<Polynomial> LineDetect::getLaneLines(std::vector <std::vector<cv::Point2d>> lanePoints) {

    std::vector<Polynomial> laneLines;
    Polynomial polyPoints;

    for (std::vector<cv::Point2d> points : lanePoints) {
        polyPoints = this->fitPolyLine(points, degree);
        laneLines.emplace_back(polyPoints);
    }

    return laneLines;
}

std::vector<Window> LineDetect::getBaseWindows(cv::Mat& filteredImage) {

    windowWidth = filteredImage.cols / 4;

    intVec baseHistogram = this->getHistogram(filteredImage);
    std::pair<int, int> peak = this->getBaseHistogramPeakPosition(baseHistogram);

    std::vector<Window> baseWindows;
    Window windowLeft{peak.first, windowWidth};
    Window windowRight{peak.second, windowWidth};
    baseWindows.emplace_back(windowLeft);
    baseWindows.emplace_back(windowRight);

    return baseWindows;
}

intVec LineDetect::getHistogram(cv::Mat &image) {

    intVec histogram(image.cols, 0);

    for (int j = 0; j < image.rows; j++) {
        for (int i = 0; i < image.cols; i++) {
            int pixelValue = image.at<uchar>(j, i);
            if (pixelValue >= white) {
                histogram[i]++;
            }
        }
    }

    return histogram;
}

std::pair<int, int> LineDetect::getBaseHistogramPeakPosition(intVec histogram) {

    std::pair<int, int> peak(0, 0);
    int peakValue = 0;

    for (int i = 0; i < (int) (histogram.size() / 2.0); i++) {
        if (histogram[i] > peakValue) {
            peakValue = histogram[i];
            peak.first = i;
        }
    }

    peakValue = 0;

    for (int i = (int) (histogram.size() / 2.0); i < histogram.size(); i++) {
        if (histogram[i] > peakValue) {
            peakValue = histogram[i];
            peak.second = i;
        }
    }

    return peak;
}

std::vector <std::vector<cv::Point2d>> LineDetect::getLanePoints(cv::Mat &filteredImage, std::vector<Window> windows) {

    std::vector <std::vector<cv::Point2d>> lanePoints(windows.size(), std::vector<cv::Point2d>());

    for (int verticalSliceIndex = 0; verticalSliceIndex < numVerticalSlice; verticalSliceIndex++) {
        for (int windowIndex = 0; windowIndex < windows.size(); windowIndex++) {
            Window window = windows.at(windowIndex);
            cv::Mat windowSlice = this->getWindowSlice(filteredImage, window, verticalSliceIndex);
            intVec windowHistogram = this->getHistogram(windowSlice);
            int peak = this->getWindowHistogramPeakPosition(windowHistogram);
            window.center = peak - windowSlice.cols/2 + window.center;
            cv::Point2d point{
                    (double) window.center,
                    (double) (verticalSliceIndex * filteredImage.rows / numVerticalSlice)
            };
            lanePoints[windowIndex].push_back(point);
        }
    }

    return lanePoints;
}

cv::Mat LineDetect::getWindowSlice(cv::Mat& filteredImage, Window window, int verticalSliceIndex) {

    cv::Mat windowSlice = filteredImage(
            Range(verticalSliceIndex * filteredImage.rows / numVerticalSlice,
                  (verticalSliceIndex + 1) * filteredImage.rows / numVerticalSlice),
            Range(window.getLeftSide(), window.getRightSide()));

    return windowSlice;
}

int LineDetect::getWindowHistogramPeakPosition(intVec histogram) {

    int peak = 0;
    int peakValue = 0;

    for (int i = 0; i < (int) histogram.size(); i++) {
        if (histogram[i] > peakValue) {
            peakValue = histogram[i];
            peak = i;
        }
    }

    return peak;
}

Polynomial LineDetect::fitPolyLine(std::vector<cv::Point2d> points, int order) {

    int moreOrder = order + 1;
    assert(points.size() >= moreOrder);
    assert(order <= 3);

    std::vector<double> xv(points.size(), 0);
    std::vector<double> yv(points.size(), 0);

    for (size_t i = 0; i < points.size(); i++) {
        xv[i] = points[i].x;
        yv[i] = points[i].y;
    }

    Eigen::MatrixXd A(xv.size(), moreOrder);
    Eigen::VectorXd yvMapped = Eigen::VectorXd::Map(&yv.front(), yv.size());
    Eigen::VectorXd result;

    // create matrix
    for (size_t i = 0; i < points.size(); i++)
        for (size_t j = 0; j < moreOrder; j++)
            A(i, j) = pow((xv.at(i)), j);

    // solve for linear least squares fit
    result = A.householderQr().solve(yvMapped);

    // 3rd order
    if (result.size() == 4)
        return Polynomial{result[3], result[2], result[1], result[0]};
    // 2nd order
    else if (result.size() == 3)
        return Polynomial{0, result[2], result[1], result[0]};
    // 1st order
    else
        return Polynomial{0, 0, result[1], result[0]};
}

cv::Point2d LineDetect::getIntersection(Polynomial leftLine, Polynomial rightLine) {

    // Isolate slopes
    double combinedSlope = leftLine.c - rightLine.c;

    // Isolate y-intercepts
    double combinedYIntercept = rightLine.d - leftLine.d;

    // Solve for x
    double x = combinedYIntercept / combinedSlope;

    // Solve for y
    double y = leftLine.c * x + leftLine.d;

    cv::Point2d point;
    point.x = x;
    point.y = y;

    return point;
}

double LineDetect::getAngleFromOriginToPoint(cv::Point2d point) {

    double dy = point.y;
    double dx = point.x;

    double angle = atan(dy / dx);


    // If the endpoint is behind and to the left
    if (dx < 0 && dy > 0) angle += M_PI;
        // If the endpoint is behind and to the right
    else if (dx < 0 && dy < 0) angle -= M_PI;

    return angle;
}

cv::Point2d LineDetect::moveAwayFromLine(Polynomial line, double distanceAwayFromRobot, double distanceAwayFromLine) {

    cv::Point2d targetPoint;

    // Get parallel line closer to the middle of the course.
    Polynomial targetParallelLine;
    targetParallelLine.c = line.c;
    targetParallelLine.d = line.d - distanceAwayFromLine * line.d / fabs(line.d);

    cv::Point2d perpendicularIntersection = getPerpendicularIntersection(targetParallelLine);

    double distanceToPerpendicularIntersection = sqrt(
            pow(perpendicularIntersection.x, 2) + pow(perpendicularIntersection.y, 2));

    // cos(theta) = A/H
    double scalarProjectionAngle = acos(distanceToPerpendicularIntersection / distanceAwayFromRobot);

    // tan(theta) = O/A
    double anglePerpendicularIntersection = atan(perpendicularIntersection.x / perpendicularIntersection.y);

    // anglePerpendicularIntersection + angleHeading = scalarProjectionAngle
    double angleHeading = scalarProjectionAngle - anglePerpendicularIntersection;

    // Polar coordinates
    targetPoint.x = distanceAwayFromRobot * cos(angleHeading);
    targetPoint.y = distanceAwayFromRobot * sin(angleHeading);

    return targetPoint;
}

cv::Point2d LineDetect::getPerpendicularIntersection(Polynomial line) {

    cv::Point2d perpendicularIntersection;

    double negReciprocal = -1.0 / line.c;

    // Set the two sides equal then isolate x to one side.
    double isolatedXSlope = negReciprocal - line.c;

    // Divide both sides by the isolated slope to get the x point intersection.
    perpendicularIntersection.x = line.d / isolatedXSlope;

    // Plug in the xIntersection to get the y point intersection.
    perpendicularIntersection.y = negReciprocal * perpendicularIntersection.x;

    return perpendicularIntersection;
}