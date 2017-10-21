/*
 * Created by: Raad Khan
 * Created On: July 1, 2017
 * Description: Takes in an image feed and generates lane lines.
 * Usage: LaneFollow node instantiates this class to generate lane lines
 */

#ifndef LANE_FOLLOW_LINEDETECT_H
#define LANE_FOLLOW_LINEDETECT_H

#include <opencv2/core/core.hpp>
#include <iostream>
#include <vector>
#include <string>
#include <stdio.h>

using namespace cv;

struct Polynomial {
    // ax^3 + bx^2 + cx + d
    double a;
    double b;
    double c;
    double d;
};

typedef std::vector<int> intVec;

class Window {

public:
    int center;
    int width;

    int getLeftSide() {
        return (center - width/2);
    }
    int getRightSide() {
        return (center + width/2);
    }
};

class LineDetect {

public:
    /**
     * Constructor
     */
    LineDetect();

    // TODO: doc functions

    std::vector<Window> getBaseWindows(cv::Mat& filteredImage);

    intVec getHistogram(cv::Mat& image);

    std::pair<int, int> getBaseHistogramPeakPosition(intVec histogram);

    std::vector <std::vector<cv::Point2d>> getLanePoints(cv::Mat& filteredImage, std::vector<Window> windows);

    cv::Mat getWindowSlice(cv::Mat& filteredImage, Window window, int verticalSliceIndex);

    int getWindowHistogramPeakPosition(intVec histogram);

    std::vector<Polynomial> getLaneLines(std::vector <std::vector<cv::Point2d>> lanePoints);

    Polynomial fitPolyLine(std::vector<cv::Point2d> points, int order);

    static cv::Point2d getIntersection(Polynomial leftLine, Polynomial rightLine);

    static cv::Point2d getPerpendicularIntersection(Polynomial line);

    static double getAngleFromOriginToPoint(cv::Point2d point);

    static cv::Point2d moveAwayFromLine(Polynomial line, double targetXDistance, double targetYDistance);

private:
    int white;

    int windowWidth;

    int numVerticalSlice;

    int degree;
};

#endif