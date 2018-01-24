/*
 * Created by: Raad Khan
 * Created On: July 1, 2017
 * Description: Header for LineDetect
 */

#ifndef LANE_FOLLOW_LINEDETECT_H
#define LANE_FOLLOW_LINEDETECT_H

#include <opencv2/core.hpp>

using namespace cv;

// Stores polynomial coefficients in increasing order
// eg: for ax^3 + bx^2 + cx + d, coefficients[0] = d, ... coefficients[3] = a
struct Polynomial {
     std::vector<double> coefficients;
 };

// Defines a vector of integer type
typedef std::vector<int> intVec;

// Defines a window slice
class Window {
    public:
    // window parameters
    int center;
    int width;
    // window methods
    int getLeftSide() { return (center - width/2); }
    int getRightSide() { return (center + width/2); }
};

class LineDetect {
public:
    // Constructor
    LineDetect();

    /**
     * Creates lane lines from lane points
     *
     * @param_one left and right lane points
     *
     * @return left and right lane polynomials
     *
     */
    std::vector<Polynomial>
    getLaneLines(std::vector<std::vector<cv::Point2d>> lanePoints);

    /**
     * Creates base windows to contain left and right lanes
     *
     * @param_one filtered image
     *
     * @return left and right base windows
     */
    std::vector<Window> getBaseWindows(cv::Mat &filteredImage);

    /**
     * Creates histogram of ROI/window
     *
     * @param_one ROI (by reference)
     *
     * @return histogram
     */
    intVec getHistogram(cv::Mat &ROI);

    /**
     * Finds base histogram's peak positions
     *
     * This is a good indication of where the lanes are located
     *
     * @param base histogram
     * @return peak location indices
     */
    std::pair<int, int> getBaseHistogramPeakPositions(intVec baseHistogram);

    /**
     * Creates a window by slicing the left/right base window vertically upwards
     *
     * @param_one filtered image
     * @param_two left/right base window
     * @param_three window slice index
     *
     * @return window slice
     */

    cv::Mat getWindowSlice(cv::Mat &filteredImage,
                          Window baseWindow,
                          int verticalSliceIndex);

    /**
     * Finds peak position of lane segment in window
     *
     * @param_one window histogram
     *
     * @return peak location index
     */
    int getWindowHistogramPeakPosition(intVec windowHistogram);

    /**
     * Creates lane points
     *
     * @param_one filtered image
     * @param_two left and right base windows
     *
     * @return left and right lane points
     */
    std::vector<std::vector<cv::Point2d>>
    getLanePoints(cv::Mat &filteredImage, std::vector<Window> baseWindows);

    /**
     * Generates polynomial coefficients representing left/right lane line
     *
     * @param_one left/right lane points
     * @param_two desired order of polynomial
     *
     * @return left/right lane polynomial coefficients
     */
    Polynomial fitPolyToLine(std::vector<cv::Point2d> points, int order);

    /**
     * Gets intersection point of lane lines
     *
     * @param_one left and right lane polynomials
     * @param_two desired order of polynomial
     *
     * @return intersection point
     */
    cv::Point2d getIntersectionPoint(std::vector<Polynomial> laneLines, int order);

private:
    // white color value
    int white;
    // number of window slices
    int numVerticalSlices;
    // lane polynomial degree
    int degree;
    // window width
    int windowWidth;
};

#endif