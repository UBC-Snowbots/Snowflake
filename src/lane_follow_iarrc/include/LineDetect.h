/*
 * Created by: Raad Khan
 * Created On: July 1, 2017
 * Description: Detects lane lines and generates destination point.
 * Usage: LaneFollow node instantiates this helper class.
 */

#ifndef LANE_FOLLOW_LINEDETECT_H
#define LANE_FOLLOW_LINEDETECT_H

#include <exception>
#include <iostream>
#include <opencv2/core.hpp>

using namespace cv;

// Stores polynomial coefficients in increasing order
// eg: ax^3 + bx^2 + cx + d
// coefficients[0] = d, coefficients[1] = c, ... ,coefficients[3] = a
struct Polynomial {
    std::vector<double> coefficients;
};

// Defines a vector of integer type
typedef std::vector<int> int_vec;

// Defines a window slice
struct Window {
    // window parameters
    int center;
    int width;
    // window methods
    int getLeftSide() { return (center - width / 2); }
    int getRightSide() { return (center + width / 2); }
};

class LineDetect {
  public:
    // Constructor
    LineDetect();

    /**
     * Gets the intersection point of the lane lines
     *
     * @param_one left and right lane line polynomials
     * @param_two order of the polynomials
     *
     * @return lane intersection point
     */
    cv::Point2d getLaneIntersectPoint(std::vector<Polynomial> lane_lines,
                                      int order);

    /**
     * Creates lane lines from lane points
     *
     * @param_one left and right lane points
     *
     * @return left and right lane line polynomials
     *
     */
    std::vector<Polynomial>
    getLaneLines(std::vector<std::vector<cv::Point2d>> lane_points);

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
     * Creates lane points which model lane lines in the filtered image
     *
     * @param_one filtered image
     * @param_two left and right base windows
     *
     * @return left and right lane points
     */
    std::vector<std::vector<cv::Point2d>>
    getLanePoints(cv::Mat &filtered_image);

    /**
     * Creates two base windows to cover left and right lanes
     *
     * @param_one filtered image
     *
     * @return left and right base windows
     */
    std::vector<Window> getBaseWindows(cv::Mat &filtered_image);

    /**
     * Creates a histogram of region of interest (ROI)
     *
     * Looks for white pixel density
     *
     * @param_one ROI
     *
     * @return histogram
     */
    int_vec getHistogram(cv::Mat &ROI);

    /**
     * Finds the base histogram's peak positions
     *
     * The columns with the biggest peaks are a
     * good indication of where the lane lines are
     *
     * @param base histogram
     * @return peak location indices
     */
    std::pair<int, int> getBaseHistogramPeakPositions(int_vec base_histogram);

    /**
     * Creates a window by slicing the left/right base window bottom up
     *
     * @param_one filtered image
     * @param_two left/right base window
     * @param_three window slice index
     *
     * @return window slice
     */
    cv::Mat getWindowSlice(cv::Mat &filtered_image,
                           Window BaseWindow,
                           int vertical_slice_index);

    /**
     * Finds the peak of a window's histogram
     *
     * @param_one window's histogram
     *
     * @return peak column position
     */
    int getWindowHistogramPeakPosition(int_vec window_histogram);

    // Exception class to throw when no intersect roots exist
    class NoLaneIntersectException : public std::exception {
        virtual const char* what() const throw() {
            return "no lane intersects found - frame discarded";
        }
    };

    // white color value
    int white;

    // number of window slices
    int vertical_slices;

    // lane polynomial degree
    int degree;

    // window width
    int window_width;
};

#endif