/*
 * Takes in an image and transforms it into a binary image
 * given some color specification.
 * Author: Valerian Ratu
 * Ref: 	Color Picker
 *			https://raw.githubusercontent.com/kylehounslow/opencv-tuts/master/auto-colour-filter/AutoColourFilter.cpp
 *			Color Bar:
 *			http://opencv-srf.blogspot.ca/2010/09/object-detection-using-color-seperation.html
 */

#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <stdio.h>

class HSVFilter {
    // Thresholds
    int _iLowH;
    int _iHighH;
    int _iLowS;
    int _iHighS;
    int _iLowV;
    int _iHighV;

    cv::Mat rangeOutput;
    cv::Mat hsvOutput;

    // Window Names
    std::string manualCalibrationWindow;

  public:
    /**
     * Initializes the filter with known values
     */
    HSVFilter(
    int iLowH, int iHighH, int iLowS, int iHighS, int iLowV, int iHighV);

    /**
     * Initializes the filter with default values
     */
    HSVFilter(void);

    /**
     * Filters an image according to threshold values
     *
     * @param input the frame being filtered
     * @param output the output image
     */
    void filterImage(const cv::Mat& input, cv::Mat& output);

    /**
     * Enables manual calibration of HSV values
     */
    void manualCalibration(void);

    /**
     * Ends the manual calibration of HSV values
     */
    void stopManualCalibration(void);

    /**
     * Prints the HSV values into the screen
     */
    void printValues(void);

    /**
     * Getter for the HSV values in the filter
     * @return a string of the HSV values in the filter formatted as:
     *             lh, hh, ls, hs, lv, hv
     */
    std::string getValues(void);

  private:
    /**
     * Initializator
     *
     * @params the appropriate HSV ranges
     */
    void createFilter(
    int iLowH, int iHighH, int iLowS, int iHighS, int iLowV, int iHighV);
};