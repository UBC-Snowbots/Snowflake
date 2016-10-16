/*
 * Takes in an image and transforms it into a binary image
 * given some color specification.
 * Author: Valerian Ratu
 * Ref: 	Color Picker
 *			https://raw.githubusercontent.com/kylehounslow/opencv-tuts/master/auto-colour-filter/AutoColourFilter.cpp
 *			Color Bar:
 *			http://opencv-srf.blogspot.ca/2010/09/object-detection-using-color-seperation.html
 */

#include <opencv2/core/core.hpp>
#include <stdio.h>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <opencv2/imgproc/imgproc.hpp>

class snowbotsFilter
{	
    //Thresholds
    int _iLowH;
    int _iHighH;
    int _iLowS;
    int _iHighS;
    int _iLowV;
    int _iHighV;

    //Calibration
    bool calibrationMode;
    cv::Point initialClickPoint, currentMousePoint;
    cv::Rect rectangleROI;
    std::vector<int> H_ROI, S_ROI, V_ROI;

    cv::Mat calibrationImage;
    cv::Mat hsv_calibrationImage;
    cv::Mat rangeOutput;
    cv::Mat hsvOutput;

    //Window Names
    std::string manualCalibrationWindow;
    std::string calibrationWindow;
    std::string imageCalibration; 

    public:

    /**
     * Initializes the filter with known values
     */
    snowbotsFilter( int iLowH, int iHighH, 
                    int iLowS, int iHighS, 
                    int iLowV, int iHighV);

    /**
     * Initializes the filter with default values
     */
    snowbotsFilter(void);

    /**
     * Opens the calibration window where a region of interest (ROI) rectangle
     * can be drawn
     * @param input the frame we want to calibrate from
     */
    //void calibrateWindow(const cv::Mat &input);

    /**
     * Filters an image according to threshold values
     * @param input the frame being filtered
     * @param output the output image
     */
    void filterImage(const cv::Mat &input, cv::Mat &output);

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

    //static void onMouse( int event, int x, int y, int f, void* param);
    /**
     * A mouse event handler for the automatic calibration
     */
    //static void clickAndDrag_Rectangle(int event, int x, int y, int flags, void* param);

    private:

    /**
     * Initializator
     * @params the appropriate HSV ranges
     */
    void createFilter(int iLowH, int iHighH,
                    int iLowS, int iHighS, 
                    int iLowV, int iHighV);


    /**
     * Takes records the HSV values from a given frame
     */
    //void recordHSV_Values();
};
