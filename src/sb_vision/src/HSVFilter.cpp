/*
 * Takes in an image and transforms it into a binary image
 * given some color specification.
 * Author: Valerian Ratu
 * Ref: 	Color Picker
 *			https://raw.githubusercontent.com/kylehounslow/opencv-tuts/master/auto-colour-filter/AutoColourFilter.cpp
 *		Color Bar:
 *			http://opencv-srf.blogspot.ca/2010/09/object-detection-using-color-seperation.html
 */

#include <HSVFilter.h>

// Two different constructors
HSVFilter::HSVFilter() {
    int sensitivity = 30;
    createFilter(60 - sensitivity, 60 + sensitivity, 100, 255, 100, 255);
}

HSVFilter::HSVFilter(
int iLowH, int iHighH, int iLowS, int iHighS, int iLowV, int iHighV) {
    createFilter(iLowH, iHighH, iLowS, iHighS, iLowV, iHighV);
}

// Initializer
void HSVFilter::createFilter(
int iLowH, int iHighH, int iLowS, int iHighS, int iLowV, int iHighV) {
    _iLowH                  = iLowH;
    _iHighH                 = iHighH;
    _iLowS                  = iLowS;
    _iHighS                 = iHighS;
    _iLowV                  = iLowV;
    _iHighV                 = iHighV;
    manualCalibrationWindow = "Manual Calibration";
}

// Functions
void HSVFilter::manualCalibration(void) {
    cv::namedWindow(manualCalibrationWindow, CV_WINDOW_AUTOSIZE);
    cv::createTrackbar(
    "LowH", manualCalibrationWindow, &_iLowH, 179); // Hue (0 - 179)
    cv::createTrackbar("HighH", manualCalibrationWindow, &_iHighH, 179);

    cv::createTrackbar(
    "LowS", manualCalibrationWindow, &_iLowS, 255); // Saturation (0 - 255)
    cv::createTrackbar("HighS", manualCalibrationWindow, &_iHighS, 255);

    cv::createTrackbar(
    "LowV", manualCalibrationWindow, &_iLowV, 255); // Value (0 - 255)
    cv::createTrackbar("HighV", manualCalibrationWindow, &_iHighV, 255);
}

void HSVFilter::stopManualCalibration() {
    cv::destroyWindow(manualCalibrationWindow);
}

void HSVFilter::filterImage(const cv::Mat& input, cv::Mat& output) {
    cv::cvtColor(input, hsvOutput, CV_BGR2HSV, 0);
    cv::inRange(hsvOutput,
                cv::Scalar(_iLowH, _iLowS, _iLowV),
                cv::Scalar(_iHighH, _iHighS, _iHighV),
                rangeOutput);

    // Determines how many times to erode then dilate
    int noise_filter_count = 5;
    for (int i = 0; i < noise_filter_count; i++) {
        cv::Size size = cv::Size(2, 2);
        // Morphological Opening (removes small objects from foreground)
        cv::erode(rangeOutput,
                  rangeOutput,
                  getStructuringElement(cv::MORPH_ELLIPSE, size));
        cv::dilate(rangeOutput,
                   rangeOutput,
                   getStructuringElement(cv::MORPH_ELLIPSE, size));

        cv::Size size2 = cv::Size(10, 10);
        // Morphological Closing (fill small holes in the foreground)
        cv::dilate(rangeOutput,
                   rangeOutput,
                   getStructuringElement(cv::MORPH_ELLIPSE, size2));
        cv::erode(rangeOutput,
                  rangeOutput,
                  getStructuringElement(cv::MORPH_ELLIPSE, size2));
    }
    rangeOutput.copyTo(output);
}

void HSVFilter::printValues(void) {
    std::cout << "iLowH: " << _iLowH << std::endl;
    std::cout << "iHighH: " << _iHighH << std::endl;
    std::cout << "iLowS: " << _iLowS << std::endl;
    std::cout << "iHighS: " << _iHighS << std::endl;
    std::cout << "iLowV: " << _iLowV << std::endl;
    std::cout << "iHighV: " << _iHighV << std::endl;
}

std::string HSVFilter::getValues(void) {
    std::stringstream values;
    values << _iLowH << " " << _iHighH << " " << _iLowS << " " << _iHighS << " "
           << _iLowV << " " << _iHighV << "\n";
    return values.str();
}