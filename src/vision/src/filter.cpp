/*
 * Takes in an image and transforms it into a binary image
 * given some color specification.
 * Author: Valerian Ratu
 * Ref: 	Color Picker
 *			https://raw.githubusercontent.com/kylehounslow/opencv-tuts/master/auto-colour-filter/AutoColourFilter.cpp
 *		Color Bar:
 *			http://opencv-srf.blogspot.ca/2010/09/object-detection-using-color-seperation.html
 */

#include <filter.h>


//Two different constructors
snowbotsFilter::snowbotsFilter(){
    createFilter(0, 179, 0, 255, 0, 255);
}

snowbotsFilter::snowbotsFilter(int iLowH, int iHighH, int iLowS, int iHighS, int iLowV, int iHighV){
    createFilter(iLowH, iHighH, iLowS, iHighS, iLowV, iHighV);
}


//Initializer
void snowbotsFilter::createFilter(int iLowH, int iHighH, int iLowS, int iHighS, int iLowV, int iHighV){
    _iLowH = iLowH;
    _iHighH = iHighH;
    _iLowS = iLowS;
    _iHighS = iHighS;
    _iLowV = iLowV;
    _iHighV = iHighV;
    manualCalibrationWindow = "Manual Calibration";
}

//Functions
void snowbotsFilter::manualCalibration(void){
    cv::namedWindow(manualCalibrationWindow, CV_WINDOW_AUTOSIZE);
    cv::createTrackbar("LowH", manualCalibrationWindow, &_iLowH, 179); //Hue (0 - 179)
    cv::createTrackbar("HighH", manualCalibrationWindow, &_iHighH, 179);

    cv::createTrackbar("LowS", manualCalibrationWindow, &_iLowS, 255); //Saturation (0 - 255)
    cv::createTrackbar("HighS", manualCalibrationWindow, &_iHighS, 255);

    cv::createTrackbar("LowV", manualCalibrationWindow, &_iLowV, 255); //Value (0 - 255)
    cv::createTrackbar("HighV", manualCalibrationWindow, &_iHighV, 255);

}

void snowbotsFilter::stopManualCalibration(){
    cv::destroyWindow(manualCalibrationWindow);
}

void snowbotsFilter::filterImage(const cv::Mat &input, cv::Mat &output){

    cv::cvtColor(input, hsvOutput, CV_BGR2HSV, 0);
    cv::inRange(hsvOutput, cv::Scalar(_iLowH, _iLowS, _iLowV), cv::Scalar(_iHighH, _iHighS, _iHighV), rangeOutput);

    //Morphological Opening (removes small objects from foreground)
    cv::erode(rangeOutput, rangeOutput, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5,5)));
    cv::dilate(rangeOutput, rangeOutput, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5,5)));

    //Morphological Closing (fill small holes in the foreground)
    cv::dilate(rangeOutput, rangeOutput, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) ); 
    cv::erode(rangeOutput, rangeOutput, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) ); 

    rangeOutput.copyTo(output);
}

void snowbotsFilter::printValues(void){
    std::cout << "iLowH: " << _iLowH << std::endl;
    std::cout << "iHighH: " << _iHighH << std::endl;
    std::cout << "iLowS: " << _iLowS << std::endl;
    std::cout << "iHighS: " << _iHighS << std::endl;
    std::cout << "iLowV: " << _iLowV << std::endl;
    std::cout << "iHighV: " << _iHighV << std::endl;
}

std::string snowbotsFilter::getValues(void){
    std::stringstream values;
    values << _iLowH << " " << _iHighH << " "
        << _iLowS << " " << _iHighS << " "
        << _iLowV << " " << _iHighV << "\n";
    return values.str();
}

