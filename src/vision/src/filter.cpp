/*
 * Takes in an image and transforms it into a binary image
 * given some color specification.
 * Author: Valerian Ratu
 * Ref: 	Color Picker
 *			https://raw.githubusercontent.com/kylehounslow/opencv-tuts/master/auto-colour-filter/AutoColourFilter.cpp
 *		Color Bar:
 *			http://opencv-srf.blogspot.ca/2010/09/object-detection-using-color-seperation.html
 */

#include "filter.h"


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
    calibrationWindow = "Frame Calibration";
    imageCalibration = "Image Calibration";

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

/* Put on backlog, automatic color analyze and filter
   static void onMouse( int event, int x, int y, int f, void* param){
   cv::Mat src = *((cv::Mat*) param);
   cv::Mat image = src.clone();
   cv::Vec3b hsv = image.at<cv::Vec3b>(y,x);
   int H=hsv.val[0];
   int S=hsv.val[1];
   int V=hsv.val[2];

   char name[30];
   sprintf(name,"H=%d",H);
   cv::putText(image,name, cv::Point(25,40) , CV_FONT_HERSHEY_SIMPLEX, .7, cv::Scalar(0,255,0), 2,8,false );

   sprintf(name,"S=%d",S);
   cv::putText(image,name, cv::Point(25,80) , CV_FONT_HERSHEY_SIMPLEX, .7, cv::Scalar(0,255,0), 2,8,false );

   sprintf(name,"V=%d",V);
   cv::putText(image,name, cv::Point(25,120) , CV_FONT_HERSHEY_SIMPLEX, .7, cv::Scalar(0,255,0), 2,8,false );

   sprintf(name,"X=%d",x);
   cv::putText(image,name, cv::Point(25,300) , CV_FONT_HERSHEY_SIMPLEX, .7, cv::Scalar(0,0,255), 2,8,false );

   sprintf(name,"Y=%d",y);
   cv::putText(image,name, cv::Point(25,340) , CV_FONT_HERSHEY_SIMPLEX, .7, cv::Scalar(0,0,255), 2,8,false );

//imwrite("hsv.jpg",image);
cv::imshow("imageCalibration", image);
}


static void snowbotsFilter::clickAndDrag_Rectangle(int event, int x, int y, int flags, void* param){
if (event == cv::EVENT_LBUTTONDOWN)
{
//keep track of initial point clicked
initialClickPoint = cv::Point(x, y);
}
if (event == cv::EVENT_LBUTTONUP)
{
currentMousePoint = cv::Point(x,y);
//set rectangle ROI to the rectangle that the user has selected
rectangleROI = cv::Rect(initialClickPoint, currentMousePoint);
recordHSV_Values();
}
}

void snowbotsFilter::recordHSV_Values(){
if (H_ROI.size()>0) H_ROI.clear();
if (S_ROI.size()>0) S_ROI.clear();
if (V_ROI.size()>0 )V_ROI.clear();
//if the rectangle has no width or height (user has only dragged a line) then we don't try to iterate over the width or height
if (rectangleROI.width<1 || rectangleROI.height<1){
std::cout << "Please drag a rectangle, not a line" << std::endl;
}
else{
for (int i = rectangleROI.x; i<rectangleROI.x + rectangleROI.width; i++){
//iterate through both x and y direction and save HSV values at each and every point
for (int j = rectangleROI.y; j<rectangleROI.y + rectangleROI.height; j++){
//save HSV value at this point
H_ROI.push_back((int)hsv_calibrationImage.at<cv::Vec3b>(j, i)[0]);
S_ROI.push_back((int)hsv_calibrationImage.at<cv::Vec3b>(j, i)[1]);
V_ROI.push_back((int)hsv_calibrationImage.at<cv::Vec3b>(j, i)[2]);
}
}
}
if (H_ROI.size()>0){
//NOTE: min_element and max_element return iterators so we must dereference them with "*"
_iLowH = *std::min_element(H_ROI.begin(), H_ROI.end());
_iHighH = *std::max_element(H_ROI.begin(), H_ROI.end());
}
if (S_ROI.size()>0){
_iLowS = *std::min_element(S_ROI.begin(), S_ROI.end());
_iHighS = *std::max_element(S_ROI.begin(), S_ROI.end());
}
if (V_ROI.size()>0){
    _iLowV = *std::min_element(V_ROI.begin(), V_ROI.end());
    _iHighV = *std::max_element(V_ROI.begin(), V_ROI.end());
}

}

void snowbotsFilter::calibrateWindow(const cv::Mat &input){
    calibrationImage = input.clone();
    cv::cvtColor(calibrationImage, hsv_calibrationImage, CV_BGR2HSV);
    cv::namedWindow(calibrationWindow, CV_WINDOW_AUTOSIZE);
    //cv::setMouseCallback(calibrationWindow, clickAndDrag_Rectangle);
}
*/
