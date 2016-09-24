/*
 * Analyzes an image and transforms it into a binary image
 * given some sort of colour specifiction
 * 
 * Current method: Conversion to HSV (Hue, Saturation, Value) and filtering
 * the numbers appropriately. Currently trying out methods to find 
 * which HSV thresholds are the best and see if it can't be
 * automated properly.
 * Ref: https://raw.githubusercontent.com/kylehounslow/opencv-tuts/master/auto-colour-filter/AutoColourFilter.cpp
 *          - Just the HSV value finding rectangle part, not the object tracking part.
 *          - Sketchy, works better if you make a rectangle on the ground
 * Usage: Point to a video file/camera, press m to start/stop calibrating.
 *
 * Subscribes to: /camera/image_raw
 * Publishes to: image
 */   

#include <ros/ros.h>
#include <opencv2/core/core.hpp>
#include <stdio.h>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <opencv2/imgproc/imgproc.hpp>
#include "filter.h"
#include "IPM.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>


using namespace cv;
using namespace std;


Mat inputImage;

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    inputImage = cv_bridge::toCvShare(msg, "bgr8")->image;
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}
  
int main(int argc, char** argv){
    
    //Window Names
    string inputWindow = "Input Image";
    string ipmOutputWindow = "IPM Output";
    string filterOutputWindow = "Filter Output";
    namedWindow(inputWindow, CV_WINDOW_AUTOSIZE);
    namedWindow(ipmOutputWindow, CV_WINDOW_AUTOSIZE);
    namedWindow(filterOutputWindow, CV_WINDOW_AUTOSIZE);

    //Calibration Variables
    bool isCalibratingManually = false;

    //Working Variables
    Mat ipmOutput;
    Mat filterOutput;
    Mat workingInput;
    inputImage = Mat::zeros(480, 640, CV_32FC4);
    cout << inputImage.depth() << endl;

    //ROS
    ros::init(argc, argv, "vision_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    string image_topic;
    nh_private.param<std::string>("image_topic", image_topic, "/camera/image_raw");

    string output_topic;
    nh_private.param<std::string>("output_topic", output_topic, "image");

    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe(image_topic, 1, imageCallback);
    image_transport::Publisher pub = it.advertise(output_topic, 1);
    ros::Rate loop_rate(5); 
   
    //Obtains parameters from the param server 
    int width, height;
    nh_private.param("width", width, 640);
    nh_private.param("height", height, 480);

    int x1,x2,x3,x4,y1,y2,y3,y4;
    nh_private.param("x1", x1, 0);
    nh_private.param("x2", x2, width);
    nh_private.param("x3", x3, width/2+132);
    nh_private.param("x4", x4, width/2-132);
    nh_private.param("y1", y1, height);
    nh_private.param("y2", y2, height);
    nh_private.param("y3", y3, 0);
    nh_private.param("y4", y4, 0);

    
    cout << "Frame size: " << width << " x " << height << endl;


    //Manually Setting up the IPM points
    vector<Point2f> origPoints;
    origPoints.push_back( Point2f(x1, y1));
    origPoints.push_back( Point2f(x2, y2));
    origPoints.push_back( Point2f(x3, y3));
    origPoints.push_back( Point2f(x4, y4));

    vector<Point2f> dstPoints;
    dstPoints.push_back( Point2f(0, height) );
    dstPoints.push_back( Point2f(width, height) );
    dstPoints.push_back( Point2f(width, 0) );
    dstPoints.push_back( Point2f(0, 0) );
    
    //Creating the binary filter
    snowbotsFilter filter(0, 155, 0, 155, 150, 255);

    //Creating the IPM transformer
    IPM ipm(Size(width, height), Size(width,height), origPoints, dstPoints);

    while(nh.ok()){ //rosOK

        //Image is empty so quit
        if (inputImage.empty()) break;
        inputImage.copyTo(workingInput);
        //Applies the IPM to the image
                
        ipm.applyHomography(workingInput, ipmOutput);
        ipm.drawPoints(origPoints, workingInput);
        imshow(inputWindow, workingInput);
        imshow(ipmOutputWindow, ipmOutput);

        //Applies the filter to the image
        filter.filterImage(ipmOutput, filterOutput);
        imshow(filterOutputWindow, filterOutput);
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", filterOutput).toImageMsg();
        pub.publish(msg);        

        //Calibration
        if (isCalibratingManually){
            filter.manualCalibration();
        } else {
            filter.stopManualCalibration();
        }

        //Escape key to finish program
        int a = waitKey(20);
        if (a == 27){
            cout << "Escaped by user" << endl;
            break;
        }
        //Press 'm' to calibrate manually
        else if (a == 109){ 
            if (!isCalibratingManually){
                cout << "Beginning manual calibration" << endl;     
            } else {
                cout << "Ending manual calibration" << endl;
            }
            isCalibratingManually = !isCalibratingManually;
            filter.printValues();
        }

        loop_rate.sleep();
        ros::spinOnce();
    }
    return 0;
}
