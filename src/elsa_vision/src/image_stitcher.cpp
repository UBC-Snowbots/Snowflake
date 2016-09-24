#include <opencv2/core/core.hpp>
#include <stdio.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/stitching.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <vector>


using namespace std;
using namespace cv;

Mat _left;
Mat _right;

void leftCallback(const sensor_msgs::ImageConstPtr& msg){
    try
    {
        _left = cv_bridge::toCvShare(msg, "bgr8")->image;
    }
    catch (cv_bridge::Exception e)
    {
        ROS_ERROR("Cannot convert '%s' to 'bgr8'", msg->encoding.c_str());
    }
    //ROS_INFO("Recieved left image");
}


void rightCallback(const sensor_msgs::ImageConstPtr& msg){
    try
    {
        _right = cv_bridge::toCvShare(msg, "bgr8")->image;
    }
    catch (cv_bridge::Exception e)
    {
        ROS_ERROR("Cannot convert '%s' to 'bgr8'", msg->encoding.c_str());
    }
    //ROS_INFO("Recieved right image");
}

int main(int argc, char** argv){
    _left = Mat::zeros(480, 640, CV_8UC4);
    _right = Mat::zeros(480, 640, CV_8UC4);
    string outputWindow = "output";
    string leftWindow = "left";
    string rightWindow = "right";    

    ros::init(argc, argv, "image_stitcher_node");
    ros::NodeHandle nh;
    Mat output;
    namedWindow("output");
    
    Stitcher stitcher = Stitcher::createDefault(true);
    
    bool transformSet = false;
 
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber left_sub = it.subscribe("/left_camera/image_raw", 2, leftCallback);
    image_transport::Subscriber right_sub = it.subscribe("/right_camera/image_raw", 2, rightCallback);    
    ros::Rate loop_rate(100);

    while(nh.ok()){
        vector<Mat> images;
        images.push_back(_left);
        images.push_back(_right);
        /*if (!transformSet){
            Stitcher::Status transform_status = stitcher.estimateTransform(images);
            if (transform_status == Stitcher::OK){
                transformSet = true;
                ROS_INFO("Transform Obtained");
            }
        }*/


                
        Stitcher::Status status = stitcher.stitch(images, output);
        images.clear();

        if (status != Stitcher::OK){
            ROS_ERROR("Unable to stitch");
        } else {
            imshow(outputWindow, output);
        }
        //imshow(leftWindow, _left);
        //imshow(rightWindow, _right);
        if (waitKey(10) == 27){
            break;
        }
        loop_rate.sleep();
        ros::spinOnce();
    }
} 
