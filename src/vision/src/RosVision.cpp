/*
 * Created by: Valerian Ratu
 * Created On: October 15, 2016
 * Description: Subscribes to image from a camera and publishes a binary image
 *              depending on the filter specification and setting
 * Usage: Press m to calibrate filter values, press m to save it.
 *
 */

#include <RosVision.h>
#include <iostream>
#include <fstream>

using namespace cv;
using namespace std;

void RosVision::imageCallback(const sensor_msgs::ImageConstPtr &msg) {
    // Check if this is the first image we've received
    // If it is, get parameters, or create sane defaults from the image
    // then, create the IPM transformer
    if (!receivedFirstImage){
        ROS_INFO("First image received!");
        receivedFirstImage = true;
        ros::NodeHandle nh_private("~");
        float ipm_base_width, ipm_top_width, ipm_base_displacement, ipm_top_displacement;
        //Obtains parameters of image and IPM points from the param server
        SB_getParam(nh_private, "width", width, (int)msg->width);
        SB_getParam(nh_private, "height", height, (int)msg->height);
        SB_getParam(nh_private, "ipm_base_width", ipm_base_width, (float)1);
        SB_getParam(nh_private, "ipm_top_width", ipm_top_width, (float)0.5);
        SB_getParam(nh_private, "ipm_base_displacement", ipm_base_displacement, (float)0);
        SB_getParam(nh_private, "ipm_top_displacement", ipm_top_displacement, (float)0.25);
        x1 = width/2 - ipm_base_width/2 * width;
        y1 = (1 - ipm_base_displacement) * height;
        x2 = width/2 + ipm_base_width/2 * width;
        y2 = (1 - ipm_base_displacement) * height;
        x3 = width/2 - ipm_top_width/2 * width;
        y3 = height * ipm_top_displacement;
        x4 = width/2 - ipm_top_width/2 * width;
        y4 = height * ipm_top_displacement;

        ROS_INFO("Image Width and Height: %d x %d", width, height);

        //Set up the IPM points
        orig_points.push_back(Point2f(x1, y1));
        orig_points.push_back(Point2f(x2, y2));
        orig_points.push_back(Point2f(x3, y3));
        orig_points.push_back(Point2f(x4, y4));

        dst_points.push_back(Point2f(0, height));
        dst_points.push_back(Point2f(width, height));
        dst_points.push_back(Point2f(width, 0));
        dst_points.push_back(Point2f(0, 0));

        //Create the IPM transformer
        ipm = IPM(Size(width, height), Size(width, height), orig_points, dst_points);
    }

    //check if enough duration has passed, ignore the error CLion is crying about
    if ((ros::Time::now() - last_published) > publish_interval) {
        last_published = ros::Time::now();
        try {
            inputImage = cv_bridge::toCvShare(msg, "bgr8")->image;
            //If input image is empty then quit
            if (!inputImage.empty()) {
                inputImage.copyTo(workingImage);

                //Applies the IPM to the image
                ipm.applyHomography(workingImage, ipmOutput);

                //Draw points on the original image
                ipm.drawPoints(orig_points, workingImage);

                //Filters the image
                filter.filterImage(ipmOutput, filterOutput);

                //Shows and updates images if debugging
                if (showWindow) createWindow();
                else destroyAllWindows();

                //Outputs the image
                sensor_msgs::ImagePtr output_message = cv_bridge::CvImage(std_msgs::Header(), "mono8",
                                                                          filterOutput).toImageMsg();
                pub.publish(output_message);

                //Color filter calibration
                if (isCalibratingManually) {
                    filter.manualCalibration();
                } else {
                    filter.stopManualCalibration();
                }

                int a = waitKey(20);
                //Press 'm' to calibrate manually
                if (a == 109) {
                    if (!isCalibratingManually) {
                        ROS_INFO("Beginning manual calibration");
                    } else {
                        ROS_INFO("Ending manual calibration");
                        ROS_INFO("Saving filter state in %s", mfilter_file.c_str());
                        fstream filter_file(mfilter_file, ios::trunc | ios::out);
                        filter_file << filter.getValues();
                        cout << filter.getValues();
                        filter_file.close();
                    }
                    isCalibratingManually = !isCalibratingManually;
                }
                    //Press 's' to show/unshow window
                else if (a == 115) {
                    showWindow = !showWindow;
                }
            }
        }
        catch (cv_bridge::Exception &e) {
            ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
        }
    }
}

void RosVision::createWindow() {
    // Create one big mat for all our images
    cv::Size main_window_size(displayWindowWidth, displayWindowHeight);
    cv::Size sub_window_size(main_window_size.width/2, main_window_size.height/2);
    cv::Mat main_image(main_window_size, CV_8UC3);

    // Copy all our images to the big Mat we just created
    cv::Mat image1Roi(main_image, cv::Rect(0, 0, sub_window_size.width, sub_window_size.height));
    resize(workingImage, image1Roi, sub_window_size);
    cv::Mat image2Roi(main_image, cv::Rect(sub_window_size.width, 0, sub_window_size.width, sub_window_size.height));
    resize(ipmOutput, image2Roi, sub_window_size);
    // Our third image is greyscale, so make it "colored" so it can be merged into the colored main mat
    cv::Mat filterOutputBGR;
    cv::cvtColor(filterOutput, filterOutputBGR, CV_GRAY2BGR);
    cv::Mat image3Roi(main_image, cv::Rect(0, sub_window_size.height, sub_window_size.width, sub_window_size.height));
    resize(filterOutputBGR, image3Roi, sub_window_size);

    // Display our big Mat
    std::string windowName("Snowbots - IPM");
    namedWindow(windowName, CV_WINDOW_NORMAL);
    imshow(windowName, main_image);
}

RosVision::RosVision() {
}


RosVision::RosVision(int argc, char **argv, std::string node_name) {

    displayWindowName = "Snowbots - IPM";
    receivedFirstImage = false;

    //Calibration Variables
    showWindow = true;
    isCalibratingManually = false;

    //ROS
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    // Set topics
    image_topic = "/robot/camera1/image_raw";
    output_topic = "/vision/filtered_image";
    ROS_INFO("Image (Subscribe) Topic: %s", image_topic.c_str());
    ROS_INFO("Output (Publish) Topic: %s", output_topic.c_str());

    // Get some params (not all though, we wait until we have an image to get IPM ones)
    SB_getParam(nh_private, "display_window_width", displayWindowWidth, 1000);
    SB_getParam(nh_private, "display_window_height", displayWindowHeight, 1000);

    //Initializes publishers and subscribers
    image_transport::ImageTransport it(nh);
    sub = it.subscribe<RosVision>(image_topic, 1, &RosVision::imageCallback, this);
    pub = it.advertise(output_topic, 1);

    //Sets up filter update frequency
    double frequency;
    SB_getParam(nh_private, "frequency", frequency, 5.0);
    publish_interval = ros::Duration(1 / frequency);
    last_published = ros::Time::now();

    //Check for filter initialization file
    mfilter_file = ros::package::getPath("vision") + "/launch/filter_init.txt";
    ROS_INFO("Looking for filter file at: %s", mfilter_file.c_str());
    fstream filter_file(mfilter_file, ios::in);
    string line;
    bool filter_set = false;
    if (filter_file.is_open()){
        if (getline(filter_file, line)){
            istringstream iss(line);
            int lh, hh, ls, hs, lv, hv;
            if (iss >> lh >> hh >> ls >> hs >> lv >> hv) {
                ROS_INFO("Filter file found");
                ROS_INFO("Filter initializing with: %d %d %d %d %d %d", lh, hh, ls, hs, lv ,hv);
                filter = snowbotsFilter(lh, hh, ls, hs, lv, hv);
                filter_set = true;
            }
        }
    }
    if (!filter_set){
        ROS_INFO("Filter file not found");
        ROS_INFO("Filter initialized with default values");
        filter = snowbotsFilter(0, 155, 0, 155, 150, 255);
    }
    filter_file.close();

    ROS_WARN("Waiting for first image");
}
