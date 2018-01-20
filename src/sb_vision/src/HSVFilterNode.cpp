/*
 * Created By: Robyn Castro
 * Created On: June 17, 2017
 * Description: Filters an image in the HSV colourspace to a binary image
 *
 */

#include <HSVFilterNode.h>

using namespace cv;
using namespace cv_bridge;

HSVFilterNode::HSVFilterNode(int argc, char** argv, std::string node_name) {
    displayWindowName  = "Snowbots - HSVFilterNode";
    receivedFirstImage = false;

    // ROS
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    // Set topics
    std::string image_topic  = "/robot/vision/raw_image";
    std::string output_topic = "/vision/hsv_filtered_image";

    // Setup image transport
    image_transport::ImageTransport it(nh);

    // Setup subscriber
    uint32_t queue_size = 1;
    image_sub           = it.subscribe(
    image_topic, queue_size, &HSVFilterNode::rawImageCallBack, this);
    // Setup publisher
    filter_pub = it.advertise(output_topic, queue_size);

    // Get some params (not all though, we wait until we have an image to get
    // the rest)
    SB_getParam(private_nh, "update_frequency", frequency, 5.0);
    SB_getParam(private_nh,
                "config_file",
                mfilter_file,
                ros::package::getPath("vision") + "/launch/filter_init.txt");
    SB_getParam(private_nh, "show_image_window", showWindow, true);
    SB_getParam(
    private_nh, "show_calibration_window", isCalibratingManually, false);

    setUpFilter();
}

void HSVFilterNode::rawImageCallBack(
const sensor_msgs::Image::ConstPtr& image) {
    if (!receivedFirstImage) {
        ROS_INFO("First image received!");
        ros::NodeHandle private_nh("~");
        SB_getParam(private_nh, "image_width", image_width, (int) image->width);
        SB_getParam(
        private_nh, "image_height", image_height, (int) image->height);
        receivedFirstImage = true;
    }

    imageInput = rosToMat(image);

    // Filter out non-green colors
    Mat filteredImage;
    filter.filterImage(imageInput, filteredImage);
    filterOutput = filteredImage;

    // If enough time has passed update filter and show image
    if ((ros::Time::now() - last_published) > publish_interval) {
        last_published = ros::Time::now();
        if (showWindow) { showRawAndFilteredImageWindow(); }
        updateFilter();
    }

    // Outputs the image
    sensor_msgs::ImagePtr output_message =
    cv_bridge::CvImage(std_msgs::Header(), "mono8", filteredImage).toImageMsg();
    // Publish recommended Twist message
    filter_pub.publish(output_message);
}

Mat HSVFilterNode::rosToMat(const sensor_msgs::Image::ConstPtr& image) {
    CvImagePtr imagePtr;
    imagePtr = toCvCopy(image, image->encoding);
    return imagePtr->image;
}

void HSVFilterNode::setUpFilter() {
    // Sets up filter update frequency
    publish_interval = ros::Duration(1 / frequency);
    last_published   = ros::Time::now();

    // Check for filter initialization file
    ROS_INFO("Looking for filter file at: %s", mfilter_file.c_str());
    std::fstream filter_file(mfilter_file, std::ios::in);
    std::string line;
    bool filter_set = false;
    if (filter_file.is_open()) {
        if (getline(filter_file, line)) {
            std::istringstream iss(line);
            int lh, hh, ls, hs, lv, hv;
            if (iss >> lh >> hh >> ls >> hs >> lv >> hv) {
                ROS_INFO("Filter file found");
                ROS_INFO("Filter initializing with: %d %d %d %d %d %d",
                         lh,
                         hh,
                         ls,
                         hs,
                         lv,
                         hv);
                filter     = HSVFilter(lh, hh, ls, hs, lv, hv);
                filter_set = true;
            }
        }
    }
    if (!filter_set) {
        ROS_INFO("Filter file not found");
        ROS_INFO("Filter initialized with default values");
        filter = HSVFilter(0, 155, 0, 155, 150, 255);
    }
    filter_file.close();

    ROS_INFO("Waiting for first image");
}

void HSVFilterNode::updateFilter() {
    // Color filter calibration
    if (isCalibratingManually) filter.manualCalibration();

    int a = waitKey(20);
    // Press 'm' to calibrate manually, press m again to save
    if (a == 109) {
        if (!isCalibratingManually) {
            ROS_INFO("Beginning manual calibration");
        } else {
            ROS_INFO("Ending manual calibration");
            ROS_INFO("Saving filter state in %s", mfilter_file.c_str());
            std::fstream filter_file(mfilter_file,
                                     std::ios::trunc | std::ios::out);
            filter_file << filter.getValues();
            std::cout << filter.getValues();
            filter_file.close();
            filter.stopManualCalibration();
        }
        isCalibratingManually = !isCalibratingManually;
    } // Press 's' to show/unshow window
    else if (a == 115) {
        showWindow = !showWindow;
        if (!showWindow) destroyWindow(displayWindowName);
    }
}

void HSVFilterNode::showRawAndFilteredImageWindow() {
    // Create one big mat for all our images
    cv::Size main_window_size(image_width * 2, image_height);
    cv::Size sub_window_size(main_window_size.width / 2,
                             main_window_size.height);
    cv::Mat main_image(main_window_size, CV_8UC3);

    // Copy all our images to the big Mat we just created (in 4 sub-windows)

    // Image 1
    cv::Mat image1Roi(
    main_image, cv::Rect(0, 0, sub_window_size.width, sub_window_size.height));
    resize(imageInput, image1Roi, sub_window_size);

    // Image 2
    Mat filterOutputBGR;
    cvtColor(filterOutput, filterOutputBGR, CV_GRAY2BGR);
    cv::Mat image2Roi(
    main_image,
    cv::Rect(
    sub_window_size.width, 0, sub_window_size.width, sub_window_size.height));
    resize(filterOutputBGR, image2Roi, sub_window_size);

    cv::imshow(displayWindowName, main_image);
}
