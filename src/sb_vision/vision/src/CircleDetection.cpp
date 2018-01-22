/*
 * Created By: Robyn Castro
 * Created On: June 17, 2017
 * Description: Determines whether or not a circle is seen on
 *              the screen then publishes a recommended twist
 *              message.
 */

#include <CircleDetection.h>
#include <image_transport/image_transport.h>
#include <std_msgs/Bool.h>

using namespace cv;
using namespace std;
using namespace cv_bridge;

CircleDetection::CircleDetection(std::string &image_path) {

    cv::Mat bgr_image = imread(image_path);

    // Check if the image can be loaded
    check_if_image_exist(bgr_image, image_path);

    minTargetRadius = 20;
    countCircles(bgr_image);

    waitKey(0);

}

CircleDetection::CircleDetection() {
    minTargetRadius = 20;
}

CircleDetection::CircleDetection(int argc, char **argv, std::string node_name) {

    // Setup handles
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    // Setup image transport
    image_transport::ImageTransport it(nh);

    // Setup subscriber
    std::string image_topic = "/robot/vision/filtered_image";
    int refresh_rate = 10;
    image_sub = it.subscribe<CircleDetection>(image_topic, refresh_rate,
                                              &CircleDetection::filteredImageCallBack, this);

    // Setup publishers
    std::string output_topic = "/robot/vision/activity_detected";
    uint32_t queue_size = 1;
    activity_publisher = private_nh.advertise<std_msgs::Bool>(output_topic, queue_size);

    // Get some params
    SB_getParam(private_nh, "minimum_target_radius", minTargetRadius, 50);
    SB_getParam(private_nh, "show_image_window", showWindow, true);

}

void CircleDetection::filteredImageCallBack(const sensor_msgs::Image::ConstPtr &image) {

    // If something is seen tell the robot to move
    int numCircles = countCircles(rosToMat(image));
    std_msgs::Bool circle_detected;
    circle_detected.data = numCircles > 0;
    activity_publisher.publish(circle_detected);
}

Mat CircleDetection::rosToMat(const sensor_msgs::Image::ConstPtr &image) {
    CvImagePtr imagePtr;
    imagePtr = toCvCopy(image, image->encoding);
    return imagePtr->image;
}

int CircleDetection::countCircles(const Mat &filtered_image, bool displayCircles) {

    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    vector<cv::Point2i> center;
    vector<float> radii;

    // Convert grayscale image to black and white image
    cv::Mat bwImage;
        bwImage = filtered_image;
    // Find contours of the black and white image
    cv::findContours(bwImage.clone(), contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_NONE);

    size_t count = contours.size();

    for (int i = 0; i < count; i++) {
        cv::Point2f c;
        float r;
        cv::minEnclosingCircle(contours[i], c, r);

        // Only count circles with large enough radius
        if (r >= minTargetRadius) {
            center.push_back(c);
            radii.push_back(r);
        }
    }

    if (displayCircles) {
        // Displays a window with the detected objects being circled
//        showFilteredObjectsWindow(filtered_image, center, radii);
    }

    // Complains about fitting a long to an int
    // non-issue
    return (int) center.size();
}

void CircleDetection::showFilteredObjectsWindow(const Mat &filtered_image, std::vector<cv::Point2i> center,
                                                 std::vector<float> radii) {
    size_t center_count = center.size();
    cv::Scalar green(0, 255, 0);
    cv::Mat colorImage;
    cv::cvtColor(filtered_image, colorImage, CV_GRAY2BGR);

    // Draw green circles around object
    for (int i = 0; i < center_count; i++) {
        cv::circle(colorImage, center[i], (int) radii[i], green, 3);
    }

    namedWindow("Filtered Objects", WINDOW_AUTOSIZE);
    imshow("Filtered Objects", colorImage);
    waitKey(20);

}

void CircleDetection::check_if_image_exist(const cv::Mat &img, const std::string &path) {
    if (img.empty()) {
        std::cout << "Error! Unable to load image: " << path << std::endl;
        std::exit(-1);
    }
}
