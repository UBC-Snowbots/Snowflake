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

CircleDetection::CircleDetection(std::string& image_path) {
    cv::Mat bgr_image = imread(image_path);

    // Check if the image can be loaded
    checkIfImageExists(bgr_image, image_path);

    min_target_radius = 20;
    countCircles(bgr_image);

    waitKey(0);
}

CircleDetection::CircleDetection() {
    min_target_radius = 20;
}

CircleDetection::CircleDetection(int argc, char** argv, std::string node_name) {
    // Setup handles
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    // Setup image transport
    image_transport::ImageTransport it(nh);

    // Setup subscriber
    std::string image_topic = "/robot/vision/filtered_image";
    int refresh_rate        = 10;
    image_sub               = it.subscribe<CircleDetection>(
    image_topic, refresh_rate, &CircleDetection::filteredImageCallBack, this);

    // Setup publishers
    std::string output_topic = "/robot/vision/activity_detected";
    uint32_t queue_size      = 1;
    activity_publisher = nh.advertise<std_msgs::Bool>(output_topic, queue_size);

    // Get some params
    SB_getParam(private_nh, "minimum_target_radius", min_target_radius, 50);
    SB_getParam(private_nh, "show_image_window", show_window, true);
}

void CircleDetection::filteredImageCallBack(
const sensor_msgs::Image::ConstPtr& image) {
    // If something is seen tell the robot to move
    int num_circles = countCircles(rosToMat(image));
    std_msgs::Bool circle_detected;
    circle_detected.data = num_circles > 0;
    activity_publisher.publish(circle_detected);
}

Mat CircleDetection::rosToMat(const sensor_msgs::Image::ConstPtr& image) {
    CvImagePtr image_ptr;
    image_ptr = toCvCopy(image, image->encoding);
    return image_ptr->image;
}

int CircleDetection::countCircles(const Mat& filtered_image,
                                  bool display_circles) {
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    vector<cv::Point2i> center;
    vector<float> radii;

    // Convert to a grayscale image, if channels do not match.
    cv::Mat bwImage;
    switch (filtered_image.channels()) {
        // 4 channel and 3 channel conversion should cover all our use cases.
        case 4: cvtColor(filtered_image, bwImage, CV_BGRA2GRAY); break;
        case 3: cvtColor(filtered_image, bwImage, CV_BGR2GRAY); break;
        default: bwImage = filtered_image;
    }

    // Find contours of the black and white image
    cv::findContours(
    bwImage.clone(), contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_NONE);

    size_t count = contours.size();

    for (int i = 0; i < count; i++) {
        cv::Point2f c;
        float r;
        cv::minEnclosingCircle(contours[i], c, r);

        // Only count circles with large enough radius
        if (r >= min_target_radius) {
            center.push_back(c);
            radii.push_back(r);
        }
    }

    if (display_circles) {
        // Displays a window with the detected objects being circled
        showFilteredObjectsWindow(bwImage, center, radii);
    }

    // Complains about fitting a long to an int
    // non-issue
    return (int) center.size();
}

void CircleDetection::showFilteredObjectsWindow(const Mat& filtered_image,
                                                std::vector<cv::Point2i> center,
                                                std::vector<float> radii) {
    size_t center_count = center.size();
    cv::Scalar green(0, 255, 0);
    cv::Mat color_image;
    cv::cvtColor(filtered_image, color_image, CV_GRAY2BGR);

    // Draw green circles around object
    for (int i = 0; i < center_count; i++) {
        cv::circle(color_image, center[i], (int) radii[i], green, 3);
    }

    namedWindow("Filtered Objects", WINDOW_AUTOSIZE);
    imshow("Filtered Objects", color_image);
    waitKey(0);
}

void CircleDetection::checkIfImageExists(const cv::Mat& img,
                                         const std::string& path) {
    if (img.empty()) {
        std::cout << "Error! Unable to load image: " << path << std::endl;
        std::exit(-1);
    }
}
