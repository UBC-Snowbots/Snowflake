/*
 * Created By: Robyn Castro
 * Created On: June 15, 2017
 * Description: Determines whether or not a circle is seen on
 *              the screen then publishes a recommended std_msgs
 *              boolean
 */

#ifndef CIRCLE_DETECTION_H
#define CIRCLE_DETECTION_H

// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/opencv.hpp>

// Image Conversion
#include <cv_bridge/cv_bridge.h>
#include <image_transport/subscriber.h>
#include <sensor_msgs/image_encodings.h>

// ROS
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>

// STD
#include <string>
#include <vector>

// Snowbots
#include <sb_utils.h>

using namespace cv;

class CircleDetection {
  public:
    // Constructors used for testing
    CircleDetection(std::string& image_path);

    CircleDetection();

    /**
     * Constructor
     */
    CircleDetection(int argc, char** argv, std::string node_name);

    /**
     * Counts the number of circles found in the image.
     *
     * @param image to be parsed
     */
    int countCircles(const Mat& filtered_image, bool display_circles = true);

  private:
    /**
     * Callback for the filtered image
     *
     * @param address of filtered image matrix
     */
    void filteredImageCallBack(const sensor_msgs::Image::ConstPtr& image);

    /**
     * Converts ros::sensor_msgs::Image into a cv::Mat
     *
     * @param message to be converted
     */
    Mat rosToMat(const sensor_msgs::Image::ConstPtr& image);

    /**
     *  Displays a window with the detected objects being circled
     */
    void showFilteredObjectsWindow(const Mat& filtered_image,
                                   std::vector<cv::Point2i> center,
                                   std::vector<float> radii);

    /**
     * Determines whether path contains an image.
     *
     */
    void checkIfImageExists(const cv::Mat& img, const std::string& path);

    /**
     * Subscribes to the filtered camera image topic
     */
    image_transport::Subscriber image_sub;

    /**
     * Publishes true when activity is detected
     */
    ros::Publisher activity_publisher;

    // Minimum radius needed to be considered an object
    int min_target_radius;

    // Show window for debugging purposes
    bool show_window;
};

#endif
