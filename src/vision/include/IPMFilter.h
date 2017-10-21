/*
 * Created by: Robyn Castro
 * Created On: July 1, 2017
 * Description: Takes in an image applies inverse perspective mapping to it
 *
 */

// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

// Image conversion
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

// I/O
#include <iostream>
#include <stdio.h>

// ROS
#include <ros/console.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <ros/time.h>
#include <sb_utils.h>

// Objects
#include <IPM.h>

class IPMFilter {
  public:
    /**
     * Initializes the corners of the filter
     */
    IPMFilter(float ipm_base_width,
              float ipm_top_width,
              float ipm_base_displacement,
              float ipm_top_displacement,
              float image_height,
              float image_width);

    /**
     * Filters an image according to ipm
     *
     * @param input the frame being filtered
     * @param output the output image
     */
    void filterImage(const cv::Mat& input, cv::Mat& output);

  private:
    /**
     * Initializator
     *
     * @params image information and where to apply the IPM
     */
    void createFilter(float ipm_base_width,
                      float ipm_top_width,
                      float ipm_base_displacement,
                      float ipm_top_displacement,
                      float image_height,
                      float image_width);

    // Corners of the portion of the image to be filtered
    int x1, y1;
    int x2, y2;
    int x3, y3;
    int x4, y4;

    // Filters and their variables
    IPM ipm;
    std::vector<cv::Point2f> orig_points;
    std::vector<cv::Point2f> dst_points;
};
