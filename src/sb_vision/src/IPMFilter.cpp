/*
 * Created by: Robyn Castro
 * Created On: July 1, 2017
 * Description: Takes in an image applies inverse perspective mapping to it
 *
 */

#include <IPMFilter.h>

using namespace cv;

IPMFilter::IPMFilter(float ipm_base_width,
                     float ipm_top_width,
                     float ipm_base_displacement,
                     float ipm_top_displacement,
                     float image_height,
                     float image_width) {
    createFilter(ipm_base_width,
                 ipm_top_width,
                 ipm_base_displacement,
                 ipm_top_displacement,
                 image_height,
                 image_width);
}

void IPMFilter::createFilter(float ipm_base_width,
                             float ipm_top_width,
                             float ipm_base_displacement,
                             float ipm_top_displacement,
                             float image_height,
                             float image_width) {
    x1 = image_width / 2 - ipm_base_width / 2 * image_width;
    y1 = (1 - ipm_base_displacement) * image_height;
    x2 = image_width / 2 + ipm_base_width / 2 * image_width;
    y2 = (1 - ipm_base_displacement) * image_height;
    x3 = image_width / 2 + ipm_top_width / 2 * image_width;
    y3 = image_height * ipm_top_displacement;
    x4 = image_width / 2 - ipm_top_width / 2 * image_width;
    y4 = image_height * ipm_top_displacement;

    // Set up the IPM points
    orig_points.push_back(Point2f(x1, y1));
    orig_points.push_back(Point2f(x2, y2));
    orig_points.push_back(Point2f(x3, y3));
    orig_points.push_back(Point2f(x4, y4));

    dst_points.push_back(Point2f(0, image_height));
    dst_points.push_back(Point2f(image_width, image_height));
    dst_points.push_back(Point2f(image_width, 0));
    dst_points.push_back(Point2f(0, 0));

    // Create the IPM transformer
    ipm = IPM(Size(image_width, image_height),
              Size(image_width, image_height),
              orig_points,
              dst_points);
}

void IPMFilter::filterImage(const cv::Mat& input, cv::Mat& output) {
    // If input image is empty then quit
    if (!input.empty()) {
        Mat workingImage;
        input.copyTo(workingImage);
        // Applies the IPM to the image
        ipm.applyHomography(workingImage, output);
    }
}
