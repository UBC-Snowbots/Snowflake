/*
 * Created By: Raad Khan
 * Created On: April 23, 2017
 * Description: Takes in an image feed and uses LineDetect to generate
 *              lane lines and a destination point, then broadcasts a
 *              Twist message to stay within the lanes.
 */

#include "LaneFollow.h"

using namespace ros;
using namespace cv;

LaneFollow::LaneFollow(int argc, char** argv, std::string node_name) {
    // ROS
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    receivedFirstImage = false;

    // setup image transport
    image_transport::ImageTransport it(nh);

    // setup topics
    std::string input_topic  = "vision/complete_filtered_image";
    std::string output_topic = "lane_follow/recommended_steer";

    uint32_t queue_size = 1;

    SB_getParam(private_nh, "ipm_base_width", ipm_base_width, (float) 1);
    SB_getParam(private_nh, "ipm_top_width", ipm_top_width, (float) 0.5);
    SB_getParam(private_nh, "ipm_base_displacement", ipm_base_displacement, (float) 0);
    SB_getParam(private_nh, "ipm_top_displacement", ipm_top_displacement, (float) 0.25);

    // set up subscriber
    filtered_image_sub = it.subscribe(
    input_topic, queue_size, &LaneFollow::laneFollowCallback, this);

    // set up publisher
    stay_in_lane_pub =
    nh.advertise<geometry_msgs::Twist>(output_topic, queue_size);
}

void LaneFollow::laneFollowCallback(
const sensor_msgs::Image::ConstPtr& filteredImage) {
    if (!receivedFirstImage) {
        ros::NodeHandle private_nh("~");
        ROS_INFO("First image received! (Lane Follow)");
        receivedFirstImage = true;

        IPMFilter(ipm_base_width, ipm_top_width, ipm_base_displacement, ipm_top_displacement, filteredImage->height, filteredImage->width);
    }
    // set don't care components to zero
    LaneFollow::steering_output.linear.y  = 0;
    LaneFollow::steering_output.linear.z  = 0;
    LaneFollow::steering_output.angular.x = 0;
    LaneFollow::steering_output.angular.y = 0;

    // convert filtered Image to filtered Mat
    filtered_image = this->rosToMat(filteredImage);

    origin_point = filtered_image.cols / 2;

    // generate left and right lane points
    std::vector<std::vector<cv::Point2d>> filtered_lane_points =
    ld.getLanePoints(filtered_image);

    // make sure we only have two sets of lane points
    assert(filtered_lane_points.size() == 2);

    // convert the filtered lane points to raw lane points
    std::vector<std::vector<Point2d>> lane_points =
    this->transformPoints(filtered_lane_points);

    // generate the left and right lane line polynomials
    std::vector<Polynomial> lane_lines = ld.getLaneLines(lane_points);

    // get the intersection point of the left and right lane lines
    cv::Point2d intersect_point =
    ld.getIntersectionPoint(lane_lines, ld.getDegree());

    // get the intersection angle
    intersect_angle = this->getAngleFromOriginToIntersectPoint(intersect_point);

    // figure out how fast we should turn
    steering_output.angular.z =
    pow(intersect_angle, 2.0) * angular_speed_multiplier;

    // limit the angular speed if needed
    if (steering_output.angular.z > angular_vel_cap)
        steering_output.angular.z = angular_vel_cap;

    // set robot velocity to max if it is not turning
    if (steering_output.angular.z == 0)
        steering_output.linear.x = linear_vel_cap;

    // figure out robot velocity if it is turning
    else
        steering_output.linear.x =
        linear_speed_multiplier / fabs(steering_output.angular.z);

    // limit the linear speed if needed
    if (steering_output.linear.x > linear_vel_cap)
        steering_output.linear.x = linear_vel_cap;

    // publish the recommended steering output to stay in lane
    stay_in_lane_pub.publish(steering_output);
}

cv::Mat LaneFollow::rosToMat(const sensor_msgs::Image::ConstPtr& image) {
    cv_bridge::CvImagePtr imagePtr;

    imagePtr = cv_bridge::toCvCopy(image, image->encoding);

    return imagePtr->image;
}

std::vector<std::vector<Point2d>> LaneFollow::transformPoints(
std::vector<std::vector<cv::Point2d>> filtered_points) {
    std::vector<std::vector<Point2d>> points;

    for (int i = 0; i < filtered_points.size(); i++) {
        for (int j = 0; j < filtered_points[i].size(); j++) {
            cv::Point2d point = ipm.applyHomographyInv(filtered_points[i][j]);

            // initialize an empty vector at the start of every column
            if (j == 0) {
                std::vector<Point2d> set_point;
                points.push_back(set_point);
            }

            points[i].push_back(point);
        }
    }

    return points;
}

double
LaneFollow::getAngleFromOriginToIntersectPoint(cv::Point2d intersect_point) {
    double y  = intersect_point.y;
    double x  = intersect_point.x;
    double dx = x - origin_point;

    double intersect_angle = atan(dx / y);

    return intersect_angle;
}

void LaneFollow::IPMFilter(float ipm_base_width,
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