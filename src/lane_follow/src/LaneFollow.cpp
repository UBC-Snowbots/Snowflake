/*
 * Created By: Raad Khan
 * Created On: April 23, 2017
 * Description: Gets angle of point of intersection of lane lines
 *              and broadcasts a recommended Twist message.
 */

#include <LaneFollow.h>

class Twist;

using namespace cv;

LaneFollow::LaneFollow(int argc, char **argv, std::string node_name) {

    // Setup handles
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    // Setup subscriber
    std::string image_topic_name = "/robot/line_detect/camera_image";
    int refresh_rate = 10;
    ros::Subscriber image_sub = nh.subscribe(image_topic_name, refresh_rate,
                                             &LaneFollow::subscriberCallBack, this);

    // Setup publishers
    std::string filter_topic_name = "/robot/lane_follow/lane_detect_image";
    std::string twist_topic_name = "/robot/lane_follow/twist_message";
    uint32_t queue_size = 1;

    ros::Publisher filter_pub = private_nh.advertise<sensor_msgs::Image>(image_topic_name, queue_size);
    ros::Publisher twist_pub = private_nh.advertise<geometry_msgs::Twist>(twist_topic_name, queue_size);

    ros::Rate loop_rate(10);

    // Get Params
    SB_getParam(nh, "angular_vel_cap", angular_vel_cap, 1.0);
    SB_getParam(nh, "linear_vel_cap", linear_vel_cap, 1.0);
    SB_getParam(nh, "angular_speed_multiplier", angular_speed_multiplier, 1.0);
    SB_getParam(nh, "linear_speed_multiplier", linear_speed_multiplier, 1.0);
    SB_getParam(nh, "target_x_distance", target_x_distance, 0.5);
    SB_getParam(nh, "target_y_distance", target_y_distance, 0.5);

    SB_getParam(nh, "ipm_filter/ipm_base_width", ipm_base_width, (float) 1.0);
    SB_getParam(nh, "ipm_filter/ipm_top_width", ipm_top_width, (float) 0.5);
    SB_getParam(nh, "ipm_filter/ipm_base_displacement", ipm_base_displacement, (float) 0);
    SB_getParam(nh, "ipm_filter/ipm_top_displacement", ipm_top_displacement, (float) 0.25);


    receivedFirstImage = false;

    /*while (ros::ok()) {

        // ...
        ros::spinOnce();
        loop_rate.sleep();
    }*/
}

void LaneFollow::subscriberCallBack(const sensor_msgs::Image::ConstPtr &msg) {

    // The command to return
    geometry_msgs::Twist stayInLane;

    // Set components we don't care about to 0
    stayInLane.linear.y = 0;
    stayInLane.linear.z = 0;
    stayInLane.angular.x = 0;
    stayInLane.angular.y = 0;

    LineDetect ld;

    cv::Mat filteredImage = this->rosToMat(msg);

    // Initialize ipm filter
    if (!receivedFirstImage) {
        receivedFirstImage = true;
        createFilter(ipm_base_width, ipm_top_width, ipm_base_displacement, ipm_top_displacement,
                     filteredImage.rows, filteredImage.cols);
    }

    std::vector<Window> baseWindows = ld.getBaseWindows(filteredImage);
    std::vector <std::vector<cv::Point2d>> filteredLanePoints = ld.getLanePoints(filteredImage, baseWindows);
    std::vector<Polynomial> filteredBoundaryLines = ld.getLaneLines(filteredLanePoints);
    // transform IPM, cartesian coordinates to real-world, ROS coordinates
    std::vector <std::vector<cv::Point2d>> realLanePoints = this->transformPoints(filteredLanePoints);
    std::vector<Polynomial> realBoundaryLines = ld.getLaneLines(realLanePoints);

    double angle_heading = 0;

    // Head to the middle of the line if 2 lines exist
    if (realBoundaryLines.size() >= 2) {
        cv::Point intersectionPoint = ld.getIntersection(realBoundaryLines[0], realBoundaryLines[1]);
        angle_heading = ld.getAngleFromOriginToPoint(intersectionPoint);
    }// Head to a point a certain distance away from the line
    else if (realBoundaryLines.size() == 1) {
        cv::Point targetPoint = ld.moveAwayFromLine(filteredBoundaryLines[0], target_x_distance, target_y_distance);
        angle_heading = ld.getAngleFromOriginToPoint(targetPoint);
    }
    // If no lines are seen go straight (See initialization)

    // Figure out how fast we should turn
    stayInLane.angular.z = pow(angle_heading, 2.0) * angular_speed_multiplier;

    // Limit the angular speed
    if (stayInLane.angular.z > angular_vel_cap)
        stayInLane.angular.z = angular_vel_cap * stayInLane.angular.z / fabs(stayInLane.angular.z);

    if (stayInLane.angular.z == 0)
        // Go as fast as possible.
        stayInLane.linear.x = linear_vel_cap;
    else
        // Figure out how fast we should move forward
        stayInLane.linear.x = linear_speed_multiplier / fabs(stayInLane.angular.z);

    // Limit the linear speed
    if (stayInLane.linear.x > linear_vel_cap)
        stayInLane.linear.x = linear_vel_cap;

    twist_pub.publish(stayInLane);
}

void LaneFollow::createFilter(float ipm_base_width, float ipm_top_width,
                              float ipm_base_displacement, float ipm_top_displacement,
                              float image_height, float image_width) {
    double x1, x2, x3, x4;
    double y1, y2, y3, y4;

    std::vector<cv::Point2f> orig_points;
    std::vector<cv::Point2f> dst_points;

    x1 = image_width / 2 - ipm_base_width / 2 * image_width;
    y1 = (1 - ipm_base_displacement) * image_height;
    x2 = image_width / 2 + ipm_base_width / 2 * image_width;
    y2 = (1 - ipm_base_displacement) * image_height;
    x3 = image_width / 2 + ipm_top_width / 2 * image_width;
    y3 = image_height * ipm_top_displacement;
    x4 = image_width / 2 - ipm_top_width / 2 * image_width;
    y4 = image_height * ipm_top_displacement;

    //Set up the IPM points
    orig_points.push_back(Point2f(x1, y1));
    orig_points.push_back(Point2f(x2, y2));
    orig_points.push_back(Point2f(x3, y3));
    orig_points.push_back(Point2f(x4, y4));

    dst_points.push_back(Point2f(0, image_height));
    dst_points.push_back(Point2f(image_width, image_height));
    dst_points.push_back(Point2f(image_width, 0));
    dst_points.push_back(Point2f(0, 0));

    //Create the IPM transformer
    ipm = IPM(Size(image_width, image_height), Size(image_width, image_height), orig_points, dst_points);
}

cv::Mat LaneFollow::rosToMat(const sensor_msgs::Image::ConstPtr &image) {
    cv_bridge::CvImagePtr imagePtr;
    imagePtr = cv_bridge::toCvCopy(image, image->encoding);
    return imagePtr->image;
}

std::vector <std::vector<Point2d>> LaneFollow::transformPoints(std::vector <std::vector<cv::Point2d>> filteredPoints) {

    std::vector <std::vector<Point2d>> realPoints;

    for (int i = 0; i < filteredPoints.size(); i++) {
        for (int j = 0; j < filteredPoints[i].size(); j++) {
            cv::Point2d realCartesianPoint = ipm.applyHomographyInv(filteredPoints[i][j]);
            cv::Point2d realROSPoint{realCartesianPoint.y, -realCartesianPoint.x};
            realPoints[i].push_back(realROSPoint);
        }
    }

    return realPoints;
}
