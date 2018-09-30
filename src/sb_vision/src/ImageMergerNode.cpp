/*
 * Created By: Robyn Castro
 * Created On: June 17, 2017
 * Description: Filters an image in the HSV colourspace to a binary image
 *
 */

#include <ImageMergerNode.h>

using namespace cv;
using namespace cv_bridge;

ImageMergerNode::ImageMergerNode(int argc, char** argv, std::string node_name) {
    display_window_name  = "Snowbots - ImageMergerNode";
    namedWindow(display_window_name, CV_WINDOW_AUTOSIZE);

    received_first_image = false;
    received_second_image = false;

    // ROS
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    // Set topics
    std::string white_line_image_topic  = "robot/vision/first_image";
    std::string yellow_line_image_topic  = "robot/vision/second_image";

    std::string output_topic = "vision/merged_image";

    // Setup image transport
    image_transport::ImageTransport it(nh);

    SB_getParam(private_nh, "show_image_window", show_window, true);

    // Setup subscribers
    uint32_t queue_size = 1;
    first_image_sub           = it.subscribe(
            white_line_image_topic, queue_size, &ImageMergerNode::firstImageCallBack, this);

    second_image_sub          = it.subscribe(
            yellow_line_image_topic, queue_size, &ImageMergerNode::secondImageCallBack, this);

    // Setup publisher
    merged_pub = it.advertise(output_topic, queue_size);
}

void ImageMergerNode::firstImageCallBack(
        const sensor_msgs::Image::ConstPtr &image) {
    if (!received_first_image) {
        ROS_INFO("First image received! (Merger)");
        received_first_image = true;
        moveWindow(display_window_name, 0, image->height * 2);
    }

    // Update the first image
    first_image = rosToMat(image);

    if (received_first_image && received_second_image) {
        // Combines two images
        bitwise_or(first_image, second_image, merged_image);

        Mat flipped_image;
        flip(merged_image, flipped_image, 0); // 0 to flip around the x-axis

        if (show_window) {
            imshow(display_window_name, merged_image);
            waitKey(100);
        }

        // Outputs the image
        sensor_msgs::ImagePtr output_message =
                cv_bridge::CvImage(std_msgs::Header(), "mono8", first_image).toImageMsg();

        merged_pub.publish(output_message);
    }
}

void ImageMergerNode::secondImageCallBack(
        const sensor_msgs::Image::ConstPtr &image) {
    if (!received_second_image) {
        ROS_INFO("Second image received! (Merger)");
        received_second_image = true;
    }

    // Update the second image
    second_image = rosToMat(image);

    if (received_first_image && received_second_image) {
        // Combines two images
        bitwise_or(first_image, second_image, merged_image);

        Mat flipped_image;
        flip(merged_image, flipped_image, 0); // 0 to flip around the x-axis

        if (show_window) {
            imshow(display_window_name, merged_image);
            waitKey(100);
        }
        // Outputs the image
        sensor_msgs::ImagePtr output_message =
                cv_bridge::CvImage(std_msgs::Header(), "mono8", second_image).toImageMsg();
        merged_pub.publish(output_message);
    }

}

Mat ImageMergerNode::rosToMat(const sensor_msgs::Image::ConstPtr& image) {
    CvImagePtr imagePtr;
    imagePtr = toCvCopy(image, image->encoding);
    return imagePtr->image;
}

