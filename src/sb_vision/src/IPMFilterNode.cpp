/*
 * Created By: Robyn Castro
 * Created On: June 17, 2017
 * Description: Filters out everything but green.
 *
 */

#include <IPMFilterNode.h>

using namespace cv;
using namespace cv_bridge;

IPMFilterNode::IPMFilterNode(int argc, char** argv, std::string node_name) {
    receivedFirstImage = false;

    // Set topics
    std::string image_topic  = "/vision/hsv_filtered_image";
    std::string output_topic = "/vision/ipm_filtered_image";

    // ROS
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    // Setup image transport
    image_transport::ImageTransport it(nh);

    // Setup subscriber
    uint32_t queue_size = 1;
    image_sub           = it.subscribe(
    image_topic, queue_size, &IPMFilterNode::filteredImageCallBack, this);

    // Setup publisher
    ipm_filter_pub = it.advertise(output_topic, queue_size);

    SB_getParam(private_nh, "ipm_base_width", ipm_base_width, (float) 1);
    SB_getParam(private_nh, "ipm_top_width", ipm_top_width, (float) 0.5);
    SB_getParam(
    private_nh, "ipm_base_displacement", ipm_base_displacement, (float) 0);
    SB_getParam(
    private_nh, "ipm_top_displacement", ipm_top_displacement, (float) 0.25);
}

IPMFilterNode::IPMFilterNode(){};

void IPMFilterNode::filteredImageCallBack(
const sensor_msgs::ImageConstPtr& msg) {
    if (!receivedFirstImage) {
        ros::NodeHandle private_nh("~");
        ROS_INFO("First image received! (IPM)");
        receivedFirstImage = true;
        // Obtains parameters of image and IPM points from the param server
        SB_getParam(private_nh, "image_width", image_width, (int) msg->width);
        SB_getParam(
        private_nh, "image_height", image_height, (int) msg->height);
        ipmFilter = new IPMFilter(ipm_base_width,
                                  ipm_top_width,
                                  ipm_base_displacement,
                                  ipm_top_displacement,
                                  image_height,
                                  image_width);
    }

    Mat imageInput = rosToMat(msg);
    Mat IPMFilteredImage;

    // Filter the image
    ipmFilter->filterImage(imageInput, IPMFilteredImage);

    // Outputs the image
    sensor_msgs::ImagePtr output_message =
    cv_bridge::CvImage(std_msgs::Header(), "mono8", IPMFilteredImage)
    .toImageMsg();
    ipm_filter_pub.publish(output_message);
}

Mat IPMFilterNode::rosToMat(const sensor_msgs::Image::ConstPtr& image) {
    CvImagePtr imagePtr;
    imagePtr = toCvCopy(image, image->encoding);
    return imagePtr->image;
}
