/*
 * Created By: Ihsan Olawale, Rowan Zawadski
 * Created On: July 17th, 2022
 * Description: An example node that subscribes to a topic publishing strings,
 *              and re-publishes everything it receives to another topic with
 *              a "!" at the end
 */

#include <DetectQRCode.h>

DetectQRCode::DetectQRCode(int argc, char **argv, std::string node_name) {
    // Setup NodeHandles
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    // Setup image transport
    image_transport::ImageTransport it(nh);

    // Setup Subscriber(s)
    std::string topic_to_subscribe_to = "subscribe_topic";
    int queue_size                    = 10;
    my_subscriber                     = it.subscribe(
    topic_to_subscribe_to, queue_size, &DetectQRCode::subscriberCallBack, this);

    // Setup Publisher(s)
    std::string topic = private_nh.resolveName("publish_topic");
    queue_size        = 1;
    my_publisher = private_nh.advertise<std_msgs::String>(topic, queue_size);
}

void DetectQRCode::subscriberCallBack(const sensor_msgs::Image::ConstPtr& msg) {
    ROS_INFO("Received message");
    rosToMat(msg);
}

cv::Mat DetectQRCode::rosToMat(const sensor_msgs::Image::ConstPtr& image) {
    cv_bridge::CvImagePtr image_ptr;
    image_ptr = cv_bridge::toCvCopy(image, image->encoding);
    return image_ptr->image;
}
