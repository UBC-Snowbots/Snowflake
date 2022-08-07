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

    // Obtains draw_qr_codes parameter from the parameter server (or launch file)
    std::string parameter_name    = "draw_qr_codes";
    //std::string parameter_name2    = "camera";
    SB_getParam(private_nh, parameter_name, draw_qr_codes, false);
    //SB_getParam(private_nh, parameter_name2, camera, 1);

    // Setup Subscriber(s)
   // if(camera == 1){
   // std::string topic_to_subscribe_to = "cam_1/color/raw";
    //}else if(camera == 2){
    std::string topic_to_subscribe_to = "cam_2/color/raw";
    //}
    int queue_size                    = 5;
    my_subscriber                     = it.subscribe(
    topic_to_subscribe_to, queue_size, &DetectQRCode::subscriberCallBack, this);

    // Setup Publisher(s)
    std::string topic = private_nh.resolveName("publish_topic");
    queue_size        = 1;
    my_publisher = private_nh.advertise<std_msgs::String>(topic, queue_size);
}

void DetectQRCode::subscriberCallBack(const sensor_msgs::Image::ConstPtr& msg) {
    ROS_INFO("Received message");
    std::vector<std::string> qr_codes = fetchQRCodes(rosToMat(msg));
}

cv::Mat DetectQRCode::rosToMat(const sensor_msgs::Image::ConstPtr& image) {
    cv_bridge::CvImagePtr image_ptr;
    image_ptr = cv_bridge::toCvCopy(image, image->encoding);
    return image_ptr->image;
}

std::vector<std::string> DetectQRCode::fetchQRCodes(const cv::Mat& image) {
    std::vector<std::string> decoded_info;
    std::vector<cv::Point> corners;
    // qrcode.detectAndDecodeMulti(image, decoded_info, corners);
    if (draw_qr_codes) {
        cv::Mat outputImage;
        image.copyTo(outputImage);
        double fps = 1; // fix this with ROS based time calc
        drawQRCodes(outputImage, decoded_info, corners, fps);
    }
    return decoded_info;
}

void DetectQRCode::drawQRCodes(cv::Mat& image, std::vector<std::string> decoded_info, std::vector<cv::Point> corners, double fps) {
    // TODO
    
}
