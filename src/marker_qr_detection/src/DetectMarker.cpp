/*
 * Created By: Ihsan Olawale, Rowan Zawadski
 * Created On: July 17th, 2022
 * Description: An example node that subscribes to a topic publishing strings,
 *              and re-publishes everything it receives to another topic with
 *              a "!" at the end
 */

#include <DetectMarker.h>

    


DetectMarker::DetectMarker(int argc, char **argv, std::string node_name) {
    // Setup NodeHandles
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    // Setup image transport
    image_transport::ImageTransport it(nh);

    // Initialize ArUco parameters
    dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
    parameters = cv::aruco::DetectorParameters::create();
  

    // Obtains draw_markers parameter from the parameter server (or launch file)
    std::string parameter_name    = "draw_markers";
    SB_getParam(private_nh, parameter_name, draw_markers, false);
  
    // Setup Subscriber(s)
    std::string topic_to_subscribe_to = "/cam1/color/image_raw/compressed";
    int queue_size                    = 10;
    my_subscriber                     = it.subscribe(
    topic_to_subscribe_to, queue_size, &DetectMarker::subscriberCallBack, this);

    // Setup Publisher(s)
    std::string topic = private_nh.resolveName("identified");
    queue_size        = 1;
    my_publisher = private_nh.advertise<std_msgs::String>(topic, queue_size);
}

void DetectMarker::subscriberCallBack(const sensor_msgs::Image::ConstPtr& msg) {
    ROS_INFO("Received message");
    std::vector<int> markerIds = fetchMarkerIds(rosToMat(msg));
    std::cout << "Markers detected:";
    for (int markerId: markerIds) {
        std::cout << " " << markerId;
    }
    std::cout << std::endl;
}

std::vector<int> DetectMarker::fetchMarkerIds(const cv::Mat& image) {
    std::vector<int> markerIds;
    std::vector<std::vector<cv::Point2f>> markerCorners;
    cv::aruco::detectMarkers(image, dictionary, markerCorners, markerIds, parameters);
    if (true) {
        cv::Mat outputImage;
        image.copyTo(outputImage);
        cv::aruco::drawDetectedMarkers(outputImage, markerCorners, markerIds);
    }
    return markerIds;
}

cv::Mat DetectMarker::rosToMat(const sensor_msgs::Image::ConstPtr& image) {
    cv_bridge::CvImagePtr image_ptr;
    image_ptr = cv_bridge::toCvCopy(image, image->encoding);
    return image_ptr->image;
}



