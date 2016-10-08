/*
 * Created By: Gareth Ellis
 * Created On: September 22, 2016
 * Description: The vision decision node, takes in an image from the robot's
 *              camera and produces a recommended twist message
 */

#include <VisionDecision.h>
#include <tiff.h>

// The constructor
VisionDecision::VisionDecision(int argc, char **argv, std::string node_name) {
    ros::init(argc, argv, node_name);
    // Setup NodeHandles
    ros::NodeHandle nh;
    ros::NodeHandle public_nh("~");

    // Setup Subscriber(s)
    std::string camera_image_topic_name = "/vision_processing/filtered_image";
    int refresh_rate = 10;
    image_subscriber = public_nh.subscribe(camera_image_topic_name, refresh_rate, &VisionDecision::imageCallBack, this);

    // Setup Publisher(s)
    std::string twist_topic = public_nh.resolveName("command");
    uint32_t queue_size = 10;
    twist_publisher = nh.advertise<geometry_msgs::Twist>(twist_topic, queue_size);

}

// This is called whenever a new message is received
void VisionDecision::imageCallBack(const sensor_msgs::Image::ConstPtr& raw_scan) {
    // Deal with new messages here
  
}

void VisionDecision::publishTwist(geometry_msgs::Twist twist){
    twist_publisher.publish(twist);
}

/**
 * Takes in an image reference, then returns the ratio of the white
 * in the left and right sides.
 * 
 * @param raw_scan
 *           image reference
 * @returns double
 *           imageWhiteRatio of left to right.
 */
static double VisionDecision::getImageRatio(const sensor_msgs::Image::ConstPtr& raw_scan){

    auto imageData = raw_scan->data;

    uint32 imageHeight = raw_scan->height;
    uint32 imageWidth = raw_scan->width;

    double rightWhiteCount = 0;
    double leftWhiteCount = 0;

    for(int rowCount = 0; rowCount < imageHeight; rowCount++){
        for(int columnCount = 0; columnCount < imageWidth; columnCount++){
            if(columnCount < imageWidth/2 && imageData[columnCount*rowCount] == 0)
                leftWhiteCount++;
            if(columnCount >  imageWidth/2 && imageData[columnCount*rowCount] == 0)
                rightWhiteCount++;
        }
    }

    // returns a special case if rightWhiteCount == 0
    if(rightWhiteCount != 0)
        return leftWhiteCount/rightWhiteCount;
    else if(rightWhiteCount == leftWhiteCount)
        return 1;
    else
        return 999;
}
