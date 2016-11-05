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
/**
 *
 */
void VisionDecision::imageCallBack(const sensor_msgs::Image::ConstPtr& raw_image) {
    // Deal with new messages here
    double imageRatio = getHorizontalImageRatio(raw_image);
    geometry_msgs::Twist twistMsg;

    // Decide how much to turn
    double maxBias = (raw_image->width * raw_image->height) / 2;
    double relativeAngle = getDesiredAngle(imageRatio, -maxBias, maxBias);

    // Initialize linear velocities to 0
    twistMsg.linear.x = getDesiredSpeed(relativeAngle);
    twistMsg.linear.y = 0;
    twistMsg.linear.z = 0;

    // Initialize x and y angular velocities to 0
    twistMsg.angular.x = 0;
    twistMsg.angular.y = 0;

    // Decide how fast to turn
    twistMsg.angular.z = getDesiredAngularSpeed(relativeAngle);

    // Do the turn
    double t0 = ros::Time::now().toSec();
    double current_angle;
    ros::Rate loop_rate(1000);
    do{
        publishTwist(twistMsg);
        double t1 = ros::Time::now().toSec();
        current_angle = twistMsg.angular.z * (t1-t0);
        ros::spinOnce();
        loop_rate.sleep();
    }while(current_angle<abs(relativeAngle));

    // Bring back all velocities to 0, showing it's done moving.
    twistMsg.angular.z = 0;
    twistMsg.linear.x = 0;
    publishTwist(twistMsg);
}

void VisionDecision::publishTwist(geometry_msgs::Twist twist){
    twist_publisher.publish(twist);
}

/**
 * Takes in an image reference, then returns the difference of the black
 * in the left and right sides.
 * 
 * @param raw_scan
 *           image reference
 * @returns double
 *           imageBlackRatio of left to right.
 */
double VisionDecision::getHorizontalImageRatio(const sensor_msgs::Image::ConstPtr& raw_scan){

    uint32 imageHeight = raw_scan->height;
    uint32 imageWidth = raw_scan->width;

    double rightWhiteCount = 0;
    double leftWhiteCount = 0;
    int currentData;

    printf("imageHeight: %d, imageWidth: %d \n", imageHeight, imageWidth);

    for(int rowCount = 0; rowCount < imageHeight; rowCount++){
        for(int columnCount = 0; columnCount < imageWidth; columnCount++){
            currentData = raw_scan->data[rowCount*8 + columnCount] == 0;
            //printf("currentData[%d*%d]: %d \n", rowCount, columnCount, currentData);

            if((columnCount < imageWidth/2.0) && (currentData == 0)) {
                leftWhiteCount++;
            }
            else if(currentData == 0){
                rightWhiteCount++;
            }
        }
    }

    //printf("LeftWhiteCount: %d, RightWhiteCount: %d \n", leftWhiteCount, rightWhiteCount);

    return rightWhiteCount - leftWhiteCount;
}

/**
 * Takes in an image reference, then returns the number of black pixels
 * at the top of the image
 *
 * @param raw_scan
 *          image reference
 * @return double
 *          number of black pixels at the top of the image
 */
double VisionDecision::getTopPixels(const sensor_msgs::Image::ConstPtr& image_scan){

    uint32 imageHeight = image_scan->height;
    uint32 imageWidth = image_scan->width;

    double topWhiteCount = 0;
    int currentData;

    // printf("imageHeight: %d, imageWidth: %d \n", imageHeig
    // ht, imageWidth);

    for(int rowCount = imageHeight/2; rowCount < imageHeight; rowCount++){
        for(int columnCount = 0; columnCount < imageWidth; columnCount++){
            currentData = image_scan->data[rowCount*8 + columnCount] == 0;
            //printf("currentData[%d*%d]: %d \n", rowCount, columnCount, currentData);
            if(currentData == 0)
                topWhiteCount++;
        }
    }

    //printf("LeftWhiteCount: %d, RightWhiteCount: %d \n", leftWhiteCount, rightWhiteCount);

    return topWhiteCount;

}

/**
 * Takes in the image ratio and decides how heavy the angle
 * should be.
 *
 * @param imageRatio
 *      rightBlackPixels - leftBlackPixels
 * @return double
 *      an angle between -180 and 180
 */
double VisionDecision::getDesiredAngle(double imageRatio, double inLowerBound, double inUpperBound){
    double outLowerBound = -180;
    double outUpperBound = 180;

    return mapRange(imageRatio, inLowerBound, inUpperBound, outLowerBound, outUpperBound);
}

/**
 *  Returns a rotation speed based on the imageRatio
 *
 *  @param imageRatio
 *      rightBlackPixels - leftBlackPixels
 *  @returns double
 *      rotation speed of robot
 */
double VisionDecision::getDesiredAngularSpeed(double desiredAngle){
    double speedToMap = abs(desiredAngle);

}

/**
 *  Returns the desired forward speed
 *
 *  @param desiredAngle
 *      angle robot will turn
 *  @returns double
 *      moving speed of robot
 */
double VisionDecision::getDesiredSpeed(double desiredAngle){
    double speedToMap = abs(desiredAngle);
}

/**
 * Re-maps a number from one range to another
 *
 * @param x
 *      the number to map
 * @param inMin
 *      lower bound of value's current range
 * @param inMax
 *      upper bound of value's current range
 * @param outMin
 *      lower bound of value's target range
 * @param outMax
 *      upper bound of value's target range
 * @return double
 *      the mapped number
 */
double VisionDecision::mapRange(double x, double inMin, double inMax, double outMin, double outMax){
    return (x - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
}

