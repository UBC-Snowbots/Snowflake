/*
 * Created By: Gareth Ellis
 * Created On: September 22, 2016
 * Description: The vision decision node, takes in an image from the robot's
 *              camera and produces a recommended twist message
 */

#include <VisionDecision.h>
#include <tiff.h>

#define PI 3.14159265
#define null -1
#define INVALID 181

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
    /*// Deal with new messages here
    double imageRatio = getHorizontalImageRatio(raw_image);
    geometry_msgs::Twist twistMsg;

    // Decide how much to turn
    double maxBias = (raw_image->width * raw_image->height) / 2;
    double relativeAngle = getAngleAt(imageRatio, -maxBias, maxBias);

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
    publishTwist(twistMsg);*/
}

void VisionDecision::publishTwist(geometry_msgs::Twist twist){
    twist_publisher.publish(twist);
}

/* Functions to determine robot movement */
int VisionDecision::getDesiredAngle(double numSamples, const sensor_msgs::Image::ConstPtr &image_scan){

    int desiredAngle = getAngleAt(false, numSamples, image_scan);

    if(desiredAngle == INVALID) {
        printf("LEFT INVALID \n");
        desiredAngle = getAngleAt(true, numSamples, image_scan);
    }

    return desiredAngle;
}


int VisionDecision::getAngleAt(bool rightSide, double numSamples, const sensor_msgs::Image::ConstPtr &image_scan){

    double imageHeight = image_scan->height;

    // Each slope will be compared to the bottom point of the white line
    double x1 = getMiddle(imageHeight - 1, rightSide, image_scan);

    double sumAngles = 0;
    double initialAngle;
    double finalAngle;

    int slopeAcceptance = 5;

    for (double division = 0; division < numSamples; division++){
        double yCompared = division * imageHeight / (2*numSamples);
        double xCompared = getMiddle(yCompared, rightSide, image_scan);

        double foundAngle;
        double foundSlope;
        if(xCompared == x1)
            foundAngle = 0;
        else {
            foundSlope = - (xCompared - x1)/ (yCompared - imageHeight - 1);
            foundAngle = atan(foundSlope);
        }
        if(division == 0){
            initialAngle = foundAngle * 180.0 / PI;
        } else if(division == (numSamples - 1))
            finalAngle = foundAngle * 180.0 / PI;
        sumAngles += foundAngle;
    }

    int angleDifference = abs(finalAngle - initialAngle);
    if (angleDifference > slopeAcceptance)
        return INVALID;
    else
        return sumAngles / numSamples * 180.0 / PI;
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


/* Helper functions for functions that determine robot movement. */

int VisionDecision::getNumLines(int row, const sensor_msgs::Image::ConstPtr& image_scan){

}

int VisionDecision::getMiddle(int row, bool rightSide, const sensor_msgs::Image::ConstPtr& image_scan){

    int noiseMax = 20;
    int startingPos;
    int incrementer;
    int startPixel, endPixel;

    // Depending on chosen side, determine where to start
    // and how to iterate.
    if(rightSide){
        startingPos = image_scan->width - 1;
        incrementer = -1;
    }else{
        startingPos = 0;
        incrementer = 1;
    }

    // Find first pixel of the white line in a certain row.
    startPixel = VisionDecision::getStartPixel(startingPos, noiseMax, incrementer, row, image_scan);

    // Find last pixel of the white line in a certain row.
    startingPos = startPixel;
    endPixel = VisionDecision::getEndPixel(startingPos, noiseMax, incrementer, row, image_scan);

    // Return average of the two pixels.
    return (startPixel + endPixel) / 2;
}

int VisionDecision::getStartPixel(int startingPos, int noiseMax, int incrementer, int row,
                     const sensor_msgs::Image::ConstPtr& image_scan){
    int column = startingPos;
    int whiteVerificationCount = 0;
    int blackVerificationCount = 0;
    int startPixel = null;
    int toBeChecked = null;

    // Find starting pixel
    while(column < image_scan->width && column >= 0){
        // If white pixel found start verifying if proper start.
        if(image_scan->data[row*image_scan->width + column] != 0){
            blackVerificationCount = 0;
            // This pixel is what we are checking
            if (toBeChecked == null)
                toBeChecked = column;

            // Determine whether toBeChecked is noise
            whiteVerificationCount++;
            if (whiteVerificationCount == noiseMax && startPixel == null)
                startPixel = toBeChecked;
        } else {
            blackVerificationCount++;
            if(blackVerificationCount == noiseMax) {
                whiteVerificationCount = 0; // Reset verification if black pixel.
                toBeChecked = null;
            }
        }
        // Go to next element
        column += incrementer;
    }
    return startPixel;
}

int VisionDecision::getEndPixel(int startingPos, int noiseMax, int incrementer, int row,
                     const sensor_msgs::Image::ConstPtr& image_scan){
    int column = startingPos;
    int blackVerificationCount = 0;
    int whiteVerificationCount = 0;
    int endPixel = null;
    int toBeChecked = null;

    while(column < image_scan->width && column >= 0){
        // If black pixel found start verifying if pixel before
        // is proper end.
        if(image_scan->data[row*image_scan->width + column] == 0){
            whiteVerificationCount = 0;
            // This pixel is what we are checking
            if (toBeChecked == null)
                toBeChecked = column - incrementer; //toBeChecked = lastPixel

            // Determine whether toBeChecked is noise
            blackVerificationCount++;
            if (blackVerificationCount == noiseMax && endPixel == null)
                endPixel = toBeChecked;
        } else {
            whiteVerificationCount++;
            if(whiteVerificationCount == noiseMax) {
                blackVerificationCount = 0; // Reset verification if white pixel.
                toBeChecked = null;
            }
        }
        // Go to next element
        column += incrementer;
    }
    return endPixel;
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




