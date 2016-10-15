/*
 * Created By: Robyn Castro
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
void VisionDecision::imageCallBack(const sensor_msgs::Image::ConstPtr& image_scan) {
    // Deal with new messages here
    geometry_msgs::Twist twistMsg;

    // Decide how much to turn
    int relativeAngle = getDesiredAngle(image_scan->height/2.0, image_scan);

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

/* Functions to determine robot movement */

/**
 * Returns the angle of a valid line
 *
 * @param numSamples the number of slopes to sample the angle
 * @param image_scan the image to parse
 *
 * @return the angle of the line to the positive y-axis.
 */
int VisionDecision::getDesiredAngle(double numSamples, const sensor_msgs::Image::ConstPtr &image_scan){

    int desiredAngle = getAngleAt(false, numSamples, image_scan);

    if(desiredAngle == INVALID)
        desiredAngle = getAngleAt(true, numSamples, image_scan);

    // If both cases are invalid it will do a turn 91 degrees.

    return desiredAngle;
}

/**
 * Determines the angle of the line parsed from the left or right side.
 * Returns INVALID if an invalid output is found.
 *
 * @param rightSide determines whether to parse from the left or from the right side.
 * @param numSamples how many slopes to sample the angle.
 * @param image_scan the image to parse.
 *
 * @returns the angle of the line, or INVALID if line is invalid.
 */
int VisionDecision::getAngleAt(bool rightSide, double numSamples, const sensor_msgs::Image::ConstPtr &image_scan){

    // initialization of local variables.
    double imageHeight = image_scan->height;
    int toParseFrom = null;
    int incrementer;
    int startingPos;
    int topRow = null;
    int validSamples = 0;

    // Decides how to parse depending on if rightSide is true or false.
    if(rightSide){
        // starts at right side then increments to the left
        startingPos = image_scan->width - 1;
        incrementer = -1;
    }else{
        // starts at left side then increments to th right
        startingPos = 0;
        incrementer = 1;
    }

    // starts parsing from the right and finds where the highest white line
    // begins.
    for(int i = 0; i < imageHeight - 1; i++){
        int startPixel = getStartPixel(startingPos, incrementer, i, image_scan);
        if(startPixel != null && topRow == null) {
            // Once found makes note of the highest point of the white line
            // and the column it starts at.
            topRow = i;
            toParseFrom = startPixel - 1;
        }
    }

    // Each slope will be compared to the top point of the highest white line
    double x1 = getMiddle(toParseFrom, topRow, rightSide, image_scan);

    double sumAngles = 0;

    // Finds slopes (corresponding to number of samples given) then returns the sum of all slopes
    // also counts how many of them are valid.
    for (double division = 0; division < numSamples && (numSamples + topRow) < imageHeight; division++){
        double yCompared = numSamples + topRow;
        double xCompared = getMiddle(toParseFrom, (int) yCompared, rightSide, image_scan);

        double foundAngle;
        double foundSlope;

        if(xCompared == null)
            // slope is invalid so nothing is added to sum of all angles.
            foundAngle = 0;
        else {
            // slope is valid, find the angle compared the the positive y-axis.
            foundSlope =  - (xCompared - x1)/ (yCompared - topRow);
            foundAngle = atan(foundSlope);
            // increment amount of valid samples
            validSamples++;
        }

        sumAngles += foundAngle;
    }

    if (validSamples == 0)
        return INVALID;
    else
        return (int) (sumAngles / validSamples * 180.0 / PI); // returns the angle in degrees
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
    double speedToMap = abs((int) desiredAngle);
    // the higher the desired angle, the higher the angular speed
    return mapRange(speedToMap, 0, 90, 0, 100);

}

/**
 *  Returns the desired forward speed
 *
 *  @param desiredAngle
 *      angle robot will turn
 *  @returns double
 *      moving speed percentage of robot
 */
double VisionDecision::getDesiredSpeed(double desiredAngle){
    double speedToMap = abs((int)desiredAngle);
    // the higher the desired angle the lower the linear speed.
    return 100 - mapRange(speedToMap, 0, 90, 0, 100);
}


/* Helper functions for functions that determine robot movement. */

/**
 * Determines the middle of the white line given a row
 *
 * @param startingPos which column to start parsing in
 * @param row the row to parse
 * @param rightSide determines whether to parse from the right or the left
 * @param image_scan the image to parse
 */
int VisionDecision::getMiddle(int startingPos, int row, bool rightSide, const sensor_msgs::Image::ConstPtr& image_scan){

    int incrementer;
    int startPixel, endPixel;

    // Depending on chosen side, determine where to start
    // and how to iterate.
    if(rightSide){
        incrementer = -1;
    }else{
        incrementer = 1;
    }

    // Find first pixel of the white line in a certain row.
    startPixel = VisionDecision::getStartPixel(startingPos, incrementer, row, image_scan);

    // Find last pixel of the white line in a certain row.
    endPixel = VisionDecision::getEndPixel(startPixel, incrementer, row, image_scan);

    // Return average of the two pixels.
    return (startPixel + endPixel) / 2;
}

/**
<<<<<<< f94a814e7838a4e637d0236ce273e3e1dd489a35
 * Returns the white pixel right after the black space in between
 * the line and the left side or right side of the screen.
 *
 * @param startingPos column to start parsing
 * @param NOISEMAX how large noise can be
 * @param incrementer decides whether to parse from the left or from the right
 * @param row determines the row to parse
 * @param image_scan the image to parse
 *
 * @returns the white pixel's column position, null if none found
 */
int VisionDecision::getStartPixel(int startingPos, int incrementer, int row,
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
            if (whiteVerificationCount == NOISEMAX && startPixel == null)
                startPixel = toBeChecked;
        } else {
            blackVerificationCount++;
            if(blackVerificationCount == NOISEMAX) {
                whiteVerificationCount = 0; // Reset verification if black pixel.
                toBeChecked = null;
            }
        }
        // Go to next element
        column += incrementer;
    }
    return startPixel;
}

/**
 * Returns the white pixel right before the black space in between
 * the lines
 *
 * @param startingPos column to start parsing
 * @param NOISEMAX how large noise can be
 * @param incrementer decides whether to parse from the left or from the right
 * @param row determines the row to parse
 * @param image_scan the image to parse
 *
 * @returns the white pixel's column position, null if none found
 */
int VisionDecision::getEndPixel(int startingPos, int incrementer, int row,
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
            if (blackVerificationCount == NOISEMAX && endPixel == null)
                endPixel = toBeChecked;
        } else {
            whiteVerificationCount++;
            if(whiteVerificationCount == NOISEMAX) {
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