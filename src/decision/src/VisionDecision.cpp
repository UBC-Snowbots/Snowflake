/*
 * Created By: Robyn Castro
 * Created On: September 22, 2016
 * Description: The vision decision node, takes in an image from the robot's
 *              camera and produces a recommended twist message
 */
#include <VisionDecision.h>

// The constructor
VisionDecision::VisionDecision(int argc, char **argv, std::string node_name) {
    ros::init(argc, argv, node_name);

    // Setup NodeHandles
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    // Setup Subscriber(s)
    std::string camera_image_topic_name = "/vision/filtered_image";
    int refresh_rate = 10;
    image_subscriber = private_nh.subscribe(camera_image_topic_name, refresh_rate, &VisionDecision::imageCallBack, this);

    // Setup Publisher(s)
    std::string twist_topic = private_nh.resolveName("twist");
    uint32_t queue_size = 10;
    twist_publisher = nh.advertise<geometry_msgs::Twist>(twist_topic, queue_size);

    // Get Param(s)
    SB_getParam(private_nh, "angular_vel_multiplier", angular_velocity_multiplier, 1.0);
//    SB_getParam(private_nh, "noise_max", noise_max, 1);
}

// This is called whenever a new message is received
void VisionDecision::imageCallBack(const sensor_msgs::Image::ConstPtr &image_scan) {
    // Deal with new messages here
    geometry_msgs::Twist twistMsg;

    // Decide how much to turn
    int relativeAngle = getDesiredAngle(image_scan->height / 5.0, image_scan, angular_velocity_multiplier);

    // Initialize linear velocities to 0
    twistMsg.linear.y = 0;
    twistMsg.linear.z = 0;

    // Initialize x and y angular velocities to 0
    twistMsg.angular.x = 0;
    twistMsg.angular.y = 0;

    // Decide how fast to move
    twistMsg.linear.x = getDesiredLinearSpeed(relativeAngle);

    // Decide how fast to turn
    twistMsg.angular.z = getDesiredAngularSpeed(relativeAngle);

    // Publish the twist message
    publishTwist(twistMsg);
}

void VisionDecision::publishTwist(geometry_msgs::Twist twist) {
    twist_publisher.publish(twist);
}

/* Functions to determine robot movement */

int VisionDecision::getDesiredAngle(double numSamples, const sensor_msgs::Image::ConstPtr &image_scan,
                                    double angular_velocity_multiplier) {

    int row;
    int whiteCount = 0;

    // Check if there is a white line in the way of the robot
    for(row = 0; row < image_scan->height; row++) {
        if(image_scan->data[row * image_scan->width + image_scan->height/2] != 0)
            whiteCount++;
        if(image_scan->data[row * image_scan->width + image_scan->height/3] != 0)
            whiteCount++;
        if(image_scan->data[row * image_scan->width + image_scan->height*2/3] != 0)
            whiteCount++;
    }

    // If there is no white line in front of the robot, stop.
    if(whiteCount < NOISE_MAX)
        return STOP_SIGNAL_ANGLE;

    // If there is a perpendicular line in front of the robot, stop.
    if (isPerpendicular(image_scan))
        return STOP_SIGNAL_ANGLE;

    int desiredAngle = -angular_velocity_multiplier * getAngleOfLine(false, numSamples, image_scan);

    // If angle coming from the left is invalid try going from the right.
    if (fabs(desiredAngle) > 90)
        desiredAngle = -angular_velocity_multiplier * getAngleOfLine(true, numSamples, image_scan);

    // If both cases are invalid it will stop
    if (fabs(desiredAngle) > 90)
        desiredAngle = STOP_SIGNAL_ANGLE;

    printf("DESIRED: %d\n", desiredAngle);
    return desiredAngle;
}

int VisionDecision::getAngleOfLine(bool rightSide, double numSamples, const sensor_msgs::Image::ConstPtr &image_scan) {

    // initialization of local variables.
    double imageHeight = image_scan->height;
    int incrementer;
    int startingPos;
    int validSamples = 0;

    // Assign garbage values
    int toParseFrom = -1;
    int bottomRow = -1;

    // Initialize how and where to parse.
    incrementer = initializeIncrementerPosition(rightSide, image_scan, &startingPos);

    // starts parsing from the right and finds where the lowest white line
    // begins.
    for (int i = imageHeight - 1; i > 0; i--) {
        int startPixel = getEdgePixel(startingPos, incrementer, i, image_scan, 1);
        if (startPixel != -1 && bottomRow == -1) {
            bottomRow = i;
            toParseFrom = startPixel - 1;
        }
    }

    // Each slope will be compared to the bottom point of the lowest white line
    double x1 = getMiddle(toParseFrom, bottomRow, rightSide, image_scan);

    double sumAngles = 0;
    // Finds slopes (corresponding to number of samples given) then returns the sum of all slopes
    // also counts how many of them are valid.
    for (double division = 1; (division < numSamples) && (bottomRow - division > 0); division++) {
        double yCompared = bottomRow - division;
        double xCompared = getMiddle(toParseFrom, (int) yCompared, rightSide, image_scan);

        double foundAngle;
        double foundSlope;

        if (xCompared == -1 || x1 == -1)
            // slope is invalid so nothing is added to sum of all angles.
            foundAngle = 0;
        else {
            // slope is valid, find the angle compared the the positive y-axis.
            foundSlope = -(xCompared - x1) / (yCompared - bottomRow);
            foundAngle = atan(foundSlope);
            // increment amount of valid samples
            validSamples++;
        }

        printf("x1: %f, bottomRow: %d, xCompared: %f, yCompared: %f, Found Angle: %f, Valid: %d \n", x1, bottomRow, xCompared, yCompared, foundAngle * 180 / M_PI, validSamples);
        // Add the angle to the average.
        sumAngles += foundAngle;
    }

    if (validSamples < numSamples / 3 || bottomRow == -1)
        return STOP_SIGNAL_ANGLE;
    else
        return (int) (sumAngles / validSamples * 180.0 / M_PI); // returns the angle in degrees
}

double VisionDecision::getDesiredAngularSpeed(double desiredAngle) {
    // the higher the desired angle, the higher the angular speed
    if (desiredAngle == STOP_SIGNAL_ANGLE)
        return 0;
    return mapRange(desiredAngle, -90, 90, -1, 1);

}

double VisionDecision::getDesiredLinearSpeed(double desiredAngle) {
    double speedToMap = abs((int) desiredAngle);
    if (desiredAngle == STOP_SIGNAL_ANGLE)
        return 0;
    // the higher the desired angle the lower the linear speed.
    return 1 - mapRange(speedToMap, 0, 90, 0, 1);
}


/* Helper functions for functions that determine robot movement. */

int VisionDecision::getMiddle(int startingPos, int row, bool rightSide,
                              const sensor_msgs::Image::ConstPtr &image_scan) {

    int incrementer;
    int startPixel, endPixel;

    // Depending on chosen side, determine where to start
    // and how to iterate.
    if (rightSide)
        incrementer = -1;
    else
        incrementer = 1;

    // Find first pixel of the white line in a certain row.
    startPixel = VisionDecision::getEdgePixel(startingPos, incrementer, row, image_scan, 1);

    // Find last pixel of the white line in a certain row.
    endPixel = VisionDecision::getEdgePixel(startPixel, incrementer, row, image_scan, 0);

    // Return average of the two pixels.
    return (startPixel + endPixel) / 2;

}

int VisionDecision::getEdgePixel(int startingPos, int incrementer, int row,
                                 const sensor_msgs::Image::ConstPtr &image_scan, bool isStartPixel) {
    // Initialization of local variables
    int column = startingPos;
    int whiteVerificationCount = 0;
    int blackVerificationCount = 0;

    // Initialize these to be garbage values
    int toBeChecked = -1;

    // Parse through image horizontally to find a valid pixel
    while (column < image_scan->width && column >= 0) {
        // If white pixel found start verifying if proper start.
        if ((image_scan->data[row * image_scan->width + column] == 0 && isStartPixel) ||
            (image_scan->data[row * image_scan->width + column] != 0 && !isStartPixel)){
            blackVerificationCount = 0;
            // This pixel is what we are checking
            if (toBeChecked == -1) {
                printf("To CHECK: %d\n", column);
                toBeChecked = column;
            }

            // Determine whether toBeChecked is noise
            whiteVerificationCount++;
            if (whiteVerificationCount == NOISE_MAX)
                return toBeChecked;
        } else {
            blackVerificationCount++;
            if (blackVerificationCount == NOISE_MAX) {
                whiteVerificationCount = 0; // Reset verification if black pixel.
                toBeChecked = -1;
            }
        }
        // Go to next element
        column += incrementer;
    }

    // No value found return an error.
    return -1;
}

int VisionDecision::initializeIncrementerPosition(bool rightSide, const sensor_msgs::Image::ConstPtr &image_scan, int *startingPos)
{
    // Decides how to parse depending on if rightSide is true or false.
    if (rightSide) {
        // starts at right side then increments to the left
        *startingPos = image_scan->width - 1;
        return -1;
    } else {
        // starts at left side then increments to th right
        *startingPos = 0;
        return 1;
    }
}

bool VisionDecision::isPerpendicular(const sensor_msgs::Image::ConstPtr &image_scan)
{
    int leftSidePixel = getVerticalEdgePixel(image_scan, image_scan->width/4);
    int rightSidePixel = getVerticalEdgePixel(image_scan, image_scan->width*3/4);
    return (abs(rightSidePixel - leftSidePixel) < image_scan->height/10);
}

int VisionDecision::getVerticalEdgePixel(const sensor_msgs::Image::ConstPtr &image_scan, int column)
{
    int row;
    int whiteVerificationCount = 0;
    int blackVerificationCount = 0;

    // Initialize these to be garbage values
    int toBeChecked = -1;

    // Parse vertically to find a valid starting white pixel.
    for(row = image_scan->height-1; row >= 0; row--) {
        // If white pixel found start verifying if proper start.
        if ((image_scan->data[row * image_scan->width + column] != 0)){
            blackVerificationCount = 0;
            // This pixel is what we are checking
            if (toBeChecked == -1)
                toBeChecked = row;

            // Determine whether toBeChecked is noise
            whiteVerificationCount++;
            if (whiteVerificationCount == NOISE_MAX)
                return toBeChecked;
        } else {
            blackVerificationCount++;
            if (blackVerificationCount == NOISE_MAX) {
                whiteVerificationCount = 0; // Reset verification if black pixel.
                toBeChecked = -1;
            }
        }
    }
}
double VisionDecision::mapRange(double x, double inMin, double inMax, double outMin, double outMax) {
    return (x - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
}
