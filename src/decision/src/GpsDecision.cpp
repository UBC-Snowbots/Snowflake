/*
 * Created By: Yu Chen(Chris)
 * Created On: September 22, 2016
 * Description: The Decision Node for GPS, takes in a point relative to
 *              the robots location and heading and broadcasts a
 *              recommended twist message
 */

#include <GpsDecision.h>



// The constructor
GpsDecision::GpsDecision(int argc, char **argv, std::string node_name) {
    // Setup NodeHandles
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;
    ros::NodeHandle public_nh("~");

    // Setup Subscriber(s)
    std::string gps_filtered_topic_name = "/gps_conversion/relative_gps";
    std::string compass_topic_name = "/gps_conversion/compass";
    int refresh_rate = 10;
    gps_subscriber = public_nh.subscribe(gps_filtered_topic_name, refresh_rate, &GpsDecision::gpsCurrentCallBack, this);
    gps_subscriber = public_nh.subscribe(gps_filtered_topic_name, refresh_rate, &GpsDecision::gpsCallBack, this);
    compass_subscriber = public_nh.subscribe(compass_topic_name, refresh_rate, &GpsDecision::compassCallBack, this);
    // Setup Publisher(s)
    std::string twist_topic = public_nh.resolveName("command");
    uint32_t queue_size = 10;
    twist_publisher = nh.advertise<geometry_msgs::Twist>(twist_topic, queue_size);

}

double GpsDecision::distance(const geometry_msgs::Point::ConstPtr& relative_gps) {
    //double current_x = relative_gps->x;
    double current_x=0.0;
    double current_y=0.0;
    double current_z=0.0;

    double x=relative_gps->x;
    double y=relative_gps->y;
    double z=relative_gps->z;

    double distance=sqrt(pow((x-current_x),2)+pow((y-current_y),2)+pow((z-current_z),2));

  return distance;
}

double GpsDecision::desiredAngle(const geometry_msgs::Point relative_gps,
                                 float current_heading,geometry_msgs::Point currentPoint) {
    const double TO_DEGREES=180/(M_PI);
    const double TO_RADIANS=(M_PI)/180;

    double next_x=relative_gps.x;
    double next_y=relative_gps.y;
    //calculate the relative x and y
    double x=(next_x-currentPoint.x);
    double y=(next_y-currentPoint.y);

    //double dis=distance(relative_gps);
    double slope=x/y;
    double angle=(atan(slope));
    double AngleRelativeForward;
    double desiredAngle;
    double angleRelativePosY=angle*TO_DEGREES;
    if(x>=0 && y>=0) { //in the 2nd quadrant
        AngleRelativeForward=-(90.0-angleRelativePosY);
    }
    else if(x<0 && y>0) {  //in the 3rd quadrant
        AngleRelativeForward=-(-angleRelativePosY+90.0);
    }
    else if (x<0 && y<0) { //in the 1st quadrant
        AngleRelativeForward=-(-angleRelativePosY-90);
    }
    else if (x>=0 && y<0) { //In the 1st quadrant
        AngleRelativeForward=-(-(90+angleRelativePosY));
    }
    else if(x<0 && y==0) {
        AngleRelativeForward=0;
    }
    else if(x==0 && y<0) {
        AngleRelativeForward=90;
    }

    //calcute the angle relative to current heading
    if(current_heading>AngleRelativeForward) {
        if(current_heading-AngleRelativeForward<=180) {
            desiredAngle=-(current_heading-AngleRelativeForward);
        }
        else {
            desiredAngle=360-(current_heading-AngleRelativeForward);
        }
    }
    else if(current_heading<=AngleRelativeForward) {

        desiredAngle=AngleRelativeForward-current_heading;
        if(desiredAngle>180) {
            desiredAngle=-(360-desiredAngle);
        }
    }
    if(x==0 && y==0 && current_heading==0) {
        desiredAngle==0;
    }
    desiredAngle=desiredAngle*TO_RADIANS;
    return desiredAngle;
}



/**this function is used for rotatation */

void  GpsDecision::rotate(const geometry_msgs::Point::ConstPtr& relative_gps) {
    geometry_msgs::Twist vel_msg;
    // Get the desired Angle
    double desiredAngle = GpsDecision::desiredAngle(*relative_gps, current_heading, currentPoint);
    // Get the desired Distance
    double distance=GpsDecision::distance(relative_gps);
    // Initialize linear velocities to 0
    vel_msg.linear.y = 0;
    vel_msg.linear.z = 0;
    // Initialize x and y angular velocities to 0
    vel_msg.angular.x = 0;
    vel_msg.angular.y = 0;

    if(desiredAngle>0){  //need to turn clockwise
        vel_msg.angular.z=-abs(desiredAngle);
    }
    else if(desiredAngle<0) { //need to turn counter-clockwise
        vel_msg.angular.z=abs(desiredAngle);
    }

    // Decide how fast to move
    vel_msg.linear.x = getDesiredLinearSpeed(distance);

    // Decide how fast to turn
    vel_msg.angular.z = getDesiredAngularSpeed(desiredAngle);

    // Publish the twist message
    publishTwist(vel_msg);

}



void GpsDecision::gpsCurrentCallBack(const geometry_msgs::Point::ConstPtr& relative_gps) {
    currentPoint = *relative_gps;
}

void GpsDecision::compassCallBack(const std_msgs::Float32::ConstPtr& compass_heading) {
    current_heading=compass_heading->data;
}

//get nextpoint
void GpsDecision::gpsCallBack(const geometry_msgs::Point::ConstPtr& relative_gps) {
    //call rotate function
    rotate(relative_gps);
}

void GpsDecision::publishTwist(geometry_msgs::Twist twist){
    twist_publisher.publish(twist);
}

double GpsDecision::getDesiredAngularSpeed(double desiredAngle) {
    // the higher the desired angle, the higher the angular speed
    if(desiredAngle<0){
        desiredAngle=-desiredAngle;
    }
    return mapRange(desiredAngle, 0, 180, 0, 100);
}

double GpsDecision::getDesiredLinearSpeed(double distance) {
    // the longer the distance the lower the linear speed.
    return 100 - mapRange(distance, 0, 90, 0, 100);
}

double GpsDecision::mapRange(double in, double inMin, double inMax, double outMin, double outMax) {
    double scale =(outMax - outMin)/(inMax - inMin);
    double offset = outMin - scale * inMin;
    return scale*in+offset;
}


