/*
 * Created By: Yu Chen(Chris)
 * Created On: September 22, 2016
 * Description: The Decision Node for GPS, takes in a point relative to
 *              the robots location and heading and broadcasts a
 *              recommended twist message
 */

#include <GpsDecision.h>
#include <math.h>


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
    double tan=y/x;
    double angle=(atan(tan));
    double AngleRelativeNorth;
    double desiredAngle;
    AngleRelativeNorth=angle*TO_DEGREES;
    if(x>=0 && y>=0) { //in the 1st quadrant
        AngleRelativeNorth=90.0-AngleRelativeNorth;
    }
    else if (x<0 && y<0) { //in the 3rd quadrant
        AngleRelativeNorth=-(90+AngleRelativeNorth);
    }
    else if (x>=0 && y<0) { //In the 4th quadrant
        AngleRelativeNorth=90-AngleRelativeNorth;
    }
    else if(x<=0 && y==0) {
        AngleRelativeNorth=-90;
    }
    else if(x==0 && y<0) {
        AngleRelativeNorth=180;
    }

    //calcute the angle relative to current heading
     if(current_heading>AngleRelativeNorth) {
         if(current_heading-AngleRelativeNorth<=180) {
             desiredAngle=-(current_heading-AngleRelativeNorth);
         }
         else {
             desiredAngle=360-(current_heading-AngleRelativeNorth);
         }
     }
     else if(current_heading<=AngleRelativeNorth) {

         desiredAngle=AngleRelativeNorth-current_heading;
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

void  GpsDecision::rotate(double desiredAngle,double angular_velocity) {
    geometry_msgs::Twist vel_msg;

    vel_msg.angular.x=0;
    vel_msg.angular.y=0;

    if(desiredAngle>0){  //need to turn clockwise
        vel_msg.angular.z=-abs(desiredAngle);
    }
    else if(desiredAngle<0) { //need to turn counter-clockwise
        vel_msg.angular.z=abs(desiredAngle);
    }

    double current_angle;
    double t0=ros::Time::now().toSec();
    ros::Rate loop_rate(1000);
    do{
        //velocity_publisher.publish(vel_msg);
        double t1=ros::Time::now().toSec();
        current_angle=angular_velocity*(t1-t0);
        ros::spinOnce();
        loop_rate.sleep();
        //loop_rate
    }while(current_angle<desiredAngle);
    vel_msg.angular.z=0;
    // velocity_publisher.publish(vel_msg);
}


void GpsDecision::gpsCurrentCallBack(const geometry_msgs::Point::ConstPtr& relative_gps) {
    currentPoint = *relative_gps;
}

void GpsDecision::compassCallBack(const std_msgs::Float32::ConstPtr& compass_heading) {
    current_heading=compass_heading->data;
}

void GpsDecision::gpsCallBack(const geometry_msgs::Point::ConstPtr& relative_gps) {

    desiredAngle(*relative_gps, current_heading, currentPoint);

}

void GpsDecision::publishTwist(geometry_msgs::Twist twist){
    twist_publisher.publish(twist);
}
