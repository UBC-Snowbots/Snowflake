/*
 * Created By: Gareth Ellis and Chris Chen
 * Created On: September 22, 2016
 * Description: The Decision Node for GPS, takes in a point relative to
 *              the robots location and heading and broadcasts a
 *              recommended twist message
 */

#include <GpsDecision.h>


GpsDecision::GpsDecision(int argc, char **argv, std::string node_name) {
    // Setup NodeHandles
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    // Setup Subscriber(s)
    uint32_t refresh_rate = 10;
    std::string current_location_topic = "/gps_manager/current_location";
    current_location_subscriber = private_nh.subscribe(current_location_topic, refresh_rate,
                                                    &GpsDecision::currentLocationCallback, this);
    std::string heading_topic = "/gps_manager/current_heading";
    heading_subscriber = private_nh.subscribe(heading_topic, refresh_rate,
                                              &GpsDecision::headingCallback, this);
    std::string waypoint_topic = "/gps_manager/current_waypoint";
    waypoint_subscriber = private_nh.subscribe(waypoint_topic, refresh_rate,
                                               &GpsDecision::waypointCallback, this);

    // Setup Publisher(s)
    uint32_t queue_size = 10;
    std::string twist_publisher_topic = private_nh.resolveName("twist");
    twist_publisher = private_nh.advertise<geometry_msgs::Twist>(twist_publisher_topic, queue_size);

    // Misc.
    mover = Mover();
}

void GpsDecision::currentLocationCallback(const geometry_msgs::Point::ConstPtr &current_location) {
    this->current_location = *current_location;
}

void GpsDecision::headingCallback(const std_msgs::Float32::ConstPtr &heading) {
    this->current_heading = heading->data;
}

void GpsDecision::waypointCallback(const geometry_msgs::Point::ConstPtr &waypoint) {
    // Create a new twist message and publish it
    geometry_msgs::Twist twist = mover.createTwistMessage(current_location, current_heading, *waypoint);
    twist_publisher.publish(twist);
}

geometry_msgs::Twist Mover::createTwistMessage(geometry_msgs::Point current_location,
                                                     float current_heading,
                                                     geometry_msgs::Point waypoint) {

}

//
//double GpsDecision::distance(const geometry_msgs::Point::ConstPtr& relative_gps) {
//    //double current_x = relative_gps->x;
//    double current_x=0.0;
//    double current_y=0.0;
//    double current_z=0.0;
//
//    double x=relative_gps->x;
//    double y=relative_gps->y;
//    double z=relative_gps->z;
//
//    double distance=sqrt(pow((x-current_x),2)+pow((y-current_y),2)+pow((z-current_z),2));
//
//  return distance;
//}
//
//#define TO_DEGREES 180/(M_PI)
//double GpsDecision::desiredAngle(const geometry_msgs::Point relative_gps,
//                                 float current_heading,geometry_msgs::Point currentPoint) {
//
//    double next_x=relative_gps.x;
//    double next_y=relative_gps.y;
//    //calculate the relative x and y
//    double x=(next_x-currentPoint.x);
//    double y=(next_y-currentPoint.y);
//
//    //double dis=distance(relative_gps);
//    double tan=y/x;
//    double angle=(atan(tan));
//    double AngleRelativeNorth;
//    double desiredAngle;
//    AngleRelativeNorth=angle*TO_DEGREES;
//    if(x>=0 && y>=0) { //in the 1st quadrant
//        AngleRelativeNorth=90.0-AngleRelativeNorth;
//    }
//    else if (x<0 && y<0) { //in the 3rd quadrant
//        AngleRelativeNorth=-(90+AngleRelativeNorth);
//    }
//    else if (x>=0 && y<0) { //In the 4th quadrant
//        AngleRelativeNorth=90-AngleRelativeNorth;
//    }
//    else if(x<=0 && y==0) {
//        AngleRelativeNorth=-90;
//    }
//    else if(x==0 && y<0) {
//        AngleRelativeNorth=180;
//    }
//
//    //calcute the angle relative to current heading
//     if(current_heading>AngleRelativeNorth) {
//         if(current_heading-AngleRelativeNorth<=180) {
//             desiredAngle=-(current_heading-AngleRelativeNorth);
//         }
//         else {
//             desiredAngle=360-(current_heading-AngleRelativeNorth);
//         }
//     }
//     else if(current_heading<=AngleRelativeNorth) {
//
//         desiredAngle=AngleRelativeNorth-current_heading;
//         if(desiredAngle>180) {
//             desiredAngle=-(360-desiredAngle);
//         }
//     }
//    if(x==0 && y==0 && current_heading==0) {
//        desiredAngle==0;
//    }
//
//    return desiredAngle;
//}
//
//void  GpsDecision::rotate(double desiredAngle,double angular_velocity) {
//    geometry_msgs::Twist vel_msg;
//
//    vel_msg.angular.x=0;
//    vel_msg.angular.y=0;
//
//    if(desiredAngle>0){  //need to turn clockwise
//        vel_msg.angular.z=-abs(desiredAngle);
//    }
//    else if(desiredAngle<0) { //need to turn counter-clockwise
//        vel_msg.angular.z=abs(desiredAngle);
//    }
//
//    double current_angle;
//    double t0=ros::Time::now().toSec();
//    ros::Rate loop_rate(1000);
//    do{
//        //velocity_publisher.publish(vel_msg);
//        double t1=ros::Time::now().toSec();
//        current_angle=angular_velocity*(t1-t0);
//        ros::spinOnce();
//        loop_rate.sleep();
//        //loop_rate
//    }while(current_angle<desiredAngle);
//    vel_msg.angular.z=0;
//    // velocity_publisher.publish(vel_msg);
//}
//
//
//void GpsDecision::gpsCurrentCallBack(const geometry_msgs::Point::ConstPtr& relative_gps) {
//    currentPoint = *relative_gps;
//}
//
//void GpsDecision::compassCallBack(const std_msgs::Float32::ConstPtr& compass_heading) {
//    current_heading=compass_heading->data;
//}
//
//void GpsDecision::gpsCallBack(const geometry_msgs::Point::ConstPtr& relative_gps) {
//    desiredAngle(*relative_gps, current_heading, currentPoint);
//
//}
//
//void GpsDecision::publishTwist(geometry_msgs::Twist twist){
//    twist_publisher.publish(twist);
//}
