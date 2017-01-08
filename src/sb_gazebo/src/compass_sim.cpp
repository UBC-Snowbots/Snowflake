/*
 * Created By: Gareth Ellis
 * Created On: October 24, 2016
 * Description: Converts a vector heading to compass heading
 */

#include <ros/ros.h>
#include "std_msgs/Float32.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Quaternion.h"
#include "tf/transform_datatypes.h"

ros::Publisher compass_pub;

/**
 * Converts a given vector to a compass heading and publishes it
 *
 * @param vector the direction vector
 * @return a compass heading from extrapolated from vector, from 0 to 2PI, increasing clockwise
 */
void odomCallBack(const geometry_msgs::Vector3Stamped::ConstPtr& vector){
    float heading = 0;
    // Compensating for the GazeboRosMagnetic plugin putting out
    // oddly scaled and shifted values for the magnetic vector
    double y = vector->vector.y * -1;
    double x = vector->vector.x;
    if (y > 0){
        if (x > 0){
            heading = 2.5*M_PI - atan(x / y);
        } else {
            heading = (M_PI / 2) - atan(x / y);
        }
    } else if (y < 0){
        heading = 1.5 * M_PI - atan(x/y);
    } else if (x < 0){
        heading = M_PI;
    } else {
        heading = 0;
    }
    heading -= M_PI/2;
    // Convert to increasing clockwise
    heading = 2*M_PI - heading;
//    ROS_INFO("Compass Heading (in radians): %f", heading);
    std_msgs::Float32 msg;
    msg.data = heading;
    compass_pub.publish(msg);
}

int main(int argc, char **argv){
    ros::init(argc, argv, "compass_sim");
    ros::NodeHandle nh;
    // Publisher for compass message
    compass_pub = nh.advertise<std_msgs::Float32>("/robot/imu/compass", 1000);
    // Subscriber for odom message
    ros::Subscriber odom_sub = nh.subscribe("/robot/magnetic_vector", 1000, odomCallBack);

    ros::spin();

    return 0;
}
