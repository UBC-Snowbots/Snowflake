//
// Created by valerian on 05/03/17.
//

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <laser_geometry/laser_geometry.h>
#include <nav_msgs/Odometry.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "tf2");
    ros::NodeHandle nh;
    ros::Publisher pr = nh.advertise<geometry_msgs::TransformStamped>("transform", 10);
    tf2_ros::Buffer buf;
    tf2_ros::TransformListener tflist(buf);

    ros::Rate rate(10.0);
    while (nh.ok()) {
        geometry_msgs::TransformStamped tfstamped;
        try {
            tfstamped = buf.lookupTransform("odom", "lidar", ros::Time(0), ros::Duration(3.0));
        } catch (tf2::TransformException &ex) {
            ROS_WARN("%s", ex.what());
            ros::Duration(1.0).sleep();
        }

        pr.publish(tfstamped);
    }
}