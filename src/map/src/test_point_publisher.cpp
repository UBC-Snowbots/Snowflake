//
// Created by valerian on 01/04/17.
//

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <cmath>


using namespace std;

int main (int argc, char** argv)
{
    ros::init(argc, argv, "test_point_publisher");

    ros::NodeHandle nh;

    ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>("vision", 1);

    float radius = 0;
    float dradius = 0.05;

    float angle = 0;
    float dangle = 0.05;

    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        pcl::PointCloud<pcl::PointXYZ> cloud;
        cloud.height = 1;

        pcl::PointXYZ point;
        point.x = radius * cos(angle);
        point.y = radius * sin(angle);
        //ROS_INFO("Plotting on (%f, %f)", point.x, point.y);
        point.z = 1;
        cloud.push_back(point);
        radius += dradius;
        angle += dangle;

        sensor_msgs::PointCloud2 ros_cloud;
        pcl::toROSMsg(cloud, ros_cloud);
        ros_cloud.header.frame_id = "odom";
        pub.publish(ros_cloud);
        ros::spinOnce();
        loop_rate.sleep();

    }


}
