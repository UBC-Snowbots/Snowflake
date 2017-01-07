/**
 * TEST NODE
 * publishes topics for map to subscribe to for testing purposes
 *
 * publishing topic:
 *  "vision" - sensor_msgs::PointCloud2
 *  "lidar" - sensor_msgs::PointCloud2
 *  "location" - geometry_msgs::Pose2D
 *
 */


#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>


void publish_vision(ros::Publisher& pub){

    sensor_msgs::PointCloud2 cloud_msgs;

}

int main(int argc, char** argv){
    string node_name = "map_test_node";
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;

    ros::Publisher vision_pub = nh.advertise<sensor_msgs::PointCloud2>("vision", 1, true);
    ros::Publisher lidar_pub = nh.advertise<sensor_msgs::PointCloud2>("lidar", 1, true);
    ros::Publisher pos_pub = nh.advertise<sensor_msgs::Pose2D>("position", 1, true);

    ros::Rate loop_rate(100);

    while (ros::ok()){

    }


}