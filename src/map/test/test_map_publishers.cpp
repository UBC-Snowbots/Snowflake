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
#include <sensor_msgs/point_cloud2_iterator.h>
#include <geometry_msgs/Pose2D.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <geometry_msgs/PoseStamped.h>

void publish_lidar(ros::Publisher& pub){
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr msg (new pcl::PointCloud<pcl::PointXYZRGB>);
    msg->header.frame_id = "map";
    msg->height = 1;
    uint32_t num_points = 10000;
    msg->width = num_points;


    int dim = sqrt(num_points);

    for (int x = 0; x < dim; x++){
        for (int y =0; y < dim; y++){
            pcl::PointXYZRGB a(255, 100, 100);
            a.x = x - 50;
            a.y = y;
            a.z = 0;
            if (abs(a.x) > 40) a.z = 1;
            msg->points.push_back(a);
        }
    }
    pub.publish(msg);
}

void publish_vision(ros::Publisher& pub){
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr msg (new pcl::PointCloud<pcl::PointXYZRGB>);
    msg->header.frame_id = "map";
    msg->height = 1;
    uint32_t num_points = 10000;

    msg->width = num_points;

    int dim = sqrt(num_points);

    for (int x = 0; x < dim; x++){
        for (int y =0; y < dim; y++){
            pcl::PointXYZRGB a(0, 255, 100);
            a.x = x - 50;
            a.y = y;
            a.z = 0;
            if (abs(a.x) > 20 & abs(a.x) < 30) a.z = 1;
            msg->points.push_back(a);
        }
    }
    pub.publish(msg);
}

void publish_pos(ros::Publisher& pub, geometry_msgs::PoseStamped& pos){
    pos.pose.position.x++;
    pos.header.seq++;
    pub.publish(pos);
}

int main(int argc, char** argv){
    std::string node_name = "map_test_node";
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;

    geometry_msgs::PoseStamped pos;
    pos.header.frame_id = "map";
    pos.header.seq = 1;
    pos.pose.position.x = 0;
    pos.pose.position.y = 0;

    ros::Publisher vision_pub = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB>>("vision", 1, true);
    ros::Publisher lidar_pub = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB>>("lidar", 1, true);
    ros::Publisher pos_pub = nh.advertise<geometry_msgs::PoseStamped>("position", 1, true);

    ros::Rate loop_rate(1);

    while (ros::ok()){


        publish_vision(vision_pub);
        publish_lidar(lidar_pub);
        publish_pos(pos_pub, pos);
        ros::spinOnce();
        loop_rate.sleep();

    }
}