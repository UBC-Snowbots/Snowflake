/*
 * Created By: Valerian Ratu
 * Created On: May 1 2017
 * Description: A node which transforms the pointcloud output on the Zed
 *              of the gazebo simulation
 */

#include <sensor_msgs/PointCloud2.h>
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

ros::Publisher pub;

void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    try {

        sensor_msgs::PointCloud2 output;
        tf2_ros::Buffer tfBuffer;
        tf2_ros::TransformListener tfListener(tfBuffer);
        geometry_msgs::TransformStamped tstamped;
        tstamped = tfBuffer.lookupTransform(msg->header.frame_id, "zed_pointcloud", ros::Time(0), ros::Duration(1.0));
        ROS_INFO("Frame %s -> %s", msg->header.frame_id.c_str(), "zed_pointcloud");
        ROS_INFO("Transform: x:%f y:%f z:%f x:%f y:%f z:%f w:%f",
                 tstamped.transform.translation.x,
                 tstamped.transform.translation.y,
                 tstamped.transform.translation.z,
                 tstamped.transform.rotation.x,
                 tstamped.transform.rotation.y,
                 tstamped.transform.rotation.z,
                 tstamped.transform.rotation.w);
        tf2::doTransform(*msg, output, tstamped);
        pub.publish(output);

    } catch (tf2::TransformException ex){
        ROS_WARN("%s", ex.what());
        ros::Duration(1.0).sleep();
    }
}

int main(int argc, char** argv){
    ros::init(argc, argv, "zed_transform");
    ros::NodeHandle nh;
    pub = nh.advertise<sensor_msgs::PointCloud2>("/zed/camera/point_cloud/cloud_corrected", 1);
    ros::Subscriber sub = nh.subscribe("/zed/camera/point_cloud/cloud_registered", 1, pointCloudCallback);
    ros::spin();
    return 0;
}
