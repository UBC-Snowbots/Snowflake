/*
 * Created By: Valerian Ratu
 * Created On: May 1 2017
 * Description: A node which transforms a given pointcloud from
 *              it's current frame to a given frame
 */

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/transform_listener.h>
#include <sb_utils.h>

ros::Publisher pub;
std::string output_frame;

void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    try {
        // Create an empty pointcloud
        sensor_msgs::PointCloud2 output;
        // Transform the pointcloud to the requested frame
        SB_doTransform(*msg, output, output_frame);
        // Publish the transformed pointcloud
        pub.publish(output);
    } catch (tf2::TransformException ex){
        ROS_WARN("%s", ex.what());
        ros::Duration(1.0).sleep();
    }
}

int main(int argc, char** argv){
    ros::init(argc, argv, "pointcloud_transformer");
    ros::NodeHandle private_nh("~");
    if (!SB_getParam(private_nh, "output_frame", output_frame)){
        // Error and exit if we didn't get a frame to transform to.
        // We need this to transform anything, and there is no reasonable default
        ROS_ERROR("Param 'output_frame' not provided. " \
                  "Can't  transform anything without a frame to transform it to");
        return 1;
    }
    ros::Subscriber sub = private_nh.subscribe("/input_pointcloud", 1, pointCloudCallback);
    pub = private_nh.advertise<sensor_msgs::PointCloud2>("/output_pointcloud", 1);
    ros::spin();
    return 0;
}
