/*
 * Created By: Raad Khan
 * Created On: April 23, 2017
 * Description: Takes in an image feed and uses LineDetect to generate
 *              lane lines and a destination point, then broadcasts a
 *              Twist message to stay within the lanes.
 */

#include "LaneFollow.h"

using namespace ros;

LaneFollow::LaneFollow(int argc, char **argv, std::string node_name) {
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    // Setup topics
    std::string input_topic  = "robot/lane_follow/raw_camera_image";
    std::string output_topic = "/lane_follow/steering_driver_output";

    // Setup image transport
    image_transport::ImageTransport it(nh);

    uint32_t queue_size = 1;

    // Set up subscriber
    image_feed_sub = it.subscribe(
    input_topic, queue_size, &LaneFollow::laneFollowCallback, this);

    // Set up publisher
    stay_in_lane_pub =
    nh.advertise<geometry_msgs::Twist>(output_topic, queue_size);
}

void LaneFollow::laneFollowCallback(const sensor_msgs::Image::ConstPtr& image) {

}
