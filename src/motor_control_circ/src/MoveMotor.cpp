#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#include <MoveMotor.h>
#include <phidget22.h>
#include <unistd.h>
#include <string>

MoveMotor::MoveMotor(int argc, char **argv, std::string node_name) {
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    std::string subscribe_topic = "subscribe_topic";
    int queue_size = 10;
    my_subscriber = nh.subscribe(subscribe_topic, queue_size, &MoveMotor::callback, this);
}

void MoveMotor::callback(const geometry_msgs::Twist::ConstPtr& msg) {
    ROS_INFO("Received Twist");
    ROS_INFO("linear.x: %i\nangular.z: %i", msg->linear.x, msg->angular.z;

    PhidgetBLDCMotorHandle bldcMotor0;

    PhidgetBLDCMotor_create(&bldcMotor0);

    int channel = 2;
    Phidget_setHubPort((PhidgetHandle)bldcMotor0, channel);

    Phidget_openWaitForAttachment((PhidgetHandle)bldcMotor0, 5000);

    PhidgetBLDCMotor_setTargetVelocity(bldcMotor0, msg->linear.x);

    usleep(5 * 1000 * 1000);

    Phidget_close((PhidgetHandle)bldcMotor0);
    PhidgetBLDCMotor_delete(&bldcMotor0);
}
