#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#include <MoveMotor.h>
#include <unistd.h>
#include <string>

MoveMotor::MoveMotor(int argc, char **argv, std::string node_name) {
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    std::string subscribe_topic = "/integration_node/lwheels_pub_topic";
    int queue_size = 1000;
    my_subscriber = nh.subscribe(subscribe_topic, queue_size, &MoveMotor::callback, this);
    //Create your Phidget channels
    PhidgetBLDCMotor_create(&bldcMotor0);
    int hubport = strtol(argv[1], NULL, 10);
    ret = Phidget_setHubPort((PhidgetHandle)bldcMotor0, hubport);
    if (ret != EPHIDGET_OK) {
        Phidget_getLastError(&errorCode, &errorString, errorDetail, &errorDetailLen);
        printf("Error at set hub (%d): %s", errorCode, errorString);
        return;
    }

    ret = Phidget_openWaitForAttachment((PhidgetHandle)bldcMotor0, 5000);
    if (ret != EPHIDGET_OK) {
        Phidget_getLastError(&errorCode, &errorString, errorDetail, &errorDetailLen);
        printf("Error at attachment (%d): %s", errorCode, errorString);
        return;
    }
}

void MoveMotor::callback(const geometry_msgs::Twist::ConstPtr& msg) {
    ROS_INFO("Received Twist");
    ROS_INFO("linear.x: %.2f\nangular.z: %.2f", msg->linear.x, msg->angular.z);
    PhidgetLog_enable(PHIDGET_LOG_INFO, "phidgetlog.log");
    ret = PhidgetBLDCMotor_setTargetVelocity(bldcMotor0, msg->linear.x);
    if (ret != EPHIDGET_OK) {
        Phidget_getLastError(&errorCode, &errorString, errorDetail, &errorDetailLen);
        printf("Error at set target (%d): %s", errorCode, errorString);
        return;
    }
}

void MoveMotor::close(){
    ret = Phidget_close((PhidgetHandle)bldcMotor0);
    if (ret != EPHIDGET_OK) {
        Phidget_getLastError(&errorCode, &errorString, errorDetail, &errorDetailLen);
        printf("Error on close (%d): %s", errorCode, errorString);
        return;
    }
    PhidgetBLDCMotor_delete(&bldcMotor0);
}
