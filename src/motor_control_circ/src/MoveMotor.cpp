/*
 * Created By: Ihsan Olawale, Kevin Lin
 * Created On: August 1st, 2021
 * Description: A node that connects reads input from integration_node and then
 *              publishes to a Phidgets BLDC motor controller to spin the motor.
 *              The motor is specified the first arg passed, as seen in the
 *              launch files.
 */

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#include <MoveMotor.h>
#include <unistd.h>
#include <string>

MoveMotor::MoveMotor(int argc, char **argv, std::string node_name) {
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    std::string left_subscribe_topic = "/integration_node/lwheels_pub_topic";
    std::string right_subscribe_topic = "/integration_node/rwheels_pubc_topic";
    int queue_size = 1000;
    left_subscriber = nh.subscribe(left_subscribe_topic, queue_size, &MoveMotor::left_callback, this);
    right_subscriber = nh.subscribe(right_subscribe_topic, queue_size, &MoveMotor::right_callback, this);
    //Create your Phidget channels
    for (int i = 0; i < NUM_MOTORS; i++){
        PhidgetBLDC_Motor_create(&motors[i])
        ret = Phidget_setHubPort((PhidgetHandle)motors[i], i);
        if (ret != EPHIDGET_OK) {
            Phidget_getLastError(&errorCode, &errorString, errorDetail, &errorDetailLen);
            printf("Error at set hub (%d) for port %d: %s", errorCode, i,  errorString);
            return;
        }
        ret = Phidget_openWaitForAttachment((PhidgetHandle)motors[i], 5000);
        if (ret != EPHIDGET_OK) {
            Phidget_getLastError(&errorCode, &errorString, errorDetail, &errorDetailLen);
            printf("Error at attachment (%d) for port %d: %s, %s ", errorCode, i, errorString);
            return;
        } else {
            ROS_INFO("Attached successfully for port %d", hubport);
        }
    }
}

void MoveMotor::left_callback(const geometry_msgs::Twist::ConstPtr& msg) {
    ROS_INFO("Received Left Twist");
    ROS_INFO("linear.x: %.2f\nangular.z: %.2f", msg->linear.x, msg->angular.z);
    PhidgetLog_enable(PHIDGET_LOG_INFO, "phidgetlog.log");

    ret = PhidgetBLDCMotor_setTargetVelocity(bldcMotor0, msg->linear.x);
    if (ret != EPHIDGET_OK) {
        Phidget_getLastError(&errorCode, &errorString, errorDetail, &errorDetailLen);
        printf("Error at set target (%d): %s", errorCode, errorString);
        return;
    }
}

void MoveMotor::right_callback(const geometry_msgs::Twist::ConstPtr& msg) {
    ROS_INFO("Received Left Twist");
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
    for (int i = 0; i < NUM_MOTORS; i++){
        ret = Phidget_close((PhidgetHandle)motors[i]);
        if (ret != EPHIDGET_OK) {
            Phidget_getLastError(&errorCode, &errorString, errorDetail, &errorDetailLen);
            printf("Error on close (%d) for port %d: %s", errorCode, i, errorString);
            return;
        }
        PhidgetBLDCMotor_delete(&motors[i]);
    }
}
