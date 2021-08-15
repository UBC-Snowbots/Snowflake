/*
 * Created By: Ihsan Olawale, Kevin Lin
 * Created On: August 1st, 2021
 * Description: A node that connects reads input from integration_node and then
 *              publishes to Phidgets BLDC Motors. Can be tested by connecting
 *              all motors, running Pro Controller launch file, and then
 *              motors_and_integration.launch to launch this node and
 *              wheel_integration_package to translate from the controller to
 *              this node. Reference master documentation for more details.
 */

#include <geometry_msgs/Twist.h>
#include <ros/ros.h>

#include <MoveMotor.h>
#include <string>
#include <unistd.h>

MoveMotor::MoveMotor(int argc, char** argv, std::string node_name) {
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    std::string left_subscribe_topic  = "/integration_node/lwheels_pub_topic";
    std::string right_subscribe_topic = "/integration_node/rwheels_pub_topic";
    int queue_size                    = 1000;

    // Create your Phidget channels
    for (int i = 0; i < NUM_MOTORS; i++) {
        PhidgetBLDCMotor_create(&motors[i]);
        ret = Phidget_setHubPort((PhidgetHandle) motors[i], i);
        if (ret != EPHIDGET_OK) {
            Phidget_getLastError(
            &errorCode, &errorString, errorDetail, &errorDetailLen);
            printf(
            "Error at set hub (%d) for port %d: %s", errorCode, i, errorString);
            return;
        }
        ret = Phidget_openWaitForAttachment((PhidgetHandle) motors[i], 5000);
        if (ret != EPHIDGET_OK) {
            Phidget_getLastError(
            &errorCode, &errorString, errorDetail, &errorDetailLen);
            printf("Error at attachment (%d) for port %d:, %s ",
                   errorCode,
                   i,
                   errorString);
            return;
        } else {
            ROS_INFO("Attached successfully for port %d", i);
        }
    }
    left_subscriber = nh.subscribe(
    left_subscribe_topic, queue_size, &MoveMotor::left_callback, this);
    right_subscriber = nh.subscribe(
    right_subscribe_topic, queue_size, &MoveMotor::right_callback, this);
}

void MoveMotor::left_callback(const geometry_msgs::Twist::ConstPtr& msg) {
    ROS_INFO("left: linear.x: %.2f\nangular.z: %.2f", msg->linear.x, msg->angular.z);
    current_motors = left_motors;
    float velocity = msg->linear.x;
    run_motors(velocity);
}

void MoveMotor::right_callback(const geometry_msgs::Twist::ConstPtr& msg) {
    ROS_INFO("right: linear.x: %.2f\nangular.z: %.2f", msg->linear.x, msg->angular.z);
    current_motors = right_motors;
    // negative because the motors are on the opposite side
    float velocity = -1 * msg->linear.x;
    run_motors(velocity);
}

void MoveMotor::run_motors(float velocity) {
    PhidgetLog_enable(PHIDGET_LOG_INFO, "phidgetlog.log");
    for (int i = 0; i < NUM_MOTORS / 2; i++) {
        int motor_index = current_motors[i];
        ret = PhidgetBLDCMotor_setTargetVelocity(motors[motor_index], velocity);
        if (ret != EPHIDGET_OK) {
            Phidget_getLastError(
            &errorCode, &errorString, errorDetail, &errorDetailLen);
            printf("Error at set target velocity (%d) for port %d: %s",
                   errorCode,
                   motor_index,
                   errorString);
            return;
        }
    }
}

void MoveMotor::close() {
    for (int i = 0; i < NUM_MOTORS; i++) {
        ret = Phidget_close((PhidgetHandle) motors[i]);
        if (ret != EPHIDGET_OK) {
            Phidget_getLastError(
            &errorCode, &errorString, errorDetail, &errorDetailLen);
            printf(
            "Error on close (%d) for port %d: %s", errorCode, i, errorString);
            return;
        }
        PhidgetBLDCMotor_delete(&motors[i]);
    }
}
