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
#include <boost/bind.hpp>
#include <cmath>
#include <string>
#include <unistd.h>

MoveMotor::MoveMotor(int argc, char** argv, std::string node_name) {
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    std::string left_subscribe_topic  = "/integration_node/lwheels_pub_topic";
    std::string right_subscribe_topic = "/integration_node/rwheels_pub_topic";
    int queue_size                    = 1;

    // Create your Phidget channels
    for (int i = 0; i < NUM_MOTORS; i++) {
        PhidgetBLDCMotor_create(&motors[i]);
        ret = Phidget_setHubPort((PhidgetHandle) motors[i], i);
        if (ret != EPHIDGET_OK) {
            Phidget_getLastError(
            &errorCode, &errorString, errorDetail, &errorDetailLen);
            ROS_ERROR(
            "Error at set hub (%d) for port %d: %s", errorCode, i, errorString);
            return;
        }else{
            ROS_INFO("hub attached succsesfully at %d", i);
        }
        ret = Phidget_openWaitForAttachment((PhidgetHandle) motors[i], 5000);
        if (ret != EPHIDGET_OK) {
            Phidget_getLastError(
            &errorCode, &errorString, errorDetail, &errorDetailLen);
            ROS_ERROR("Error at attachment (%d) for port %d:, %s ",
                      errorCode,
                      i,
                      errorString);
            return;
        } else {
            ROS_INFO("Attached successfully for port %d", i);
        }
    }
    left_subscriber = nh.subscribe<geometry_msgs::Twist>(
    left_subscribe_topic,
    queue_size,
    boost::bind(&MoveMotor::callback, this, _1, true));
    right_subscriber = nh.subscribe<geometry_msgs::Twist>(
    right_subscribe_topic,
    queue_size,
    boost::bind(&MoveMotor::callback, this, _1, false));
}

void MoveMotor::callback(const geometry_msgs::Twist::ConstPtr& msg, bool left) {
    float velocity = msg->linear.x;
    if (left) {
        run_motors(left_motors, velocity);
    } else {
        run_motors(right_motors, velocity);
    }
}

void MoveMotor::run_motors(std::vector<int> selected_motors, float velocity) {
    PhidgetLog_enable(PHIDGET_LOG_INFO, "phidgetlog.log");
    if (velocity > 1) {
        velocity = 1.0;
    } else if (velocity < -1) {
        velocity = -1.0;
    }
    for (int i = 0; i < NUM_MOTORS / 2; i++) {
        int motor_index = selected_motors[i];
        ret = PhidgetBLDCMotor_setTargetVelocity(motors[motor_index], velocity);
        if (ret != EPHIDGET_OK) {
            Phidget_getLastError(
            &errorCode, &errorString, errorDetail, &errorDetailLen);
            ROS_ERROR("Error at set target velocity (%d) for port %d: %s",
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
            ROS_ERROR(
            "Error on close (%d) for port %d: %s", errorCode, i, errorString);
            return;
        }
        PhidgetBLDCMotor_delete(&motors[i]);
    }
}
