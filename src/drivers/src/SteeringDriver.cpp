/*
 * Created By: Gareth Ellis
 * Created On: April 16, 2017
 * Description: This node is responsible for passing twist messages received
 *              over serial to the arduino controlling the robot
 */

#include "SteeringDriver.h"

/**
 * Maps a given value from one range to another.
 *
 * If the given value is outside the input range, it will
 * treated as a closest value in the input range (either the input min or max)
 *
 * @param input the value to be mapped
 * @param input_min the minimum possible value of the input range
 * @param input_max the maximum possible value of the input range
 * @param output_min the minimum possible value of the output range
 * @param output_max the maximum possible value of the output range
 *
 * @return the mapped value
 **/
void map(double& input, double input_min, double input_max,
        double output_min, double output_max) {
    if (input < input_min) input = input_min;
    else if (input > input_max) input = input_max;
    input = ((input - input_min) * (output_max - output_min)) /
            (input_max - input_min)
            + output_min;
}

SteeringDriver::SteeringDriver(int argc, char **argv, std::string node_name) {
    // Setup ROS stuff
    ros::init(argc, argv, "steering_driver");
    ros::NodeHandle private_nh("~");
    ros::Rate loop_rate(10);

    // Setup Subscriber(s)
    twist_subscriber = private_nh.subscribe("/final_decision/twist", 10,
                                            &SteeringDriver::twistCallback, this);

    // Get Params
    SB_getParam(private_nh, "port", port, (std::string)"/dev/ttyACM0");
    SB_getParam(private_nh, "max_abs_linear_speed", max_abs_linear_speed, 5.0);
    SB_getParam(private_nh, "max_abs_angular_speed", max_abs_angular_speed, 5.0);
    SB_getParam(private_nh, "max_throttle_percentage", max_throttle_percentage, 0.15);

    // Ensure that the absolute values are positive
    max_abs_linear_speed = fabs(max_abs_linear_speed);
    max_abs_angular_speed = fabs(max_abs_angular_speed);

    // Make sure throttle is positive, cap the throttle percentage at 1,
    // and warn if we have to fix it
    if (max_throttle_percentage > 1 || max_throttle_percentage < 0) {
        double new_max_throttle_percentage = fabs(max_throttle_percentage);
        if (new_max_throttle_percentage > 1)
            new_max_throttle_percentage = 1;
        ROS_ERROR("Invalid max throttle value given. Changing from '%f' to '%f'",
            max_throttle_percentage, new_max_throttle_percentage);
        max_throttle_percentage = new_max_throttle_percentage;
    }

    // Open the given serial port
    arduino.Open(port);
    arduino.SetBaudRate(LibSerial::SerialStreamBuf::BAUD_9600);
    arduino.SetCharSize(LibSerial::SerialStreamBuf::CHAR_SIZE_8);
}

SteeringDriver::~SteeringDriver() {
    arduino.Close();
}

void SteeringDriver::twistCallback(const geometry_msgs::Twist::ConstPtr twist_msg) {
    // Get our own copies of the linear and angular components of the twist message
    std::vector<double> linear = {
            twist_msg->linear.x,
            twist_msg->linear.y,
            twist_msg->linear.z,
    };
    std::vector<double> angular = {
            twist_msg->angular.x,
            twist_msg->angular.y,
            twist_msg->angular.z,
    };

    // Translate the throttle percentage to a concrete value to send to the robot
    double max_signal = 90 + max_throttle_percentage * 90;
    double min_signal = 90 - max_throttle_percentage * 90;
    // Map the twist message values to ones the arduino can understand
    for (double& val : linear) map(val, -max_abs_linear_speed, max_abs_linear_speed,
                                   min_signal, max_signal);
    for (double& val : angular) map(val, -max_abs_angular_speed, max_abs_angular_speed,
                                    min_signal, max_signal);

    // Send the message over to the arduino
    arduino << "B";
    for (double val : linear) arduino << (char)val;
    for (double val : angular) arduino << (char)val;
}
