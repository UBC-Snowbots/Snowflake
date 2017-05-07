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
 * Mapping is performed IN PLACE
 *
 * @param input the value to be mapped
 * @param input_min the minimum possible value of the input range
 * @param input_max the maximum possible value of the input range
 * @param output_min the minimum possible value of the output range
 * @param output_max the maximum possible value of the output range
 **/
void map(double& input, double input_min, double input_max,
        double output_min, double output_max) {
    // Make sure our input is within acceptable input bounds
    if (input < input_min) input = input_min;
    else if (input > input_max) input = input_max;
    // Map our input value from the input range to the output range
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
    SB_getParam(private_nh, "steering_trim", steering_trim, 0.0);

    // Ensure that the absolute values are positive
    max_abs_linear_speed = fabs(max_abs_linear_speed);
    max_abs_angular_speed = fabs(max_abs_angular_speed);

    // Map the correction value to one the arduino can understand
    map(steering_trim, -1, 1, 1, 255);

    // Open the given serial port
    arduino.Open(port);
    arduino.SetBaudRate(LibSerial::SerialStreamBuf::BAUD_115200);
    arduino.SetCharSize(LibSerial::SerialStreamBuf::CHAR_SIZE_8);
}

SteeringDriver::~SteeringDriver() {
    arduino.Close();
}

// TODO: We need to account for error here by calculating the number of expected ticks and compare them the actual number of ticks given the last command we sent to the robot
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

    // Negate the angular values, because firmware doesn't obey ROS coordinate systems
    for (double& val : angular) val *= -1;

    // TODO: 0 and 255 should be constants or member variables (arduino mapping side)
    // Map the twist message values to ones the arduino can understand
    for (double& val : linear) map(val, -max_abs_linear_speed, max_abs_linear_speed,
                                   1, 255);
    for (double& val : angular) map(val, -max_abs_angular_speed, max_abs_angular_speed,
                                    1, 255);

    // Send the message over to the arduino
    // TODO: "B" should be a member variable or constant
    arduino << "B";
    for (double val : linear) arduino << (char)val;
    for (double val : angular) arduino << (char)val;
    // Send the correction constant to the arduino
    arduino << (char)steering_trim;

    // Print the values we sent for debugging purposes
    // TODO: These should be "ROSINFO" and should have some descriptions
    // TODO: maybe add a debug flag? or just delete them if we don't need them
    for (double& val : linear) std::cout << (int)val << std::endl;
    for (double& val : angular) std::cout << (int)val << std::endl;
    std::cout << (int)steering_trim << std::endl << std::endl;
}
