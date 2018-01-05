/*
 * Created By: Gareth Ellis
 * Created On: April 16, 2017
 * Description: This node is responsible for passing twist messages received
 *              over serial to the Arduino controlling the robot
 */

#include <SteeringDriver.h>

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
void map(double& input,
         double input_min,
         double input_max,
         double output_min,
         double output_max) {
    if (input < input_min)
        input = input_min;
    else if (input > input_max)
        input = input_max;
    input =
    (input - input_min) * (output_max - output_min) / (input_max - input_min) +
    output_min;
}

SteeringDriver::SteeringDriver(int argc, char** argv, std::string node_name) {
    // Setup ROS stuff
    ros::init(argc, argv, "steering_driver");
    ros::NodeHandle private_nh("~");
    ros::Rate loop_rate(10);

    // Setup Subscriber(s)
    twist_subscriber =
    private_nh.subscribe("/cmd_vel", 10, &SteeringDriver::twistCallback, this);

    // Get Params
    SB_getParam(private_nh, "port", port, (std::string) "/dev/ttyACM0");
    SB_getParam(private_nh, "max_abs_linear_speed", max_abs_linear_speed, 2.0);
    SB_getParam(
    private_nh, "max_abs_angular_speed", max_abs_angular_speed, 1.0);
    // Ensure that the values given for max absolute speeds are positive
    max_abs_linear_speed  = fabs(max_abs_linear_speed);
    max_abs_angular_speed = fabs(max_abs_angular_speed);

    // Setup Arduino stuff
    // Get the Arduino port from a ros param
    // TODO - Give some indication when we attach/detach from the Steering
    // Controller. Could be as simple as checking how long it was since we last
    // received a message
    // Open the given serial port
    arduino.Open(port);
    arduino.SetBaudRate(LibSerial::SerialStreamBuf::BAUD_9600);
    arduino.SetCharSize(LibSerial::SerialStreamBuf::CHAR_SIZE_8);
}

void SteeringDriver::twistCallback(
const geometry_msgs::Twist::ConstPtr twist_msg) {
    // Get our own copies of the linear and angular components of the twist
    // message
    std::vector<double> linear = {
    twist_msg->linear.x, twist_msg->linear.y, twist_msg->linear.z,
    };
    std::vector<double> angular = {
    twist_msg->angular.x, twist_msg->angular.y, twist_msg->angular.z,
    };

    // Invert the angular speeds, because ROS coordinate systems.
    // Valerian: Don't think you have to?
    // for (double& val : angular) val *= -1;

    // Map the twist message values to ones the arduino can understand
    for (double& val : linear)
        map(val, -max_abs_linear_speed, max_abs_linear_speed, 1, 255);
    for (double& val : angular)
        map(val, -max_abs_angular_speed, max_abs_angular_speed, 0, 180);

    // Send the message over to the arduino
    arduino << "B";
    for (double val : linear) arduino << (char) val;
    for (double val : angular) arduino << (char) val;

    // TODO: We should be logging to ROS_INFO here
    //    std::cout << "Sending Message: ";
    //    for (double val : linear) std::cout << val << " / ";
    //    for (double val : angular) std::cout << val << " / ";
    //    std::cout << std::endl;
}
