/*
 * Created By: Ihsan Olawale, Tate Kolton
 * Created On: June 26th, 2021
 * Description: An example node that subscribes to a topic publishing strings,
 *              and re-publishes everything it receives to another topic with
 *              a "!" at the end
 */

#ifndef DRIVERS_ARM_DRIVER_H
#define DRIVERS_ARM_DRIVER_H
// STD Includes
#include <iostream>

// ROS Includes
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <ros/ros.h>

// Snowbots Includes
#include <sb_utils.h>

// Other
#include <SerialStream.h>

class ArmDriver {
public:
    ArmDriver(int argc, char **argv, std::string node_name);
    /**
     * Adds an exclamation point to a given string
     *
     * Some Longer explanation should go here
     *
     * @param input_string the string to add an exclamation point to
     *
     * @return input_string with an exclamation point added to it
     */
     static std::string addCharacterToString(std::string input_string, std::string suffix);
     std::string suffix;

private:
    /**
     * Callback function for when a new string is received
     *
     * @param msg the string received in the callback
     */
    void subscriberCallBack(const std_msgs::String::ConstPtr& msg);
    /**
     * Callback function for switching between the Xbox and the Pro Controller
     *
     * @param msg the boolean received in the callback
     * true when the Xbox mode is active
     * false when the Pro Controller mode is active
     */
    void controllerModeCallBack(const std_msgs::Bool::ConstPtr& msg);
    /**
     * Publishes a given string
     *
     * @param msg_to_publish the string to publish
     */
    void republishMsg(std::string msg_to_publish);

    // The SerialStream to/from the teensy
    LibSerial::SerialStream teensy;

    // The Port the teensy is connected to
    std::string port;

    // True if we are accepting commands from Xbox controller
    // false if we are accepting commands from Pro Controller
    bool xbox_mode = false;

    ros::Subscriber my_subscriber;
    ros::Publisher my_publisher;
};
#endif //DRIVERS_ARM_DRIVER_H
