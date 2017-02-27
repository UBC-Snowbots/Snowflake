/*
 * Created By: Gareth Ellis
 * Created On: September 22, 2016
 * Description: This node is responsible for passing twist messages received
 *              over serial to the arduino controlling the robot
 *
 */

#include <ros/ros.h>
#include <sb_utils.h>
#include <sensor_msgs/NavSatFix.h>
#include <SerialStream.h>
#include <iostream>
#include <string>
#include <sstream>
#include <vector>
#define BUFF_SIZE 1000

bool validGpsMessage(std::string msg, double& lat, double& lon, bool& has_fix){
    // Split the string at every ',' character
    std::vector<std::string> split_string;
    std::string str;
    std::istringstream msg_stream(msg);
    while(getline(msg_stream, str, ',')) {
        split_string.push_back(str);
    }
    // Check that we have a GPS message
    if (split_string[0] != "GPS") return false;
    lat = std::stod(split_string[1]);
    lon = std::stod(split_string[2]);
    has_fix = std::stoi(split_string[3].c_str());
}

int main(int argc, char **argv){

    // Setup ROS stuff
    ros::init(argc, argv, "gps_driver");
    ros::NodeHandle nh_private("~");
    ros::Rate loop_rate(10);
    ros::Publisher gps_lat_lon_pub = nh_private.advertise<sensor_msgs::NavSatFix>
            ("lat_and_lon", 1000);

    // Setup Arduino stuff
    // Get the arduino port from a ros param
    std::string port = "/dev/ttyACM0";
    SB_getParam(nh_private, "port", port, port);
    // TODO - Add detecton to make sure that this is the right port (ie. that this is where our gps is plugged in to)
    // Open the given serial port
    LibSerial::SerialStream arduino;
    arduino.Open(port);
    arduino.SetBaudRate(LibSerial::SerialStreamBuf::BAUD_9600);
    arduino.SetCharSize(LibSerial::SerialStreamBuf::CHAR_SIZE_8);

    while(ros::ok()){
        // Make a char array that is definitely bigger then
        // the message we'll from the arduino
        char serial_msg[BUFF_SIZE];

        // Request a message from the arduino by sending it a single 'r' char
        arduino << (char)'r';
        // Get the GPS message it should send in response
        arduino >> serial_msg;
        std::cout << serial_msg << std::flush << std::endl;

        // Check if it's a valid gps message.
        double lat, lon;
        bool gps_has_fix;
        if (validGpsMessage(std::string(serial_msg), lat, lon, gps_has_fix) &&
                gps_has_fix){
            // Create a point message and publish it
            sensor_msgs::NavSatFix curr_lat_and_lon;
            curr_lat_and_lon.latitude = lat;
            curr_lat_and_lon.longitude = lon;
            gps_lat_lon_pub.publish(curr_lat_and_lon);
        }

        // Do some ROS stuff
        loop_rate.sleep();
        ros::spinOnce();
    }


    // Once the node stops, close the stream from the arduino
    arduino.Close();

    return 0;
}