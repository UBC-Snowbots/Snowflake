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
#define BUFF_SIZE 1000

/**
 * Checks whether or not a message from the arduino is a valid GPS message
 *
 * @param msg the raw message
 * @param lat a reference to a double to assign the lat value of the gps message to
 * @param lon a reference to a double to assign the lon value of the gps message to
 * @param has_fix a reference to a bool to assign the has_fix section of the GPS message to
 *
 * @return whether or not the gps message was valid
 */
bool validGpsMessage(std::string msg, double& lat, double& lon, bool& has_fix){
    // Split the string at every ',' character
    std::vector<std::string> split_string;
    std::string str;
    std::istringstream msg_stream(msg);
    while(getline(msg_stream, str, ',')) {
        split_string.push_back(str);
    }

    int has_fix_int;
    // Check that we have a GPS message
    // A message should be of the form: "GPS,double,double,1 or 0"
    // Check that the message starts with "GPS"
    if (split_string[0] != "GPS") return false;
    // Check that the doubles are valid doubles and has_fix is a valid int
    try {
        lat = std::stod(split_string[1]);
        lon = std::stod(split_string[2]);
        has_fix_int = std::stoi(split_string[3].c_str());
    } catch (...) {
        return false;
    };
    // Check that has_fix is either a 1 or a 0
    if (has_fix_int != 1 && has_fix_int != 0){
        return false;
    } else {
        has_fix = has_fix_int;
    }
    return true;
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
    // TODO - Give some indication when we attach/detach from the GPS. Could be as simple as checking how long it was since we last received a message
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
        // Get the GPS message that the arduino should send in response to our request
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