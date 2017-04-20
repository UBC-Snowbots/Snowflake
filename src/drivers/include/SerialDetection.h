/*
 * Created By: Em Sal, Gareth Ellis
 * Created On: October 20, 2016
 * Description: Manages USB serial devices
 */

#ifndef SERIAL_DETECTION_H
#define SERIAL_DETECTION_H

// STD
#include <iostream>
#include <fstream>
#include <dirent.h>
#include <list>
#include <exception>
#include <utility>
#include <chrono>
#include <thread>
// TODO: Delete ME
//#include <unistd.h>
#include <stdlib.h>
#include <cstring>

// Other
#include <SerialStream.h>

class SerialDetector {
public:
    /**
     * Constructs a SerialDetector looking for a device with device id,
     * using a given baud rate
     *
     * @param device_id the id of the device we're looking for
     * @param baud_rate the baud_rate of the device we're looking for
     */
    SerialDetector(std::string device_id,
                   LibSerial::SerialStreamBuf::BaudRateEnum baud_rate);
    ~SerialDetector();

    /**
     * Gets a serial stream for the chosen device
     * @return A serial stream to the device chosen in the constructor
     */
    LibSerial::SerialStream& getSerialStream();

private:
    /**
     * Gets all serial devices as absolute file paths
     * Symlinks to all serial devices can be found in `/dev/serial/by-id`
     *
     * @return a vector of absolute paths to all serial devices
     */
    std::vector<std::string> getAllSerialDevices();

    /**
     * Finds a serial stream for a device with ID device_id
     * and using Baud Rate baud_rate
     */
    void findSerialStream();

    /**
     * Checks if our current serial stream is still open and valid
     *
     * @return whether or not our current serial stream is open and valid
     */
    bool serialStreamIsValid();

    // The ID of the device we're looking for
    std::string device_id;
    // The Baud rate the device will be operating at
    LibSerial::SerialStreamBuf::BaudRateEnum baud_rate;
    // A serial stream to the chosen device
    LibSerial::SerialStream serial_stream;
    // The absolute filepath to where we got the serial_stream from
    std::string serial_file_path;

};

#endif // SERIAL_DETECTION_H
