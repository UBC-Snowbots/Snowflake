/*
 * Created by: Emmanuel Sales
 * Created on: February 18, 2017
 * Description: This class is responsible for discovering application-specific arduinos 
 *              by a particular fixed-length prefix at the beginning of each line 
 *              the arduino broadcasts.
 * 
 */
 
#include <SerialDetection.h>

SerialDetector::SerialDetector(std::string device_id, LibSerial::SerialStreamBuf::BaudRateEnum baud_rate) {
    this->device_id = device_id;
    this->baud_rate = baud_rate;
}

SerialDetector::~SerialDetector() {
    serial_stream.Close();
}

LibSerial::SerialStream& SerialDetector::getSerialStream() {
    // Check if our serial stream is valid.
    // If it is, we assume this is the correct device
    if (!serialStreamIsValid()){
        // If not, we want to find the device
        findSerialStream();
    }
    // Now return the serial stream, safe in the knowledge that it's valid
    return serial_stream;
}

bool SerialDetector::serialStreamIsValid() {
    // Check if the serial stream is open, and the file we're using
    // to access the serial stream is still here
    return serial_stream.IsOpen() && std::ifstream(serial_file_path.c_str()).good();
}

void SerialDetector::findSerialStream() {
    // Periodically search for device
    while (true) {
        auto serialDeviceFileNames = getAllSerialDevices();
        for (std::string serial_file_path : serialDeviceFileNames) {
            // Open the device
            serial_stream.Open(serial_file_path);
            serial_stream.SetBaudRate(baud_rate);
            serial_stream.SetCharSize(LibSerial::SerialStreamBuf::CHAR_SIZE_8);

            // Check that it opened successfully
            if (serial_stream.IsOpen()){
                char buffer[256];
                // "Clear" the buffer
                memset(buffer, '\0', 256);
                // TODO: THIS IS THE FUCKING BUG - We need to wait for the arduino to turn on, or the buffer will open and hang,
                // Request the ID of the device
                sleep(10);
                serial_stream << (char)'I';

                // Read the ID the device should send back
//                std::string response;
//                char serial_char;
//                while (serial_stream.read(&serial_char, 1) && serial_char != '\n'){
//                    response += serial_char;
//                }
                // TODO: YOU ARE HERE -  Seems to hang on opening the buffer after replugging usb devices
                // TODO: Can we put a timeout here so we don't just hang with the buffer open??
                serial_stream >> buffer;

                // Is this the device we're looking for?
                if (buffer == device_id){
                    this->serial_file_path = serial_file_path;
                    // TODO: Should we maybe be using a ROS stream here? Probably...
                    std::cout << "Found Device on: " << serial_file_path << std::endl;
                    return;
                }
            }
            serial_stream.Close();
        }
        // Wait for a little while so we don't spam USB devices
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
}

std::vector<std::string> SerialDetector::getAllSerialDevices() {
    // Clear out the filenames
    std::vector<std::string> possibleSerialFilenames;

    // Get all possible serial devices as files
    DIR *dir;
    struct dirent *ent;
    // Open the serial file directory
    std::string serial_folder = "/dev/serial/by-id";
    if ((dir = opendir(serial_folder.c_str())) != NULL) {
        // Run through all the files in the directory (they are all symlinks)
        while((ent = readdir(dir)) != NULL) {
            std::string filepath = serial_folder + "/" + ent->d_name;
            char* abs_path = realpath(filepath.c_str(), nullptr);
            possibleSerialFilenames.emplace_back(std::string(abs_path));
        }
    }

    return possibleSerialFilenames;
}
