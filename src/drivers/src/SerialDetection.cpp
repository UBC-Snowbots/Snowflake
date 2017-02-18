/*
 * Created by: Emmanuel Sales
 * Created on: February 18, 2017
 * Description: This class is responsible for discovering application-specific arduinos 
 *              by a particular fixed-length prefix at the beginning of each line 
 *              the arduino broadcasts.
 * 
 */
 
#include <SerialDetection.h>

// Constructor for the class
SerialDetector::SerialDetector() {
    DIR *dir;
    struct dirent *ent;
    if ((dir = opendir("/dev/serial/by-id/")) != NULL) {
        while((ent = readdir(dir)) != NULL) {
            possibleSerialFilenames.push_back(std::string(ent->d_name));
        }
    } else {
        throw direx;
    }
}

// return an fstream of the arduino broadcasting the appropriate data
std::fstream SerialDetector::getSerialWithPrefix(std::string prefix) {
    for (std::list<std::string>::iterator it = possibleSerialFilenames.begin(); it != possibleSerialFilenames.end();
            ++it) {

        char line[256];
        std::fstream serial_file(*it, std::fstream::in);
        serial_file.getline(line, 256);

        std::string beginning_chars(line, 5);

        if (beginning_chars == prefix) {
            return std::move(serial_file);
        }

        serial_file.close();
    }
}
