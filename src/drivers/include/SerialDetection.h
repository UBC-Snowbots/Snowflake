#include <iostream>
#include <fstream>
#include <dirent.h>
#include <list>
#include <exception>
#include <utility>

class SerialDetector {
    private:
        std::list<std::string> possibleSerialFilenames;
    public:
        SerialDetector();
        std::fstream getSerialWithPrefix(std::string prefix);
};

class directorynonexistent : public std::exception {
    virtual const char * what() const throw() {
        return "Cannot open directory";
    }
} direx;
