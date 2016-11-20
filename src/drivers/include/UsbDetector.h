#include <usb.h>
#include <libusb.h>
#include <ros/ros.h>
#include <set>
#include <string>
#include <vector>
#include <list>

class UsbDetector {

    private:
        libusb_context *ctx;
        std::set<libusb_device *> arduinos;
        libusb_device **list;
        ssize_t cnt;
        void poll_arduinos();

    public:
        std::set<libusb_device *> find_arduinos();
        libusb_device_handle *open_arduino_with_prefix(char *);
        libusb_device_handle *open_arduino_with_prefix(std::string);
        UsbDetector();
        ~UsbDetector();
};
