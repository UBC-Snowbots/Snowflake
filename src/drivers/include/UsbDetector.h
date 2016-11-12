#include <usb.h>
#include <libusb.h>
#include <ros/ros.h>
#include <set>

class UsbDetector {

    private:
        libusb_context *ctx;
        std::set<struct usb_device> arduinos;
        libusb_device **list;
        ssize_t cnt;
        void poll_arduinos();

    public:
        std::set<struct usb_device> find_arduinos();
        UsbDetector();
        ~UsbDetector();
};
