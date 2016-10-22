#include <usb.h>
#include <ros/ros.h>
#include <vector>

class UsbDetector() {

    public:
        std::vector<struct usb_device> find_arduinos();

};
