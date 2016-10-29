#include <UsbDetector.h>
#include <libusb.h>

std::vector<struct usb_device> UsbDetector::find_arduinos() {
    // discover devices
    libusb_init(NULL);
    libusb_device **list;
    ssize_t cnt = libusb_get_device_list(NULL, &list);
    ssize_t i = 0;
    int err = 0;
    std::cout << cnt << std::endl;
    if (cnt < 0) {
        std::cerr << "error" << std::endl;
    }
    for (int i = 0; i < cnt; i++) {
        struct libusb_device_descriptor info;
        libusb_get_device_descriptor(list[i], &info);
        if (info.idVendor == 0x2a03 && info.idProduct == 0x0043) {

        }
    }
}
