#include <UsbDetector.h>

/* Finds all arduino unos connected to the computer.
 *
 * stores all newly found devices in the set of arduinos
 *
*/

UsbDetector::UsbDetector() {
    libusb_init(ctx);
}

UsbDetector::~UsbDetector() {
    libusb_free_device_list(list);
    libusb_exit(ctx);
}

std::set<struct usb_device> UsbDetector::find_arduinos() {
    cnt = libusb_get_device_list(ctx, &list);
    int err = 0;
    std::cout << cnt << std::endl;
    if (cnt < 0) {
        std::cerr << "error" << std::endl;
    }
    for (int i = 0; i < cnt; i++) {
        struct libusb_device_descriptor info;
        libusb_get_device_descriptor(list[i], &info);
        if (info.idVendor == 0x2a03 && info.idProduct == 0x0043) {
            arduinos.insert(list[i]);
        }
    }
    return arduinos;
}

void UsbDetector::poll_arduinos() {
    for (std::set<struct usb_device>::iterator it = arduinos.begin(); it != arduinos.end(); ++it) {
     
    }
}

