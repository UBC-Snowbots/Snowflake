#include <UsbDetector.h>

/* Finds all arduino unos connected to the computer.
 *
 * stores all newly found devices in the set of arduinos
*/

UsbDetector::UsbDetector() {
    libusb_init(&ctx);
}

UsbDetector::~UsbDetector() {
    libusb_free_device_list(list, 1);
    libusb_exit(ctx);
}

std::set<libusb_device *> UsbDetector::find_arduinos() {
    cnt = libusb_get_device_list(ctx, &list);
    int err = 0;
    std::cout << cnt << std::endl;
    if (cnt < 0) {
        std::cerr << "error" << std::endl;
    }
    for (int i = 0; i < cnt; i++) {
        struct libusb_device_descriptor info;
        libusb_get_device_descriptor(list[i], &info);
        if (info.idVendor == 0x2a03) { // 2a03 is arduino vendor id
            arduinos.insert(list[i]);
        }
    }
    return arduinos;
}

libusb_device_handle *UsbDetector::open_arduino_with_prefix(std::string prefix) {
    find_arduinos();
    size_t sz = arduinos.size();

    std::vector<libusb_device_handle *> handles(sz, NULL);;

    int i = 0;
    for (std::set<libusb_device *>::iterator it = arduinos.begin(); it != arduinos.end(); ++it) {
        libusb_device_handle *devhandle;
        int err = libusb_open(*it, &devhandle);
        if (err == 0) {
            handles[i] = devhandle;
        } else {} //TODO
        i++;
    }

    for (std::vector<libusb_device_handle *>::iterator it = handles.begin(); it != handles.end(); ++it) {
        libusb_device_handle *current_handle = *it;

        int numBytes;
        unsigned char data[10];

        int transfer = libusb_bulk_transfer(current_handle, LIBUSB_ENDPOINT_IN, data, sizeof(data), &numBytes, 0);

        std::string dataStr((char *)data, 0, 10);
        if (dataStr.compare(0, prefix.length(), prefix) == 0) {
            return *it;
        }
    }
    return NULL;
}

libusb_device_handle *UsbDetector::open_arduino_with_prefix(char *prefix) {
    std::string prefixStr(prefix);
    return open_arduino_with_prefix(prefixStr);
}
