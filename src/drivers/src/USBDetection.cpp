#include <USBDetection.h>

/* Finds all arduino unos connected to the computer.
 *
 * stores all newly found devices in the set of arduinos
*/
namespace USBDetection {

    UsbDetector::UsbDetector() {
        libusb_context *tmp;
        libusb_init(&tmp);
        ctx = ctx_t(tmp,libusb_exit);
    }

    std::set<usbdev_t> UsbDetector::find_arduinos() {
        libusb_device **list;
        cnt = libusb_get_device_list(ctx.get(), &list);

        std::set<usbdev_t> arduinos;

        int err = 0;
        std::cout << cnt << std::endl;
        if (cnt < 0) {
            std::cerr << "error" << std::endl;
        }
        for (int i = 0; i < cnt; i++) {
            struct libusb_device_descriptor info;
            libusb_get_device_descriptor(list[i], &info);
            if (info.idVendor == 0x2a03) { // 2a03 is arduino vendor id
                libusb_ref_device(list[i]);
                arduinos.insert(usbdev_t(list[i], libusb_unref_device));
            }
        }
        libusb_free_device_list(list, 1);
        return arduinos;
    }

    ArduinoConnection UsbDetector::open_arduino_with_prefix(std::string prefix) {
        auto arduinos = find_arduinos();

        size_t sz = arduinos.size();
        std::vector<dhandle_t> handles(sz);

        int i = 0;
        for (usbdev_t const &it : arduinos) {
            libusb_device_handle *devhandle;
            int err = libusb_open(it.get(), &devhandle);
            if (err == 0) {
                handles[i] = dhandle_t(devhandle, libusb_close);
            } else {} //TODO
            i++;
        }

        for (dhandle_t &it : handles) {
            libusb_device_handle *current_handle = it.get();

            int numBytes;
            unsigned char data[512];

            // takes 512 bytes from the libusb bulk transfer from the current handle
            // LIBUSB_ENDPOINT_IN is a constant defined in libusb.
            // It is used to specify that the computer is receiving data from the USB device, not transmitting.
            int transfer = libusb_bulk_transfer(current_handle, LIBUSB_ENDPOINT_IN, data, sizeof(data), &numBytes, 0);

            std::string dataStr((char *)data, 0, 10);
            if (dataStr.compare(0, prefix.length(), prefix) == 0) {
                return ArduinoConnection(std::move(it));
            }
        }
        return ArduinoConnection();
    }

    ArduinoConnection::ArduinoConnection(dhandle_t handle) {
        arduino = std::move(handle); 
    }

    /*void ArduinoConnection::write(const std::string &msg) {
        unsigned char *data = reinterpret_cast<unsigned char *>(const_cast<char *>((const_cast<std::string&>(msg)).c_str()));

        ssize_t data_sz = msg.length();
        int numBytes;
        int transfer = libusb_bulk_transfer(arduino, LIBUSB_ENDPOINT_OUT, data, data_sz, &numBytes, 0);
    }*/

    std::vector<unsigned char> ArduinoConnection::read() {
        unsigned char data[512];
        int numBytes;

        // takes 512 bytes from the libusb bulk transfer from the current handle
        // LIBUSB_ENDPOINT_IN is a constant defined in libusb.
        // It is used to specify that the computer is receiving data from the USB device, not transmitting.
        int transfer = libusb_bulk_transfer(arduino.get(), LIBUSB_ENDPOINT_IN, data, sizeof(data), &numBytes, 0);

        return std::vector<unsigned char>(data, data+numBytes);
    }
}
