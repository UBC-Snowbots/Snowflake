#include <usb.h>
#include <libusb.h>
#include <memory>
#include <list>
#include <functional>
#include <iostream>
#include <set>
#include <string>
#include <vector>

namespace USBDetection {
    using usbdev_t = std::unique_ptr<libusb_device, std::function<decltype(libusb_unref_device)>>;
    using dhandle_t = std::unique_ptr<libusb_device_handle, std::function<decltype(libusb_close)>>;
    using ctx_t = std::unique_ptr<libusb_context, std::function<decltype(libusb_exit)>>;

    class ArduinoConnection {

        private:
            dhandle_t arduino;

        public:
            ArduinoConnection() = default;
            ArduinoConnection(dhandle_t);
            std::vector<unsigned char> read();
            template <typename deserializable> void write(const deserializable &obj) {
                auto msg = obj.serialize();

                int numBytes;
                int transfer = libusb_bulk_transfer(arduino.get(), LIBUSB_ENDPOINT_OUT, msg.data(), msg.size(), &numBytes, 0);
            }
    };

    class UsbDetector {

        private:
            ctx_t ctx;
            ssize_t cnt;
            void poll_arduinos();
            std::string get_prefix(char *bytestream, int n);

        public:
            std::set<usbdev_t> find_arduinos();
            ArduinoConnection open_arduino_with_prefix(std::string);
            UsbDetector();
    };

}
