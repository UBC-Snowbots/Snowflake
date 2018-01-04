# Drivers

### Misc Notes:
- the `SerialDetection` class can get confused if one device is unplugged, and another one is plugged back in and allocated the same serial port as the first, all before the `getSerialStream()` is called again
