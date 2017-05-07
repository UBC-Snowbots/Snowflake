# Fimware

### gps_firmware
- Designed to allow pass lat and lon coordinates from a [Adafruit Ultimate GPS Breakout v3](https://www.adafruit.com/product/746) to a PC over serial
- Sends messages in the form `GPS,lat,lon,has_gps_fix`, where `has_gps_fix` is a 1 or 0 representing whether or not the GPS has a fix

char to send | what it does
-------------| ------------
`d` | toggles debug mode. in debug mode, the gps will send messages over serial as it receives them
`r` | requests the most recent gps message (it will be returned over serial)

### differential_drive_controller
- Designed to control a differential drive robot, via a remote control or autonomously (via commands sent over usb serial)
- All commands consist of a pair of linear and angular 3D vectors, where each component of each vector is an integer between 0 and 255
- Commands should be sent in the form (But with **no** spaces): `B LinearX LinearY LinearZ AngularX AngularY AngularZ`. **Ie:** linear and angular commands in all 3 dimensions, sent as single byte integer values, prefaced by `B`
- the `steering_driver` node in the `drivers` package provides an example of how to send commands.

