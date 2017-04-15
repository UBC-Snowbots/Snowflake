# Fimware

### gps_firmware

- Designed to allow pass lat and lon coordinates from a [Adafruit Ultimate GPS Breakout v3](https://www.adafruit.com/product/746) to a PC over serial
- Sends messages in the form `GPS,lat,lon,has_gps_fix`, where `has_gps_fix` is a 1 or 0 representing whether or not the GPS has a fix

char to send | what it does
-------------| ------------
`d` | toggles debug mode. in debug mode, the gps will send messages over serial as it receives them
`r` | requests the most recent gps message (it will be returned over serial)

