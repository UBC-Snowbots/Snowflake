#!/bin/bash

###################
# Install Arduino #
###################

echo "================================================================"
echo "Installing Arduino"
echo "================================================================"
ARDUINO_INSTALL_DIR=/usr/share
ARDUINO_DIR_NAME=arduino
ARDUINO_ARCHIVE_NAME=$ARDUINO_DIR_NAME.tar.xz

echo "Downloading Arduino"
sudo rm -rf /tmp/$ARDUINO_DIR_NAME
sudo rm -rf /tmp/$ARDUINO_ARCHIVE_NAME
wget http://downloads.arduino.cc/arduino-1.8.5-linux64.tar.xz -O /tmp/$ARDUINO_ARCHIVE_NAME

echo "Extracting Arduino"
mkdir /tmp/$ARDUINO_DIR_NAME
tar xpvf /tmp/$ARDUINO_ARCHIVE_NAME -C /tmp/$ARDUINO_DIR_NAME --strip-components 1 > /dev/null
sudo rm -rf $ARDUINO_INSTALL_DIR/$ARDUINO_DIR_NAME
sudo mv /tmp/$ARDUINO_DIR_NAME $ARDUINO_INSTALL_DIR

echo "Running the Arduino install script"
$ARDUINO_INSTALL_DIR/$ARDUINO_DIR_NAME/install.sh

echo "Linking Arduino"
sudo rm /usr/local/bin/arduino
sudo ln -s -f $ARDUINO_INSTALL_DIR/$ARDUINO_DIR_NAME/arduino /usr/local/bin/arduino

echo "Applying Snowbots modifications to core Arduino files"

## Add the `flushRX` function. For context, see issue #176
# Add the implementation to the `.cpp` file
# Remove the last line (the final `#endif`) so we're inside the guards
sed -i '$ d' $ARDUINO_INSTALL_DIR/$ARDUINO_DIR_NAME/hardware/arduino/avr/cores/arduino/HardwareSerial.cpp
# Append the implementation (and the `#endif` we just removed)
sudo tee -a $ARDUINO_INSTALL_DIR/$ARDUINO_DIR_NAME/hardware/arduino/avr/cores/arduino/HardwareSerial.cpp > /dev/null <<'TXT'
void HardwareSerial::flushRX()
{
  _rx_buffer_head = _rx_buffer_tail;
}
#endif
TXT
# Add the declaration to the `.h` file 
# Add it after the normal `flush` function in the `HardwareSerial` class
sudo sed -i '/virtual void flush/a virtual void flushRX(void);' \
/usr/share/arduino/hardware/arduino/avr/cores/arduino/HardwareSerial.h
