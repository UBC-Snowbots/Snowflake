# Fimware

### Udev Rules

Udev assigns the names specified in the rule file to the devices, which helps matching dev files with devices.

- Rule Writing Guideline
	- Create file: `/etc/udev/rules.d/10-snowbots.rules` (where xx represent the priority; Here 10 is higher than 50, the default)
	- Write rules with match keys (helps indentify devices) and assignment keys (assigns names/symlinks to identified devices)
	    - common match keys: 
	    	KERNEL - match against the kernel name for the device
	    	SUBSYSTEM - match against the subsystem of the device
	    	DRIVER - match against the name of the driver backing the device
	    - common assignment keys:
	    	NAME - the name that shall be used for the device node
	    	SYMLINK - a list of symbolic links which act as alternative names for the device node
	    	MODE - define Unix permissions, by default 0666 (read/write to owner group)
	    	GROUP - define the Unix group ownship of the device node

- Device Info Collection
	- Serial Number: `udevadm info -a -n /dev/ttyUSB1 | grep '{serial}' | head -n1`
	- Vendor ID and Product ID: `lsusb` (Bus xxx Device xxx: ID <Vendor>:<Product>)

- Example
```
SUBSYSTEMS=="usb", ATTRS{idVendor}=="<idVendor>", ATTRS{idProduct}=="<idProduct>", ATTRS{serial}=="<serial>", MODE="0666", SYMLINK+="SB-Ardu", GROUP="dialout"
```

- References
	- [Writing udev rules](http://www.reactivated.net/writing_udev_rules.html)
	- [Persistent names for usb-serial devices](http://hintshop.ludvig.co.nz/show/persistent-names-usb-serial-devices/)

- Caveat: Arduino IDE only recognizes serial ports whose names follow this format `ttyS|ttyUSB|ttyACM|ttyAMA|rfcomm|ttyO)[0-9]{1,3}`

