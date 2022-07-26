## Process this file with automake to produce Makefile.in

p22src = $(top_srcdir)/src
p22inc = -I$(p22src) -I$(p22src)/ext -I$(p22src)/ext/include
vscript = $(srcdir)/src/plat/linux/symversion.def

ACLOCAL_AMFLAGS= -I m4

AM_CFLAGS = $(p22inc)

#AM_CFLAGS+= -Werror
AM_CFLAGS+= -Wreturn-type -Wcast-qual -Wwrite-strings
AM_CFLAGS+= -Wpointer-arith
AM_CFLAGS+= -Wall -Wno-format-y2k
AM_CFLAGS+= -Wno-switch
AM_CFLAGS+= -Wmissing-prototypes -Wnested-externs -Wreturn-type
AM_CFLAGS+= -Wchar-subscripts
AM_CFLAGS+= -Wredundant-decls -Wsign-compare

libphidget22_la_LDFLAGS= -version-info 0:0:0
libphidget22_la_LDFLAGS+= -Wl,--version-script=$(vscript)

pkgconfigdir = $(libdir)/pkgconfig
pkgconfig_DATA = libphidget22.pc

lib_LTLIBRARIES = libphidget22.la

libphidget22_la_SOURCES = \
	src/analogsensor.c \
	src/analogsensor.h \
	src/bridge.c \
	src/bridge.h \
	src/bridgepackets.gen.c \
	src/class/accelerometer.c \
	src/class/bldcmotor.c \
	src/class/capacitivetouch.c \
	src/class/currentinput.c \
	src/class/currentoutput.c \
	src/class/dataadapter.c \
	src/class/dcmotor.c \
	src/class/dictionary.c \
	src/class/digitalinput.c \
	src/class/digitaloutput.c \
	src/class/distancesensor.c \
	src/class/encoder.c \
	src/class/firmwareupgrade.c \
	src/class/frequencycounter.c \
	src/class/generic.c \
	src/class/gps.c \
	src/class/gyroscope.c \
	src/class/hub.c \
	src/class/humiditysensor.c \
	src/class/ir.c \
	src/class/lcd.c \
	src/class/lightsensor.c \
	src/class/magnetometer.c \
	src/class/meshdongle.c \
	src/class/motorpositioncontroller.c \
	src/class/phsensor.c \
	src/class/powerguard.c \
	src/class/pressuresensor.c \
	src/class/rcservo.c \
	src/class/resistanceinput.c \
	src/class/rfid.c \
	src/class/soundsensor.c \
	src/class/spatial.c \
	src/class/stepper.c \
	src/class/temperaturesensor.c \
	src/class/voltageinput.c \
	src/class/voltageoutput.c \
	src/class/voltageratioinput.c \
	src/constants.c \
	src/constants.h \
	src/constantsinternal.h \
	src/datainterval.gen.c \
	src/debug.c \
	src/debug.h \
	src/device/accelerometerdevice.c \
	src/device/accelerometerdevice.h \
	src/device/advancedservodevice.c \
	src/device/advancedservodevice.h \
	src/device/analogdevice.c \
	src/device/analogdevice.h \
	src/device/bridgedevice.c \
	src/device/bridgedevice.h \
	src/device/dataadapterdevice.c \
	src/device/dataadapterdevice.h \
	src/device/dictionarydevice.c \
	src/device/dictionarydevice.h \
	src/device/encoderdevice.c \
	src/device/encoderdevice.h \
	src/device/firmwareupgradedevice.c \
	src/device/firmwareupgradedevice.h \
	src/device/frequencycounterdevice.c \
	src/device/frequencycounterdevice.h \
	src/device/genericdevice.c \
	src/device/genericdevice.h \
	src/device/gpsdevice.c \
	src/device/gpsdevice.h \
	src/device/hubdevice.c \
	src/device/hubdevice.h \
	src/device/interfacekitdevice.c \
	src/device/interfacekitdevice.h \
	src/device/irdevice.c \
	src/device/irdevice.h \
	src/device/leddevice.c \
	src/device/leddevice.h \
	src/device/meshdongledevice.c \
	src/device/meshdongledevice.h \
	src/device/motorcontroldevice.c \
	src/device/motorcontroldevice.h \
	src/device/phsensordevice.c \
	src/device/phsensordevice.h \
	src/device/rfiddevice.c \
	src/device/rfiddevice.h \
	src/device/servodevice.c \
	src/device/servodevice.h \
	src/device/spatialdevice.c \
	src/device/spatialdevice.h \
	src/device/stepperdevice.c \
	src/device/stepperdevice.h \
	src/device/temperaturesensordevice.c \
	src/device/temperaturesensordevice.h \
	src/device/textlcddevice.c \
	src/device/textlcddevice.h \
	src/device/vintdevice.c \
	src/device/vintdevice.h \
	src/devices.c \
	src/devices.h \
	src/dispatch.c \
	src/enumutil.gen.c \
	src/enumutil.gen.h \
	src/errorstrings.gen.c \
	src/errorstrings.gen.h \
	src/ext/cvtutf.c \
	src/ext/cvtutf.h \
	src/ext/md5.c \
	src/ext/md5.h \
	src/ext/include/avahi-client/client.h \
	src/ext/include/avahi-client/lookup.h \
	src/ext/include/avahi-client/publish.h \
	src/ext/include/avahi-common/address.h \
	src/ext/include/avahi-common/alternative.h \
	src/ext/include/avahi-common/cdecl.h \
	src/ext/include/avahi-common/defs.h \
	src/ext/include/avahi-common/domain.h \
	src/ext/include/avahi-common/error.h \
	src/ext/include/avahi-common/gccmacro.h \
	src/ext/include/avahi-common/llist.h \
	src/ext/include/avahi-common/malloc.h \
	src/ext/include/avahi-common/rlist.h \
	src/ext/include/avahi-common/simple-watch.h \
	src/ext/include/avahi-common/strlst.h \
	src/ext/include/avahi-common/thread-watch.h \
	src/ext/include/avahi-common/timeval.h \
	src/ext/include/avahi-common/watch.h \
	src/ext/mos/base64.c \
	src/ext/mos/base.c \
	src/ext/mos/bsdqueue.h \
	src/ext/mos/bsdtree.h \
	src/ext/mos/crc32.c \
	src/ext/mos/ctype.c \
	src/ext/mos/endswith.c \
	src/ext/mos/getcwd.c \
	src/ext/mos/getenv.c \
	src/ext/mos/getopt.c \
	src/ext/mos/getpasswd-unix.c \
	src/ext/mos/glob.c \
	src/ext/mos/glock.c \
	src/ext/mos/hexdump.c \
	src/ext/mos/init_daemon.c \
	src/ext/mos/init_daemon.h \
	src/ext/mos/iop.c \
	src/ext/mos/kv/kv.c \
	src/ext/mos/kv/kvent.c \
	src/ext/mos/kv/kv.h \
	src/ext/mos/kv/parse.c \
	src/ext/mos/kv/parse.h \
	src/ext/mos/kv/scan.c \
	src/ext/mos/kv/scan.h \
	src/ext/mos/malloc.c \
	src/ext/mos/malloc-user.c \
	src/ext/mos/md5c.c \
	src/ext/mos/memchr.c \
	src/ext/mos/memcmp.c \
	src/ext/mos/memmem.c \
	src/ext/mos/mkdirp.c \
	src/ext/mos/mos_assert.h \
	src/ext/mos/mos_atomic.h \
	src/ext/mos/mos_atomic-pthread.c \
	src/ext/mos/mos_base64.h \
	src/ext/mos/mos_basic.h \
	src/ext/mos/mos_basic_types.h \
	src/ext/mos/mos_byteorder.h \
	src/ext/mos/mos_crc32.h \
	src/ext/mos/mos_crc32_impl.h \
	src/ext/mos/mos_dl.h \
	src/ext/mos/mos_dl-unix.c \
	src/ext/mos/mos_endian-i386.h \
	src/ext/mos/mos_error-errno.c \
	src/ext/mos/mos_error-errno.h \
	src/ext/mos/mos_fmt.h \
	src/ext/mos/mos_fileio.h \
	src/ext/mos/mos_fileio-unix-user.c \
	src/ext/mos/mos_getpasswd.h \
	src/ext/mos/mos_hexdump.h \
	src/ext/mos/mos_iop.h \
	src/ext/mos/mos_lock.h \
	src/ext/mos/mos_lock-pthread.c \
	src/ext/mos/mos_macrocompat.h \
	src/ext/mos/mos_md5.h \
	src/ext/mos/mos_net.c \
	src/ext/mos/mos_net.h \
	src/ext/mos/mos_netops.h \
	src/ext/mos/mos_netops-unix.c \
	src/ext/mos/mos_os.c \
	src/ext/mos/mos_os.h \
	src/ext/mos/mos_os-${OS}-user.h \
	src/ext/mos/mos_random.h \
	src/ext/mos/mos_random-unix.c \
	src/ext/mos/mos_readdir.h \
	src/ext/mos/mos_readline.h \
	src/ext/mos/mos_rwrlock.h \
	src/ext/mos/mos_service.h \
	src/ext/mos/mos_setenv-native.c \
	src/ext/mos/mos_sha1.h \
	src/ext/mos/mos_sha2.h \
	src/ext/mos/mos_stacktrace-gnu.c \
	src/ext/mos/mos_stacktrace.h \
	src/ext/mos/mos_task.h \
	src/ext/mos/mos_task-pthread.c \
	src/ext/mos/mos_time.h \
	src/ext/mos/mos_time-unix.c \
	src/ext/mos/mos_tlock.c \
	src/ext/mos/mos_tlock.h \
	src/ext/mos/mos_typeio.h \
	src/ext/mos/mos_urldecode.h \
	src/ext/mos/mos_urlencode.h \
	src/ext/mos/path.c \
	src/ext/mos/pkcs5_pbkdf2.c \
	src/ext/mos/pkcs5_pbkdf2.h \
	src/ext/mos/printf-smart.c \
	src/ext/mos/process-unix.c \
	src/ext/mos/rand48.c \
	src/ext/mos/random.c \
	src/ext/mos/readdir-unix.c \
	src/ext/mos/readline.c \
	src/ext/mos/rwrlock.c \
	src/ext/mos/scanf.c \
	src/ext/mos/sha1.c \
	src/ext/mos/sha2.c \
	src/ext/mos/snprintf.c \
	src/ext/mos/strcasecmp.c \
	src/ext/mos/strchr.c \
	src/ext/mos/strcmp.c \
	src/ext/mos/strdup.c \
	src/ext/mos/strlcat.c \
	src/ext/mos/strlcpy.c \
	src/ext/mos/strlen.c \
	src/ext/mos/strncmp.c \
	src/ext/mos/strncpy.c \
	src/ext/mos/strnsep.c \
	src/ext/mos/strnstr.c \
	src/ext/mos/strrchr.c \
	src/ext/mos/strrev.c \
	src/ext/mos/strstr.c \
	src/ext/mos/strton.c \
	src/ext/mos/strtrim.c \
	src/ext/mos/time.c \
	src/ext/mos/urldecode.c \
	src/ext/mos/urlencode.c \
	src/ext/mos/vasprintf.c \
	src/formatters.c \
	src/gpp.c \
	src/gpp.h \
	src/lightning.c \
	src/lightning.h \
	src/locks.h \
	src/macros.h \
	src/manager.c \
	src/manager.h \
	src/mesh.c \
	src/mesh.h \
	src/network/channel.c \
	src/network/client.c \
	src/network/network.c \
	src/network/network.h \
	src/network/networkcontrol.c \
	src/network/openremote.h \
	src/network/server.c \
	src/network/servers.c \
	src/network/zeroconf.h \
	src/network/zeroconf-avahi.c \
	src/object.c \
	src/object.h \
	src/phidget.c \
	src/phidget.h \
	src/phidget22.c \
	src/phidget22int.h \
	src/phidgetbase.h \
	src/plat/linux/usblinux.c \
	src/spi.c \
	src/spi.h \
	src/stats.c \
	src/stats.h \
	src/supportedpacket.gen.c \
	src/usb.c \
	src/usb.h \
	src/util/config.c \
	src/util/dataadaptersupport.c \
	src/util/dataadaptersupport.h \
	src/util/irsupport.c \
	src/util/irsupport.h \
	src/util/jsmn.c \
	src/util/jsmn.h \
	src/util/json.c \
	src/util/json.h \
	src/util/log.c \
	src/util/packettracker.c \
	src/util/packettracker.h \
	src/util/packing.c \
	src/util/packing.h \
	src/util/phidgetconfig.h \
	src/util/phidgetlog.h \
	src/util/rfidsupport.c \
	src/util/rfidsupport.h \
	src/util/utils.c \
	src/util/utils.h \
	src/util/voltageinputsupport.c \
	src/util/voltageinputsupport.h \
	src/vint.c \
	src/vint.h \
	src/vintpackets.c \
	src/vintpackets.h \
	src/virtual.c \
	src/virtual.h

nodist_include_HEADERS = phidget22.h

moskvincludedir = $(includedir)/mos/kv
moskvinclude_HEADERS = \
	src/ext/mos/kv/kv.h \
	src/ext/mos/kv/parse.h \
	src/ext/mos/kv/scan.h

mosincludedir = $(includedir)/mos
mosinclude_HEADERS = \
	src/ext/mos/bsdqueue.h \
	src/ext/mos/bsdtree.h \
	src/ext/mos/init_daemon.h \
	src/ext/mos/mos_assert.h \
	src/ext/mos/mos_atomic.h \
	src/ext/mos/mos_base64.h \
	src/ext/mos/mos_basic.h \
	src/ext/mos/mos_basic_types.h \
	src/ext/mos/mos_byteorder.h \
	src/ext/mos/mos_crc32.h \
	src/ext/mos/mos_crc32_impl.h \
	src/ext/mos/mos_dl.h \
	src/ext/mos/mos_endian-i386.h \
	src/ext/mos/mos_error-errno.h \
	src/ext/mos/mos_fmt.h \
	src/ext/mos/mos_fileio.h \
	src/ext/mos/mos_getpasswd.h \
	src/ext/mos/mos_hexdump.h \
	src/ext/mos/mos_iop.h \
	src/ext/mos/mos_lock.h \
	src/ext/mos/mos_macrocompat.h \
	src/ext/mos/mos_md5.h \
	src/ext/mos/mos_net.h \
	src/ext/mos/mos_netops.h \
	src/ext/mos/mos_os.h \
	src/ext/mos/mos_os-${OS}-user.h \
	src/ext/mos/mos_random.h \
	src/ext/mos/mos_readdir.h \
	src/ext/mos/mos_readline.h \
	src/ext/mos/mos_rwrlock.h \
	src/ext/mos/mos_service.h \
	src/ext/mos/mos_sha1.h \
	src/ext/mos/mos_sha2.h \
	src/ext/mos/mos_stacktrace.h \
	src/ext/mos/mos_task.h \
	src/ext/mos/mos_time.h \
	src/ext/mos/mos_tlock.h \
	src/ext/mos/mos_typeio.h \
	src/ext/mos/mos_urldecode.h \
	src/ext/mos/mos_urlencode.h \
	src/ext/mos/pkcs5_pbkdf2.h

CLEANFILES = \
	phidget22.h \
	phidget22matlab.h \
	phidget22.jar \
	Java/com/phidgets/*.class \
	Java/com/phidgets/event/*.class \
	config.log \
	libphidget22.pc

EXTRA_DIST = \
	plat/linux/udev/99-libphidget22.rules \
	libphidget22.pc.in \
	cppheader \
	cppfooter \
	library_version \
	src/plat/linux/symversion.def \
	src/bridge.gen.c \
	src/bridge.gen.h \
	src/bridgepackets.gen.h \
	src/class/accelerometer.gen.c \
	src/class/accelerometer.gen.h \
	src/class/bldcmotor.gen.c \
	src/class/bldcmotor.gen.h \
	src/class/capacitivetouch.gen.c \
	src/class/capacitivetouch.gen.h \
	src/class/currentinput.gen.c \
	src/class/currentinput.gen.h \
	src/class/currentoutput.gen.c \
	src/class/currentoutput.gen.h \
	src/class/dataadapter.gen.c \
	src/class/dataadapter.gen.h \
	src/class/dcmotor.gen.c \
	src/class/dcmotor.gen.h \
	src/class/dictionary.gen.c \
	src/class/dictionary.gen.h \
	src/class/digitalinput.gen.c \
	src/class/digitalinput.gen.h \
	src/class/digitaloutput.gen.c \
	src/class/digitaloutput.gen.h \
	src/class/distancesensor.gen.c \
	src/class/distancesensor.gen.h \
	src/class/encoder.gen.c \
	src/class/encoder.gen.h \
	src/class/firmwareupgrade.gen.c \
	src/class/firmwareupgrade.gen.h \
	src/class/frequencycounter.gen.c \
	src/class/frequencycounter.gen.h \
 	src/class/generic.gen.c \
 	src/class/generic.gen.h \
	src/class/gps.gen.c \
	src/class/gps.gen.h \
	src/class/gyroscope.gen.c \
	src/class/gyroscope.gen.h \
	src/class/hub.gen.c \
	src/class/hub.gen.h \
	src/class/humiditysensor.gen.c \
	src/class/humiditysensor.gen.h \
	src/class/ir.gen.c \
	src/class/ir.gen.h \
	src/class/lcd.gen.c \
	src/class/lcd.gen.h \
	src/class/lightsensor.gen.c \
	src/class/lightsensor.gen.h \
	src/class/magnetometer.gen.c \
	src/class/magnetometer.gen.h \
	src/class/meshdongle.gen.c \
	src/class/meshdongle.gen.h \
	src/class/phsensor.gen.c \
	src/class/phsensor.gen.h \
	src/class/motorpositioncontroller.gen.c \
	src/class/motorpositioncontroller.gen.h \
	src/class/powerguard.gen.c \
	src/class/powerguard.gen.h \
	src/class/pressuresensor.gen.c \
	src/class/pressuresensor.gen.h \
	src/class/rcservo.gen.c \
	src/class/rcservo.gen.h \
	src/class/resistanceinput.gen.c \
	src/class/resistanceinput.gen.h \
	src/class/rfid.gen.c \
	src/class/rfid.gen.h \
	src/class/soundsensor.gen.c \
	src/class/soundsensor.gen.h \
	src/class/spatial.gen.c \
	src/class/spatial.gen.h \
	src/class/stepper.gen.c \
	src/class/stepper.gen.h \
	src/class/temperaturesensor.gen.c \
	src/class/temperaturesensor.gen.h \
	src/class/voltageinput.gen.c \
	src/class/voltageinput.gen.h \
	src/class/voltageoutput.gen.c \
	src/class/voltageoutput.gen.h \
	src/class/voltageratioinput.gen.c \
	src/class/voltageratioinput.gen.h \
	src/constants.gen.h \
	src/devices.gen.c \
	src/devices.gen.h \
	src/phidget22int.gen.h \
	src/plat/labview/phidget_labview.gen.c \
	src/plat/labview/phidget_labview.gen.h \
	src/structio.gen.c \
	src/structio.gen.h \
	src/types.gen.h \
	src/vintpackets.gen.c

if LABVIEW
libphidget22_la_SOURCES += src/plat/labview/phidget_labview.c
LABVIEW_CINTOOLS_PATH ?= /usr/local/natinst/LabVIEW-8.5/cintools
AM_CFLAGS += -DCOMPILE_PHIDGETS_LABVIEW -I$(LABVIEW_CINTOOLS_PATH)
AM_CFLAGS += -L$(LABVIEW_CINTOOLS_PATH)
AM_CFLAGS += -Wl,-rpath,$(LABVIEW_CINTOOLS_PATH) -llv
endif

if SPI
libphidget22_la_SOURCES += src/plat/linux/spilinux.c
endif
	
if RUN_LDCONFIG
install-exec-hook:
	-$(LDCONFIG)
endif

phidget22.h:
	@echo "Making $@"
	@echo "#ifndef PHIDGET22_H" > $@
	@echo "#define PHIDGET22_H" >> $@
	@cat $(top_srcdir)/cppheader >> $@
	@cat $(p22src)/constants.h >> $@
	@cat $(p22src)/constants.gen.h >> $@
	$(CC) -g -E -P $(phidget22FLAGS) $(p22inc) ${DEFS} -DEXTERNALPROTO $(p22src)/phidget22int.h | sed -e "s/extern \"C\" //g" >> $@
	@cat $(top_srcdir)/cppfooter >> $@
	@echo "#endif" >> $@


