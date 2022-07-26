/*
 * This file is part of libphidget22
 *
 * Copyright 2015 Phidgets Inc <patrick@phidgets.com>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 3 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, see
 * <http://www.gnu.org/licenses/>
 */

#ifdef SPI_SUPPORT

#include <fcntl.h>

#include "phidgetbase.h"
#include "manager.h"
#include "spi.h"
#include "gpp.h"
#include "util/utils.h"


/* Since we support only 1 SPI Phidget, and only on 1 system (VINTSBC), we can hardcode these.. */
static const char *hub_id_file =		"/sys/class/vinthub/vinthub/device/hub_id";
static const char *hub_sku_file =		"/sys/class/vinthub/vinthub/device/hub_sku";
static const char *hub_version_file =	"/sys/class/vinthub/vinthub/device/hub_version";
static const char *hub_serial_file =	"/sys/class/vinthub/vinthub/device/hub_serial";
static const char *hub_label_file =		"/sys/class/vinthub/vinthub/device/hub_label";
static const char *hub_devices_file =	"/sys/class/vinthub/vinthub/device/devices";
static const char *hub_dev_file =		"/dev/vinthub";

static int attachedSpiDevicesPopulated = PFALSE;


static PhidgetReturnCode
readHubInt(const char *file, int base, int32_t *res) {
	char buf[12];
	char *nl;
	FILE *fp;

	fp = fopen(file, "r");
	if (fp == NULL)
		return (EPHIDGET_NOENT);

	if (fgets(buf, sizeof(buf), fp) == NULL) {
		logerr("Failed read SPI Hub info (%s): %s", file, strerror(errno));
		fclose(fp);
		return (EPHIDGET_IO);
	}
	
	/* mos_strto32 can't handle \n */
	nl = mos_strchr(buf, '\n');
	if (nl)
		*nl = '\0';

	return (mos_strto32(buf, base, res));
}

static int
readHubDevicesString(char *str, size_t str_len) {
	FILE *fp;

	fp = fopen(hub_devices_file, "r");
	if (fp == NULL) {
		logerr("Failed to open SPI Hub devices file: %s", strerror(errno));
		return (-1);
	}

	if (!fgets(str, str_len, fp)) {
		logerr("Failed to read SPI Hub devices: %s", strerror(errno));
		fclose(fp);
		return (-1);
	}
	fclose(fp);

	//Remove trailing newline
	if (str[strlen(str) - 1] == '\n')
		str[strlen(str) - 1] = '\0';

	return (0);
}

static int
readHubLabelString(char *buf, size_t buf_len) {
	int ret;
	int fp;

	fp = open(hub_label_file, O_RDONLY);
	if (fp < 0) {
		logerr("Failed to open SPI Hub label file: %s", strerror(errno));
		return (-1);
	}

	ret = read(fp, buf, buf_len);
	if (ret < 0) {
		logerr("Failed to read SPI Hub label: %s", strerror(errno));
		close(fp);
		return (-1);
	}
	close(fp);

	return (0);
}

static int
readHubSKUString(char *str, size_t str_len) {
	FILE *fp;

	fp = fopen(hub_sku_file, "r");
	if (fp == NULL) {
		logerr("Failed to open SPI Hub sku file: %s", strerror(errno));
		return (-1);
	}

	if (!fgets(str, str_len, fp)) {
		logerr("Failed to read SPI Hub sku: %s", strerror(errno));
		fclose(fp);
		return (-1);
	}
	fclose(fp);

	//Remove trailing newline
	if (str[strlen(str) - 1] == '\n')
		str[strlen(str) - 1] = '\0';

	return (0);
}

PhidgetReturnCode
PhidgetSPIGetVINTDevicesString(char *str, size_t len) {
	int ret;

	ret = readHubDevicesString(str, len);

	if (ret < 0)
		return (EPHIDGET_UNEXPECTED);

	return (EPHIDGET_OK);
}

PhidgetReturnCode
PhidgetSPIOpenHandle(PhidgetDeviceHandle phid) {
	PhidgetSPIConnectionHandle conn;

	logverbose("In PhidgetSPIOpenHandle()");

	conn = PhidgetSPIConnectionCast(phid->conn);
	assert(conn);

	conn->dev = open(hub_dev_file, O_RDWR);
	if (conn->dev < 0) {
		if (errno == EBUSY){
			logverbose("failed to open SPI device (%s): %s", hub_dev_file, strerror(errno));
			return (EPHIDGET_BUSY);
		}
		logerr("failed to open SPI device (%s): %s", hub_dev_file, strerror(errno));
		return (EPHIDGET_UNEXPECTED);
	}

	return (EPHIDGET_OK);
}

PhidgetReturnCode
PhidgetSPICloseHandle(PhidgetSPIConnectionHandle conn) {

	assert(conn);

	if (conn->dev == -1)
		return (EPHIDGET_NOTATTACHED);

	stopSPIReadThread(conn);

	/* Lock so we do not close the handle while the read thread is using it */
	PhidgetRunLock(conn);
	
	close(conn->dev);
	conn->dev = -1;
	
	PhidgetRunUnlock(conn);

	return (EPHIDGET_OK);
}

PhidgetReturnCode
PhidgetSPIReadPacket(PhidgetSPIConnectionHandle conn, unsigned char *buffer, size_t *len) {
	int ret;

	assert(conn);

	ret = read(conn->dev, buffer, *len);
	if (ret >= 0) {
		*len = ret;
		return (EPHIDGET_OK);
	}

	switch (errno) {
	case ETIMEDOUT:
		logdebug("read timeout");
		return (EPHIDGET_TIMEOUT);
	default:
		logerr("Failed to read from SPI: %s", strerror(errno));
		return EPHIDGET_UNEXPECTED;
	}
}

PhidgetReturnCode
PhidgetSPISendPacket(mosiop_t iop, PhidgetSPIConnectionHandle conn, const unsigned char *buffer, size_t len) {
	ssize_t ret;

	assert(conn);

	logverbose("Writing: 0x%*D", len, buffer, " 0x");
	ret = write(conn->dev, buffer, len);
	if (ret == (ssize_t)len)
		return (EPHIDGET_OK);

	if (ret < 0) {
		switch (errno) {
		case ETIMEDOUT:
			logerr("write timeout");
			return (EPHIDGET_TIMEOUT);
		default:
			logerr("Failed to write to SPI: %s", strerror(errno));
			return (EPHIDGET_UNEXPECTED);
		}
	}

	logerr("Error writing to SPI: didn't take full packet!");
	return (EPHIDGET_UNEXPECTED);
}

PhidgetReturnCode
PhidgetSPISetLabel(PhidgetDeviceHandle phid, char *buffer) {
	assert(phid);

	return (GPP_setLabel(NULL, phid, buffer));
}

static PhidgetReturnCode
getLabel(char *label, int serialNumber) {
	char labelData[22];

	memset(label, 0, MAX_LABEL_STORAGE);

	if (readHubLabelString(labelData, sizeof(labelData)) == 0) {
		if (labelData[0] > 0) {
			if (decodeLabelString(labelData, label, serialNumber) != EPHIDGET_OK) {
				logerr("Failed to decode label string");
				memset(label, 0, MAX_LABEL_STORAGE);
			}
		}
	}

	return (EPHIDGET_OK);
}

PhidgetReturnCode
PhidgetSPIRefreshLabelString(PhidgetDeviceHandle device) {
	PhidgetSPIConnectionHandle conn;

	assert(device);
	conn = PhidgetSPIConnectionCast(device->conn);
	assert(conn);

	return getLabel(device->deviceInfo.label, device->deviceInfo.serialNumber);
}

PhidgetReturnCode
populateAttachedSPIDevices() {
	const PhidgetUniqueDeviceDef *pdd;
	int hubId, hubVersion, hubSerial;
	char label[MAX_LABEL_STORAGE];
	char skuString[64];
	PhidgetDeviceHandle phid;
	PhidgetReturnCode res;
	int ret;

	logverbose("In populateAttachedSPIDevices()");

	if (attachedSpiDevicesPopulated)
		return (EPHIDGET_OK);

	/*
	 * If the SPI driver is not loaded, this may fail.  Do not log an error
	 * as it will just be noise.
	 */
	res = readHubInt(hub_id_file, 16, &hubId);
	if (res != EPHIDGET_OK)
		return (res);

	/*
	 * If we get here, we better be able to read the rest of the SPI files.
	 */
	res = readHubInt(hub_version_file, 10, &hubVersion);
	if (res != EPHIDGET_OK) {
		logerr("failed to read hub version");
		return (res);
	}

	res = readHubInt(hub_serial_file, 10, &hubSerial);
	if (res != EPHIDGET_OK) {
		logerr("failed to read hub serial number");
		return (res);
	}

	if (hubId <= 0 || hubVersion <= 0 || hubSerial <= 0) {
		logerr("Failed to read SPI Hub attributes");
		return (EPHIDGET_UNEXPECTED);
	}

	for (pdd = Phidget_Unique_Device_Def; ((int)pdd->type) != END_OF_LIST; pdd++) {
		if (pdd->type != PHIDTYPE_SPI || hubId != pdd->productID ||
		  hubVersion < pdd->versionLow || hubVersion >= pdd->versionHigh)
			continue;

		if (getLabel(label, hubSerial))
			logerr("Failed to get SPI Hub label");
		
		if (readHubSKUString(skuString, sizeof(skuString)) != 0)
			logerr("Failed to get SPI Hub sku");

		ret = createPhidgetSPIDevice(pdd, hubVersion, label, hubSerial, skuString, &phid);
		if (ret != EPHIDGET_OK)
			return (ret);

		PhidgetSetFlags(phid, PHIDGET_SCANNED_FLAG);
		ret = deviceAttach(phid, 1);
		if (ret != EPHIDGET_OK) {
			PhidgetRelease(&phid);
			return (ret);
		}

		loginfo("Found SPI VINT Hub: 0x%04x, %d, %d", hubId, hubVersion, hubSerial);

		attachedSpiDevicesPopulated = PTRUE;

		return (EPHIDGET_OK);
	}

	logwarn("An SPI Phidget (PID: 0x%04x Version: %d) was found which is not "
		"supported by the library. A library upgrade is required to work with this Phidget",
		hubId, hubVersion);

	// Attach the unknown SPI device so the user can at least see something
	for (pdd = Phidget_Unique_Device_Def; ((int)pdd->type) != END_OF_LIST; pdd++) {
		if (pdd->uid != PHIDUID_UNKNOWNSPI)
			continue;

		if (getLabel(label, hubSerial))
			logerr("Failed to get SPI Hub label");
		
		if (readHubSKUString(skuString, sizeof(skuString)) != 0)
			logerr("Failed to get SPI Hub sku");

		ret = createPhidgetSPIDevice(pdd, hubVersion, label, hubSerial, skuString, &phid);
		if (ret != EPHIDGET_OK)
			return (ret);

		PhidgetSetFlags(phid, PHIDGET_SCANNED_FLAG);
		ret = deviceAttach(phid, 1);
		if (ret != EPHIDGET_OK) {
			PhidgetRelease(&phid);
			return (ret);
		}

		attachedSpiDevicesPopulated = PTRUE;

		return (EPHIDGET_OK);
	}
		
	attachedSpiDevicesPopulated = PTRUE;

	return (EPHIDGET_OK);
}

PhidgetReturnCode
clearAttachedSPIDevices() {
	PhidgetDeviceHandle deviceTmp;
	PhidgetDeviceHandle device;

	PhidgetWriteLockDevices();

again:
	FOREACH_DEVICE_SAFE(device, deviceTmp) {
		if (device->connType == PHIDCONN_SPI) {
			deviceDetach(device);
			goto again;
		}
	}
	PhidgetUnlockDevices();

	attachedSpiDevicesPopulated = PFALSE;
	return (EPHIDGET_OK);
}

#endif /* SPI_SUPPORT */
