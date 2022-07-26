/* Generated: Wed Oct 05 2016 13:13:43 GMT-0600 (Mountain Daylight Time) */
/* Will not be regenerated. */

#include "phidgetbase.h"
#include "class/dictionary.gen.h"
#include "class/dictionary.gen.c"
#include "manager.h"

static void CCONV
PhidgetDictionary_errorHandler(PhidgetChannelHandle phid, Phidget_ErrorEventCode code) {
}

static void CCONV
PhidgetDictionary_free(PhidgetChannelHandle *ch) {
	_free(ch);
}

API_PRETURN
PhidgetDictionary_create(PhidgetDictionaryHandle *phidp) {
	return (_create(phidp));
}

static PhidgetReturnCode CCONV
PhidgetDictionary_setStatus(PhidgetChannelHandle phid, BridgePacket *bp) {
	return (_setStatus(phid, bp));
}

static PhidgetReturnCode CCONV
PhidgetDictionary_getStatus(PhidgetChannelHandle phid, BridgePacket **bp) {
	return (_getStatus(phid, bp));
}

static PhidgetReturnCode CCONV
PhidgetDictionary_initAfterOpen(PhidgetChannelHandle phid) {
	return (_initAfterOpen(phid));
}

static PhidgetReturnCode CCONV
PhidgetDictionary_setDefaults(PhidgetChannelHandle phid) {
	return (_setDefaults(phid));
}

static PhidgetReturnCode
PhidgetDictionary_bridgeInput(PhidgetChannelHandle phid, BridgePacket *bp) {

	return (_bridgeInput(phid, bp));
}

static void CCONV
PhidgetDictionary_fireInitialEvents(PhidgetChannelHandle phid) {
	_fireInitialEvents(phid);
}

static int
PhidgetDictionary_hasInitialState(PhidgetChannelHandle phid) {
	return (_hasInitialState(phid));
}

API_PRETURN
PhidgetDictionary_get(PhidgetDictionaryHandle ch, const char *key, char *value, size_t len) {

	TESTPTR_PR(ch);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_DICTIONARY);
	TESTATTACHED_PR(ch);

	return (bridgeSendToDeviceWithReply((PhidgetChannelHandle)ch, BP_DICTIONARYGET, NULL, NULL, (uint8_t *)value,
	  (uint32_t *)&len, "%s", key));
}

API_PRETURN
PhidgetDictionary_scan(PhidgetDictionaryHandle ch, const char *startKey, char *keys, size_t len) {

	TESTPTR_PR(ch);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_DICTIONARY);
	TESTATTACHED_PR(ch);

	if (startKey == NULL)
		startKey = "";

	return (bridgeSendToDeviceWithReply((PhidgetChannelHandle)ch, BP_DICTIONARYSCAN, NULL, NULL, (uint8_t *)keys,
	  (uint32_t *)&len, "%s", startKey));
}

API_PRETURN
PhidgetDictionary_enableStatsDictionary() {

	return (addDictionary(1, "Phidget22 Stats"));
}

API_PRETURN
PhidgetDictionary_enableControlDictionary() {

	return (addDictionary(2, "Phidget22 Control"));
}

PhidgetReturnCode
addDictionary(int serialNumber, const char *label) {
	static const PhidgetUniqueDeviceDef *pdd = NULL;
	PhidgetDeviceHandle device;
	PhidgetReturnCode res;

	if (pdd == NULL) {
		for (pdd = Phidget_Unique_Device_Def; ((int)pdd->type) != END_OF_LIST; pdd++) {
			if (pdd->id == PHIDID_DICTIONARY)
				goto founddictionary;
		}

		pdd = NULL;
		return (PHID_RETURN_ERRSTR(EPHIDGET_UNEXPECTED, "Failed to find dictionary pdd."));
	}

founddictionary:

	/*
	 * Ensure the dictionary doesn't already exist.
	 */
	PhidgetWriteLockDevices();
	FOREACH_DEVICE(device) {
		if (device->deviceInfo.class != PHIDCLASS_DICTIONARY)
			continue;

		if (device->deviceInfo.serialNumber == serialNumber) {
			PhidgetUnlockDevices();
			return (PHID_RETURN_ERRSTR(EPHIDGET_DUPLICATE, "Dictionary already exists."));
		}
	}

	res = createPhidgetVirtualDevice(pdd, 100, label, serialNumber, &device);
	if (res != EPHIDGET_OK) {
		logerr("failed to create dictionary device");
		return (PHID_RETURN(res));
	}

	res = deviceAttach(device, 0);
	if (res != EPHIDGET_OK)
		logerr("failed to attach dictionary device");

	PhidgetUnlockDevices();
	PhidgetRelease(&device);
	return (PHID_RETURN(res));
}

API_PRETURN
PhidgetDictionary_addDictionary(int serialNumber, const char *label) {

	if (serialNumber < 1000)
		return (PHID_RETURN_ERRSTR(EPHIDGET_INVALIDARG, "Serial number (%d) must be > 1000.", serialNumber));

	return (addDictionary(serialNumber, label));
}


API_PRETURN
PhidgetDictionary_removeDictionary(int serialNumber) {
	PhidgetDeviceHandle device;

	if (serialNumber < 1000)
		return (PHID_RETURN_ERRSTR(EPHIDGET_INVALIDARG, "Serial number (%d) is not in the valid range (> 1000).", serialNumber));

	PhidgetWriteLockDevices();
	FOREACH_DEVICE(device) {
		if (device->deviceInfo.serialNumber == serialNumber) {
			chlog("%"PRIphid"", device);
			deviceDetach(device);
			PhidgetUnlockDevices();
			return (EPHIDGET_OK);
		}
	}
	PhidgetUnlockDevices();

	return (PHID_RETURN_ERRSTR(EPHIDGET_NOENT, "Dictionary with serial number (%d) does not exist.", serialNumber));
}

#include "class/dictionary.gen.h"

API_PRETURN
PhidgetDictionary_loadDictionary(int serialNumber, const char *file) {
	PhidgetDictionaryHandle dict;
	PhidgetReturnCode res;
	char buf[2048];
	char val[2048];
	char key[255];
	char *sep;
	FILE *fp;

	logverbose("(%d) [%s]", serialNumber, file);

	fp = fopen(file, "r");
	if (fp == NULL) {
		return (PHID_RETURN_ERRSTR(EPHIDGET_IO, "Failed to open '%s'.", file));
	}

	res = PhidgetDictionary_create(&dict);
	if (res != EPHIDGET_OK)
		return (PHID_RETURN(res));

	res = Phidget_openWaitForAttachment((PhidgetHandle)dict, 1000);
	if (res != EPHIDGET_OK) {
		fclose(fp);
		Phidget_close((PhidgetHandle)dict);
		PhidgetDictionary_delete(&dict);
		return (PHID_RETURN_ERRSTR(res, "Failed to open dictionary %d.", serialNumber));
	}

	while (fgets(buf, sizeof(buf), fp)) {
		if (buf[0] == '#')
			continue;
		sep = mos_strchr(buf, '=');
		if (sep == NULL)
			continue;

		*sep = '\0';
		sep++;
		mos_strtrim(buf, key, sizeof(key));
		mos_strtrim(sep, val, sizeof(val));
		if (mos_strlen(key) == 0 || mos_strlen(val) == 0)
			continue;

		// XXX - this just silently continues on error - do we want to fail?
		logdebug("Dictionary %d set(%s=%s)", serialNumber, key, val);
		res = PhidgetDictionary_set(dict, key, val);
		if (res != EPHIDGET_OK)
			logerr("failed to set %s=<val> in dictionary %d: "PRC_FMT, key, serialNumber, PRC_ARGS(res));
	}

	fclose(fp);
	Phidget_close((PhidgetHandle)dict);
	PhidgetDictionary_delete(&dict);
	return (EPHIDGET_OK);
}

