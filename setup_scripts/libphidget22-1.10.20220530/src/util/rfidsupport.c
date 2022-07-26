/*
* This file is part of libphidget22
*
* Copyright 2020 Phidgets Inc <patrick@phidgets.com>
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

#include "phidgetbase.h"
#include "util/rfidsupport.h"

// Access the PhidgetRFIDSupport struct via the channel private pointer
#define RFID_SUPPORT(ch) ((PhidgetRFIDSupportHandle)(((PhidgetChannelHandle)(ch))->private))


/*
 * Public API
 */

PhidgetReturnCode
RFIDSupport_setLatestTagString(mosiop_t iop, PhidgetChannelHandle ch, const char *tagString) {

	PhidgetRFIDSupportHandle rfidSupport = RFID_SUPPORT(ch);

	mos_strncpy(rfidSupport->latestTagString, tagString, sizeof(rfidSupport->latestTagString) - 1);

	return EPHIDGET_OK;
}

PhidgetReturnCode
RFIDSupport_waitForTag(mosiop_t iop, PhidgetChannelHandle ch, const char *tagString, int timeout, mos_mutex_t* tagLock) {

	PhidgetRFIDSupportHandle rfidSupport = RFID_SUPPORT(ch);

	//Wait for this tag to show up
	while (timeout > 0) {
		if (tagLock == NULL)
			PhidgetLock(ch);
		else
			mos_mutex_lock(tagLock);

		if (!strncmp(rfidSupport->latestTagString, tagString, RFIDDevice_MAX_TAG_STRING_LEN)) {
			if (tagLock == NULL)
				PhidgetUnlock(ch);
			else
				mos_mutex_unlock(tagLock);
			return (EPHIDGET_OK);
		}
		if (tagLock == NULL)
			PhidgetUnlock(ch);
		else
			mos_mutex_unlock(tagLock);
		mos_usleep(50000);
		timeout -= 50;
	}

	return (MOS_ERROR(iop, EPHIDGET_TIMEOUT, "Timed out waiting for tag to appear after writing. Try again."));
}

void
PhidgetRFIDSupport_free(PhidgetRFIDSupportHandle *arg) {

	if (arg == NULL || *arg == NULL)
		return;

	mos_free(*arg, sizeof(PhidgetRFIDSupport));
	*arg = NULL;
}

PhidgetReturnCode
PhidgetRFIDSupport_create(PhidgetRFIDSupportHandle *rfid) {

	TESTPTR_PR(rfid);
	*rfid = mos_zalloc(sizeof(PhidgetRFIDSupport));

	return (EPHIDGET_OK);
}

void
PhidgetRFIDSupport_init(PhidgetRFIDSupportHandle rfid) {

	assert(rfid);
}
