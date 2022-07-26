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

#include "phidgetbase.h"
#include "device/gpsdevice.h"

// === Internal Functions === //
static BOOL checkcrc(char *data, int crc);
static PhidgetReturnCode parse_NMEA_data(char *data, PhidgetGPSDeviceInfo *phid);
static void parse_GPSDevice_packets(PhidgetGPSDeviceInfo *phid);

//initAfterOpen - sets up the initial state of an object, reading in packets from the device if needed
//				  used during attach initialization - on every attach
static PhidgetReturnCode CCONV
PhidgetGPSDevice_initAfterOpen(PhidgetDeviceHandle device) {
	PhidgetGPSDeviceHandle phid = (PhidgetGPSDeviceHandle)device;
	uint8_t buffer[16];
	int readtries;

	assert(phid);

	phid->sckbuf_read = 0;
	phid->sckbuf_write = 0;
	memset(phid->sckbuf, 0, 256);

	phid->lastFix = PUNK_BOOL;
	phid->lastLatitude = PUNK_DBL;
	phid->lastLongitude = PUNK_DBL;
	phid->lastAltitude = PUNK_DBL;

	phid->positionFixState[0] = PUNK_BOOL;
	phid->heading[0] = PUNK_DBL;
	phid->velocity[0] = PUNK_DBL;
	phid->altitude[0] = PUNK_DBL;
	phid->latitude[0] = PUNK_DBL;
	phid->longitude[0] = PUNK_DBL;

	phid->timeValid[0] = PUNK_BOOL;
	phid->dateValid[0] = PUNK_BOOL;
	phid->NMEADataValid[0] = PFALSE;
	phid->lastTimeValid = PFALSE;
	phid->lastDateValid = PFALSE;

	// clean out the NMEA sentences
	memset(&phid->NMEAData[0], 0, sizeof(PhidgetGPS_NMEAData));

	// Query software version
	//buffer[0] = 2; // 2 byte payload
	//buffer[1] = GPS_SKYTRAQ_IN_QUERY_SOFTWARE_VERSION;
	//buffer[2] = 0x01; // System code
	//PhidgetDevice_sendpacket(NULL, device, buffer, buffer[0] + 1);

	// Enable WAAS
	buffer[0] = 3; // 3 byte payload
	buffer[1] = GPS_SKYTRAQ_IN_CONFIGURE_WAAS;
	buffer[2] = 0x01; // enable
	buffer[3] = 0x00; // update to SRAM
	PhidgetDevice_sendpacket(NULL, device, buffer, buffer[0] + 1);

	// Query WAAS
	//buffer[0] = 1; // 1 byte payload
	//buffer[1] = GPS_SKYTRAQ_IN_QUERY_WAAS;
	//PhidgetDevice_sendpacket(NULL, device, buffer, buffer[0] + 1);

	//read some initial data - rate is 10Hz so we shouldn't have to wait long
	//This ensures we have received at least one GGA and one RMC
	readtries = 30; //250ms
	while ((phid->positionFixState[0] == PUNK_BOOL || phid->timeValid[0] == PUNK_BOOL || phid->dateValid[0] == PUNK_BOOL) && readtries) {
		waitForReads(device, 1, 100);
		readtries--;
	}
	if (phid->positionFixState[0] == PUNK_BOOL)
		phid->positionFixState[0] = PFALSE;
	if (phid->timeValid[0] == PUNK_BOOL)
		phid->timeValid[0] = PFALSE;
	if (phid->dateValid[0] == PUNK_BOOL)
		phid->dateValid[0] = PFALSE;

	return (EPHIDGET_OK);
}

//dataInput - parses device packets
static PhidgetReturnCode CCONV
PhidgetGPSDevice_dataInput(PhidgetDeviceHandle device, uint8_t *buffer, size_t length) {
	PhidgetGPSDeviceHandle phid = (PhidgetGPSDeviceHandle)device;
	int i;

	assert(phid);
	assert(buffer);

	/* stick it in a buffer */
	for (i = 0; i < buffer[0]; i++)
		phid->sckbuf[phid->sckbuf_write++] = buffer[i + 1];

	parse_GPSDevice_packets(phid);

	return (EPHIDGET_OK);
}

/* checks a CRC */
static BOOL
checkcrc(char *data, int crc) {
	uint8_t check;
	uint32_t i;

	check = 0;
	for (i = 1; i < strlen(data); i++)
		check ^= data[i];
	if (check == crc)
		return PFALSE;
	return PTRUE;
}

// Date algorithms from http://howardhinnant.github.io/date_algorithms.html#days_from_civil
static int
days_from_civil(PhidgetGPS_Date date) {
	date.tm_year -= date.tm_mon <= 2;
	int era = (date.tm_year >= 0 ? date.tm_year : date.tm_year - 399) / 400;
	unsigned yoe = date.tm_year - era * 400;      // [0, 399]
	unsigned doy = (153 * (date.tm_mon + (date.tm_mon > 2 ? -3 : 9)) + 2) / 5 + date.tm_mday - 1;  // [0, 365]
	unsigned doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;         // [0, 146096]
	return era * 146097 + doe - 719468;
}

static PhidgetGPS_Date
civil_from_days(int z) {
	PhidgetGPS_Date date;
	z += 719468;
	int era = (z >= 0 ? z : z - 146096) / 146097;
	unsigned doe = (z - era * 146097);          // [0, 146096]
	unsigned yoe = (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365;  // [0, 399]
	int y = (yoe) + era * 400;
	unsigned doy = doe - (365 * yoe + yoe / 4 - yoe / 100);                // [0, 365]
	unsigned mp = (5 * doy + 2) / 153;                                   // [0, 11]
	unsigned d = doy - (153 * mp + 2) / 5 + 1;                             // [1, 31]
	unsigned m = mp + (mp < 10 ? 3 : -9);                            // [1, 12]

	date.tm_mon = m;
	date.tm_mday = d;
	date.tm_year = y + (m <= 2);

	return date;
}

/* this parses a full NMEA sentence */
static PhidgetReturnCode
parse_NMEA_data(char *data, PhidgetGPSDeviceInfo *phid) {
	PhidgetChannelHandle channel;
	PhidgetReturnCode res;
	PhidgetGPS_Date date;
	char *dataarray[50];
	BridgePacket *bp;
	double dintpart;
	double decpart;
	int numfields;
	double tempD;
	int intpart;
	int days;
	int crc;
	int i;
	int j;
#if 0
	int sentenceNumber;
	int numSentences;
	int numSats;
#endif

	/* fist check CRC if there is one */
	j = (int)strlen(data);
	for (i = 0; i < j; i++) {
		if (data[i] == '*') {
			crc = (int)strtol(&data[i + 1], NULL, 16);
			data[i] = '\0';
			if (checkcrc(data, crc)) {
				logwarn("CRC Error parsing NMEA sentence.");
				return (EPHIDGET_INVALID);
			}
			goto crcGood;
		}
	}

	// Unexpected packet with no CRC
	return (EPHIDGET_INVALID);

crcGood:
	/* seperate out by commas */
	numfields = 0;
	memset(dataarray, 0, sizeof(dataarray));
	dataarray[0] = data;
	j = (int)strlen(data);
	for (i = 0; i < j; i++) {
		if (data[i] == ',') {
			numfields++;
			dataarray[numfields] = data + i + 1;
			data[i] = '\0';
		}
	}
	numfields++;

	if (strlen(dataarray[0]) != 6) {
		logwarn("Bad sentence type.");
		return (EPHIDGET_UNEXPECTED);
	}

	/* find the type of sentence */
	if (!strncmp("GGA", dataarray[0] + 3, 3)) {

		if (numfields < 12) {
			logwarn("Bad GGA sentence");
			return EPHIDGET_INVALID;
		}

		//time: HHMMSS.milliseconds
		if (strlen(dataarray[1]) < 6) {
			phid->timeValid[0] = PFALSE;
		} else {
			decpart = modf(strtod(dataarray[1], NULL), &dintpart);
			intpart = (int)dintpart;
			phid->time[0].tm_hour = (int16_t)(intpart / 10000);
			phid->time[0].tm_min = (int16_t)(intpart / 100 % 100);
			phid->time[0].tm_sec = (int16_t)(intpart % 100);
			phid->time[0].tm_ms = (int16_t)round(decpart * 1000);
			phid->timeValid[0] = PTRUE;
		}

		/* convert lat/long to signed decimal degree format */
		if (dataarray[2] && strlen(dataarray[2])) {
			tempD = (int)(strtol(dataarray[2], NULL, 10) / 100) + (strtod((dataarray[2] + 2), NULL) / 60);
			if (dataarray[3] && dataarray[3][0] == 'S')
				phid->NMEAData[0].GGA.latitude = -tempD;
			else
				phid->NMEAData[0].GGA.latitude = tempD;
		} else
			phid->NMEAData[0].GGA.latitude = 0;

		if (dataarray[4] && strlen(dataarray[4])) {
			tempD = (int)(strtol(dataarray[4], NULL, 10) / 100) + (strtod((dataarray[4] + 3), NULL) / 60);
			if (dataarray[5][0] == 'W')
				phid->NMEAData[0].GGA.longitude = -tempD;
			else
				phid->NMEAData[0].GGA.longitude = tempD;
		} else
			phid->NMEAData[0].GGA.longitude = 0;

		phid->NMEAData[0].GGA.fixQuality = (int16_t)strtol(dataarray[6], NULL, 10);
		phid->NMEAData[0].GGA.numSatellites = (int16_t)strtol(dataarray[7], NULL, 10);
		phid->NMEAData[0].GGA.horizontalDilution = strtod(dataarray[8], NULL);

		phid->NMEAData[0].GGA.altitude = strtod(dataarray[9], NULL);
		phid->NMEAData[0].GGA.heightOfGeoid = strtod(dataarray[11], NULL);

		//Set local variables for getters/events
		phid->positionFixState[0] = (phid->NMEAData[0].GGA.fixQuality == 0) ? PFALSE : PTRUE;
		if (phid->positionFixState[0]) {
			phid->altitude[0] = phid->NMEAData[0].GGA.altitude;
			phid->latitude[0] = phid->NMEAData[0].GGA.latitude;
			phid->longitude[0] = phid->NMEAData[0].GGA.longitude;
		} else {
			phid->altitude[0] = PUNK_DBL;
			phid->latitude[0] = PUNK_DBL;
			phid->longitude[0] = PUNK_DBL;
		}

		if ((channel = getChannel(phid, 0)) != NULL) {
			//Time
			if (phid->lastTimeValid != phid->timeValid[0] || phid->timeValid[0] == PTRUE) {
				res = createBridgePacket(&bp, BP_TIME, "%d", phid->timeValid[0]);
				if (res == EPHIDGET_OK) {
					if (phid->timeValid[0])
						writeGPSTime(&phid->time[0], bp);
					bridgeSendBPToChannel(channel, bp); // destroys bp
				}
			}
			phid->lastTimeValid = phid->timeValid[0];

			//Fix status changed
			if (phid->positionFixState[0] != phid->lastFix) {
				bridgeSendToChannel(channel, BP_POSITIONFIXSTATUSCHANGE, "%d", phid->positionFixState[0]);
				phid->lastFix = phid->positionFixState[0];
			}

			/* only sends event if the fix is valid, and position has changed a bit */
			if (phid->positionFixState[0] == PTRUE
				&& (phid->latitude[0] != phid->lastLatitude || phid->longitude[0] != phid->lastLongitude || phid->altitude[0] != phid->lastAltitude)
				) {
				bridgeSendToChannel(channel, BP_POSITIONCHANGE, "%g%g%g", phid->latitude[0], phid->longitude[0], phid->altitude[0]);
				phid->lastLatitude = phid->latitude[0];
				phid->lastLongitude = phid->longitude[0];
				phid->lastAltitude = phid->altitude[0];
			}
			PhidgetRelease(&channel);
		}
	} else if (!strncmp("GSA", dataarray[0] + 3, 3)) {
		if (numfields < 18) {
			logwarn("Bad GSA sentence");
			return EPHIDGET_INVALID;
		}
		phid->NMEAData[0].GSA.mode = dataarray[1][0];
		phid->NMEAData[0].GSA.fixType = (int16_t)strtol(dataarray[2], NULL, 10);
		for (i = 0; i < 12; i++)
			phid->NMEAData[0].GSA.satUsed[i] = (int16_t)strtol(dataarray[i + 3], NULL, 10);
		phid->NMEAData[0].GSA.posnDilution = strtod(dataarray[15], NULL);
		phid->NMEAData[0].GSA.horizDilution = strtod(dataarray[16], NULL);
		phid->NMEAData[0].GSA.vertDilution = strtod(dataarray[17], NULL);
	} else if (!strncmp("GSV", dataarray[0] + 3, 3)) {
#if 0 // GSV Decoding disabled
		numSentences = strtol(dataarray[1], NULL, 10);
		sentenceNumber = strtol(dataarray[2], NULL, 10);
		numSats = strtol(dataarray[3], NULL, 10);
		phid->GPSData[0].GSV.satsInView = (int16_t)numSats;
		for (i = 0; i < (numSentences == sentenceNumber ? numSats - (4 * (numSentences - 1)) : 4); i++) {
			phid->GPSData[0].GSV.satInfo[i + ((sentenceNumber - 1) * 4)].ID = (int16_t)strtol(dataarray[4 + (i * 4)], NULL, 10);
			phid->GPSData[0].GSV.satInfo[i + ((sentenceNumber - 1) * 4)].elevation = (int16_t)strtol(dataarray[5 + (i * 4)], NULL, 10);
			phid->GPSData[0].GSV.satInfo[i + ((sentenceNumber - 1) * 4)].azimuth = strtol(dataarray[6 + (i * 4)], NULL, 10);
			phid->GPSData[0].GSV.satInfo[i + ((sentenceNumber - 1) * 4)].SNR = (int16_t)strtol(dataarray[7 + (i * 4)], NULL, 10);
		}
#endif
	} else if (!strncmp("RMC", dataarray[0] + 3, 3)) {
		if (numfields < 13) {
			logwarn("Bad RMC sentence");
			return EPHIDGET_INVALID;
		}

		phid->NMEAData[0].RMC.status = dataarray[2][0];

		/* convert lat/long to signed decimal degree format */
		if (strlen(dataarray[3])) {
			tempD = (int)(strtol(dataarray[3], NULL, 10) / 100) + (strtod((dataarray[3] + 2), NULL) / 60);
			if (dataarray[4][0] == 'S')
				phid->NMEAData[0].RMC.latitude = -tempD;
			else
				phid->NMEAData[0].RMC.latitude = tempD;
		} else
			phid->NMEAData[0].RMC.latitude = 0;

		if (strlen(dataarray[5])) {
			tempD = (int)(strtol(dataarray[5], NULL, 10) / 100) + (strtod((dataarray[5] + 3), NULL) / 60);
			if (dataarray[6][0] == 'W')
				phid->NMEAData[0].RMC.longitude = -tempD;
			else
				phid->NMEAData[0].RMC.longitude = tempD;
		} else
			phid->NMEAData[0].RMC.longitude = 0;

		phid->NMEAData[0].RMC.speedKnots = strtod(dataarray[7], NULL);
		phid->NMEAData[0].RMC.heading = strtod(dataarray[8], NULL);

		if (strlen(dataarray[9]) >= 6) {
			intpart = (int)strtol(dataarray[9], NULL, 10);

			date.tm_mday = (int16_t)(intpart / 10000);
			date.tm_mon = (int16_t)(intpart / 100 % 100);
			date.tm_year = (int16_t)(intpart % 100) + 2000; //2-digit year, add 2000 years

			// Assume the year is between 1980 and 2080
			if (date.tm_year > 2080)
				date.tm_year -= 100;

			days = days_from_civil(date);

			// NOTE: Added to code after the April 6 2019 week counter rollover
			//  Need to advance the date 1024 weeks
			days += (1024 * 7);

			phid->date[0] = civil_from_days(days);

			phid->dateValid[0] = PTRUE;
		} else
			phid->dateValid[0] = PFALSE;

		tempD = strtod(dataarray[10], NULL);
		if (dataarray[11][0] == 'W')
			phid->NMEAData[0].RMC.magneticVariation = -tempD;
		else
			phid->NMEAData[0].RMC.magneticVariation = tempD;

		phid->NMEAData[0].RMC.mode = dataarray[12][0];

		if (phid->NMEAData[0].RMC.status == 'A') {
			phid->velocity[0] = phid->NMEAData[0].RMC.speedKnots * 1.852; //convert to km/h
			phid->heading[0] = phid->NMEAData[0].RMC.heading;
		} else {
			phid->velocity[0] = PUNK_DBL;
			phid->heading[0] = PUNK_DBL;
		}

		if ((channel = getChannel(phid, 0)) != NULL) {

			//Date
			if (phid->lastDateValid != phid->dateValid[0] || phid->dateValid[0] == PTRUE) {
				res = createBridgePacket(&bp, BP_DATE, "%d", phid->dateValid[0]);
				if (res == EPHIDGET_OK) {
					if (phid->dateValid[0])
						writeGPSDate(&phid->date[0], bp);
					bridgeSendBPToChannel(channel, bp); // destroys bp
				}
			}
			phid->lastDateValid = phid->dateValid[0];

			if (phid->velocity[0] != PUNK_DBL && phid->heading[0] != PUNK_DBL)
				bridgeSendToChannel(channel, BP_HEADINGCHANGE, "%g%g", phid->heading[0], phid->velocity[0]);

			PhidgetRelease(&channel);
		}
	} else if (!strncmp("VTG", dataarray[0] + 3, 3)) {
		if (numfields < 10) {
			logwarn("Bad VTG sentence");
			return EPHIDGET_INVALID;
		}
		phid->NMEAData[0].VTG.trueHeading = strtod(dataarray[1], NULL);
		phid->NMEAData[0].VTG.magneticHeading = strtod(dataarray[3], NULL);
		phid->NMEAData[0].VTG.speedKnots = strtod(dataarray[5], NULL);
		phid->NMEAData[0].VTG.speed = strtod(dataarray[7], NULL);
		phid->NMEAData[0].VTG.mode = dataarray[9][0];
	} else {
		loginfo("Unrecognized sentence type: %s", dataarray[0] + 3);
		return (EPHIDGET_INVALID);
	}

	phid->NMEADataValid[0] = PTRUE;

	// Send NMEA data to channel
	if ((channel = getChannel(phid, 0)) != NULL) {
		res = createBridgePacket(&bp, BP_DATA, NULL);
		if (res == EPHIDGET_OK) {
			res = writeNMEAData(&phid->NMEAData[0], bp);
			if (res == EPHIDGET_OK)
				bridgeSendBPToChannel(channel, bp); // destroys bp
			else
				destroyBridgePacket(&bp);
		}
		PhidgetRelease(&channel);
	}

	return (EPHIDGET_OK);
}

static PhidgetReturnCode CCONV
PhidgetGPSDevice_bridgeInput(PhidgetChannelHandle ch, BridgePacket *bp) {

	assert(((PhidgetGPSDeviceHandle)ch->parent)->phid.deviceInfo.class == PHIDCLASS_GPS);
	assert(ch->class == PHIDCHCLASS_GPS);
	assert(ch->index < ((PhidgetGPSDeviceHandle)ch->parent)->devChannelCnts.numGPSes);

	switch (bp->vpkt) {
	case BP_OPENRESET:
	case BP_CLOSERESET:
	case BP_ENABLE:
		return (EPHIDGET_OK);
	default:
		MOS_PANIC("Unexpected packet type");
	}
}

/* this parses a full sentence */
static PhidgetReturnCode
parse_SkyTraq_response(uint8_t *data, PhidgetGPSDeviceInfo *phid) {
	int msgLength;
	int crccheck;
	int crc;
	int i;

	msgLength = data[3];
	crc = data[msgLength + 4];
	crccheck = 0;
	for (i = 0; i < msgLength; i++) {
		crccheck ^= data[i + 4];
	}
	if (crc != crccheck) {
		logwarn("CRC Error parsing SkyTraq response.");
		return (EPHIDGET_INVALID);
	}

	switch (data[4]) {
	case GPS_SKYTRAQ_OUT_ACK: //ACK
		loginfo("SkyTraq ACK: 0x%02x", data[5]);
		break;
	case GPS_SKYTRAQ_OUT_NACK: //NACK
		loginfo("SkyTraq NACK: 0x%02x", data[5]);
		break;
	case GPS_SKYTRAQ_OUT_SOFTWARE_VERSION:
		loginfo("SkyTraq Software Version: %*D", msgLength - 1, data + 5, " ");
		break;
	case GPS_SKYTRAQ_OUT_SOFTWARE_CRC:
		loginfo("SkyTraq Software CRC: %*D", msgLength - 1, data + 5, " ");
		break;
	case GPS_SKYTRAQ_OUT_POSITION_UPDATE_RATE:
		loginfo("SkyTraq Position Update Rate: %*D", msgLength - 1, data + 5, " ");
		break;
	default:
		loginfo("Got a SkyTraq message: %*D", msgLength, data + 4, " ");
	}

	return (EPHIDGET_OK);
}

/* this parses out the packets */
static void
parse_GPSDevice_packets(PhidgetGPSDeviceInfo *phid) {
	uint8_t current_queuesize;
	uint8_t tempbuffer[256] = { 0 };
	uint8_t msgsize;
	int keepLooking;
	int i;

	do {
		keepLooking = PFALSE;
		/* Not known if packetsize is valid yet... */

		/* advance read ptr to '$' or 0xa0 */
		for (i = 0; (i < 255) && (phid->sckbuf_read != phid->sckbuf_write); i++, phid->sckbuf_read++) {
			if (phid->sckbuf[phid->sckbuf_read] == '$' || phid->sckbuf[phid->sckbuf_read] == 0xa0)
				break;
		}

		current_queuesize = phid->sckbuf_write - phid->sckbuf_read;

		//response msg from skytraq - size is in posn 3
		if (phid->sckbuf[phid->sckbuf_read] == 0xa0) {
			if (current_queuesize < 4)
				break;
			msgsize = 7 + phid->sckbuf[(uint8_t)(phid->sckbuf_read + 3)];
			if (current_queuesize < msgsize)
				break;

			for (i = 0; i < msgsize; i++)
				tempbuffer[i] = phid->sckbuf[phid->sckbuf_read++];
			tempbuffer[i] = 0;

			/* We know that we have at least a full sentence here... look for another */
			keepLooking = PTRUE;

			/* here we'll actually parse this sentence */
			if (parse_SkyTraq_response(tempbuffer, phid) != EPHIDGET_OK)
				logwarn("Error parsing SkyTraq response.");
		} else {
			//NMEA/Other
			/* find the end of the sentence */
			msgsize = 0;
			for (i = 0; i < current_queuesize; i++) {
				if (phid->sckbuf[(uint8_t)(phid->sckbuf_read + i)] == '\n') {
					msgsize = i;
					break;
				}
			}
			if (msgsize == 0)
				break; //couldn't find it

			for (i = 0; i < msgsize; i++)
				tempbuffer[i] = phid->sckbuf[phid->sckbuf_read++];
			tempbuffer[i] = 0;

			/* We know that we have at least a full sentence here... look for another */
			keepLooking = PTRUE;

			//NMEA - always starts with '$GP'
			if (current_queuesize >= 6 && tempbuffer[1] == 'G' && tempbuffer[2] == 'P') {
				/* here we'll actually parse this sentence */
				if (parse_NMEA_data((char *)tempbuffer, phid) != EPHIDGET_OK)
					logwarn("Error parsing NMEA sentence.");
			} else {
				//Something else that starts with a '$'
				loginfo("GPSDevice Message: %s", tempbuffer);
			}
		}
	} while (keepLooking);
}

static void CCONV
PhidgetGPSDevice_free(PhidgetDeviceHandle *phid) {

	mos_free(*phid, sizeof(struct _PhidgetGPSDevice));
	*phid = NULL;
}

// === Exported Functions === //

//create and initialize a device structure
PhidgetReturnCode
PhidgetGPSDevice_create(PhidgetGPSDeviceHandle *phidp) {

	DEVICECREATE_BODY(GPSDevice, PHIDCLASS_GPS);
	return (EPHIDGET_OK);
}
