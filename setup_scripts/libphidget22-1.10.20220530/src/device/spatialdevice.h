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

#ifndef __CPHIDGETSPATIALDEVICE
#define __CPHIDGETSPATIALDEVICE

typedef struct _PhidgetSpatialDevice *PhidgetSpatialDeviceHandle;
PhidgetReturnCode PhidgetSpatialDevice_create(PhidgetSpatialDeviceHandle *phid);

/**
 * Timestamped position data returned by the \ref PhidgetSpatialDevice_setOnSpatialDeviceDataHandler event.
 */
typedef struct _PhidgetSpatialDevice_SpatialDeviceEventData {
	double acceleration[3]; /* Acceleration data for up to 3 axes. */
	double angularRate[3]; /* Angular rate data (Gyroscope) for up to 3 axes */
	double magneticField[3]; /* Magnetic field data (Compass) for up to 3 axes */
	double quaternion[4];
	double timestamp; /* Hardware timestamp in ms */
	double temperature; //optional temperature return for sensors that allow it
} PhidgetSpatialDevice_SpatialDeviceEventData, *PhidgetSpatialDevice_SpatialDeviceEventDataHandle;

#define SPATIAL_MAX_ACCELEROMETERS 1
#define SPATIAL_MAX_GYROS 1
#define SPATIAL_MAX_COMPASSES 1

#define SPATIAL_MAX_ACCELAXES 3
#define SPATIAL_MAX_GYROAXES 3
#define SPATIAL_MAX_COMPASSAXES 3

//in milliseconds - this is the fastest hardware rate of any device
#define SPATIAL_MAX_DATA_RATE 1
//1 second is the longest between events that we support
#define SPATIAL_MIN_DATA_RATE 1000
//add 200ms for timing differences (late events, etc) - should be plenty
#define SPATIAL_DATA_BUFFER_SIZE ((SPATIAL_MIN_DATA_RATE * 2 + 200)/SPATIAL_MAX_DATA_RATE)
//2 seconds of data to zero the gyro - make sure DATA_BUFFER_SIZE is big enough to hold this much data
#define SPATIAL_ZERO_GYRO_TIME 2000

//packet types
//IN
#define SPATIAL_PACKET_DATA		0x00
#define SPATIAL_PACKET_CALIB	0x80
//OUT
#define SPATIAL_READCALIB			0x01
#define SPATIAL_SET_POLLING_TYPE	0x02
#define SPATIAL_ZERO_GYRO			0x03
#define SPATIAL_UNZERO_GYRO			0x04
#define SPATIAL_ZERO_AHRS			0x05
#define SPATIAL_CONFIGURE_AHRS		0x06
#define SPATIAL_RESET_AHRS			0x07
#define MAGNETOMETER_RESET_CORRECTION_PARAMETERS		0x08
#define MAGNETOMETER_SAVE_CORRECTION_PARAMETERS			0x09
#define MAGNETOMETER_SET_CORRECTION_PARAMETERS			0x10
#define SPATIAL_RESET				0x11
#define SPATIAL_HEATING				0x12
#define SPATIAL_AHRS_PARAMS			0x13

//M3 SpatialDevice calibration table indexes
#define SPATIAL_LOW_ACCEL_CALIB_TABLE_INDEX			0
#define SPATIAL_HIGH_ACCEL_CALIB_TABLE_INDEX		1
#define SPATIAL_LOW_GYRO_CALIB_TABLE_INDEX			2
#define SPATIAL_HIGH_GYRO_CALIB_TABLE_INDEX			3
#define SPATIAL_COMPASS_CALIB_TABLE_INDEX			4
#define SPATIAL_COMPASS_TEMP_CALIB_TABLE_INDEX		5

#define SPATIAL_ACCEL_GYRO_CALIB_TABLE_LENGTH		64
#define SPATIAL_COMPASS_CALIB_TABLE_LENGTH			56

#define SPATIAL_GyroCalibTable_ID 		1000
#define SPATIAL_AccelCalibTable_ID 		1001

// Flags
#define SPATIAL_GYRO_LOW			0x01
#define SPATIAL_ACCEL_LOW			0x02

/**
 * M3 SpatialDevice constants (1041, 1042, 1043, 1044)
 *  -Values are transmitted as signed integers with a unit of 'bits'
 *  -Values are calibrated in-firmware
 *  -Values are centered at zero
 *
 * Precision Voltage Ref is 3.3 V
 * AD7689 ADC is 16-bit
 *
 * LPR410AL x,y Analog Gyro is 2.5 mV/dps and +-400 dps
 * LY330ALH z-axis Analog Gyro is 3.752 mV/dps and +-300 dps
 * L3GD20 Digital Gyro is 70 mdps/bit and +-2000 dps
 *
 * KXR94-2050 Analog Accelerometer is 660 mV/g and +-2g
 * MMA8451Q Digital Accelerometer is 1024 bits/g and +-8g
 *
 * HMC5883L Digital Compass is 3.03 mG/bit and +-5.6 Gauss
 **/
#define SPATIAL_VOLTAGE_REF 3.3
#define SPATIAL_AD7689_BITS_PER_VOLT (0x10000 / SPATIAL_VOLTAGE_REF)

#define SPATIAL_LPR410AL_VOLTS_PER_DPS 0.0025
#define SPATIAL_LRP410AL_w_AD7689_BITS_PER_DPS (SPATIAL_LPR410AL_VOLTS_PER_DPS * SPATIAL_AD7689_BITS_PER_VOLT)

#define SPATIAL_LY330ALH_VOLTS_PER_DPS 0.003752
#define SPATIAL_LY330ALH_w_AD7689_BITS_PER_DPS (SPATIAL_LY330ALH_VOLTS_PER_DPS * SPATIAL_AD7689_BITS_PER_VOLT)

#define SPATIAL_L3GD20_DPS_PER_BIT 0.07
#define SPATIAL_L3GD20_BITS_PER_DPS (1 / SPATIAL_L3GD20_DPS_PER_BIT)

#define SPATIAL_KXR94_2050_VOLTS_PER_G 0.660
#define SPATIAL_KXR94_2050_w_AD7689_BITS_PER_G (SPATIAL_KXR94_2050_VOLTS_PER_G * SPATIAL_AD7689_BITS_PER_VOLT)

#define SPATIAL_MMA8451Q_BITS_PER_G 1024.0

#define SPATIAL_HMC5883L_GAUSS_PER_BIT 0.00303
#define SPATIAL_HMC5883L_BITS_PER_GAUSS (1 / SPATIAL_HMC5883L_GAUSS_PER_BIT)

#define SPATIAL_ADXRS810_BITS_PER_DPS 80.0
#define SPATIAL_ADXRS290_BITS_PER_DPS 200.0
#define SPATIAL_LIS3DHH_G_PER_BIT 0.000076
#define SPATIAL_LIS3DHH_BITS_PER_G (1 / SPATIAL_LIS3DHH_G_PER_BIT)
#define SPATIAL_LIS2MDL_GAUSS_PER_BIT 0.0015
#define SPATIAL_LIS2MDL_BITS_PER_GAUSS (1 / SPATIAL_LIS2MDL_GAUSS_PER_BIT)
#define SPATIAL_LIS2HH12_G_PER_BIT 0.000244
#define SPATIAL_LIS2HH12_BITS_PER_G (1 / SPATIAL_LIS2HH12_G_PER_BIT)

struct _PhidgetSpatialDevice {
#undef devChannelCnts
#define devChannelCnts	phid.deviceInfo.UDD->channelCnts.spatial
	PhidgetDevice phid;

	/* Public Members */

	int dataInterval[1];
	int temperatureDataInterval;
	double timestamp[1];

	double acceleration[SPATIAL_MAX_ACCELEROMETERS][SPATIAL_MAX_ACCELAXES];
	double accelerationChangeTrigger[SPATIAL_MAX_ACCELEROMETERS];

	double angularRate[SPATIAL_MAX_GYROS][SPATIAL_MAX_GYROAXES];

	double magneticField[SPATIAL_MAX_COMPASSES][SPATIAL_MAX_COMPASSAXES];
	double magneticFieldChangeTrigger[SPATIAL_MAX_COMPASSES];

	double quaternion[4];

	double temperature[1];
	double temperatureChangeTrigger[1];

	/* Private Members */

	double gryoCorrection[SPATIAL_MAX_GYROAXES];
	uint8_t doZeroGyro;
	int gyroZeroReadPtr;

	PhidgetSpatialDevice_SpatialDeviceEventData dataBuffer[SPATIAL_DATA_BUFFER_SIZE];
	int bufferReadPtr, bufferWritePtr;

	double accelerationMax, accelerationMin;
	double angularRateMax, angularRateMin;
	double magneticFieldMax, magneticFieldMin;
	double temperatureMax, temperatureMin;

	uint8_t calDataValid;

	double accelGain1[SPATIAL_MAX_ACCELAXES];
	double accelGain2[SPATIAL_MAX_ACCELAXES];
	double accelOffset[SPATIAL_MAX_ACCELAXES];
	double accelFactor1[SPATIAL_MAX_ACCELAXES];
	double accelFactor2[SPATIAL_MAX_ACCELAXES];

	double gyroGain1[SPATIAL_MAX_GYROAXES];
	double gyroGain2[SPATIAL_MAX_GYROAXES];
	double gyroOffset[SPATIAL_MAX_GYROAXES];
	double gyroFactor1[SPATIAL_MAX_GYROAXES];
	double gyroFactor2[SPATIAL_MAX_GYROAXES];

	int compassGain[SPATIAL_MAX_COMPASSAXES];
	int compassOffset[SPATIAL_MAX_COMPASSAXES];

	double userMagField;
	double userCompassGain[SPATIAL_MAX_COMPASSAXES];
	double userCompassOffset[SPATIAL_MAX_COMPASSAXES];
	double userCompassTransform[SPATIAL_MAX_COMPASSAXES*(SPATIAL_MAX_COMPASSAXES - 1)];

	double lastEventTime, latestDataTime;
	double lastTemperatureEventTime;

	uint8_t lastTimeCounterValid;
	int lastTimeCounter;
	int flip;

	char *compassCorrectionParamsString;
	uint8_t spatialDataNetwork;

	int interruptRate;
	int dataRateMax, dataRateMin;

	double accelAxisLastTrigger[SPATIAL_MAX_ACCELAXES];
	double magAxisLastTrigger[SPATIAL_MAX_COMPASSAXES];
	double temperatureLastTrigger[1];

	Phidget_SpatialAlgorithm algorithm;
	double AHRSMagGain;

} typedef PhidgetSpatialDeviceInfo;

#endif
