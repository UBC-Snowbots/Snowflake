/*
* This file is part of libphidget22
*
* Copyright 2016 Phidgets Inc <patrick@phidgets.com>
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
#include "analogsensor.h"
#include "util/voltageinputsupport.h"

const Phidget_UnitInfo Phidget_Units[] = {
	{ PHIDUNIT_NONE,			"none",				"" },
	{ PHIDUNIT_BOOLEAN,			"boolean",			"" },
	{ PHIDUNIT_PERCENT,			"percent",			"%" },
	{ PHIDUNIT_DECIBEL,			"decibel",			"dB" },
	{ PHIDUNIT_MILLIMETER,		"millimeter",		"mm" },
	{ PHIDUNIT_CENTIMETER,		"centimeter",		"cm" },
	{ PHIDUNIT_METER,			"meter",			"m" },
	{ PHIDUNIT_GRAM,			"gram",				"g" },
	{ PHIDUNIT_KILOGRAM,		"kilogram",			"kg" },
	{ PHIDUNIT_MILLIAMPERE,		"milliampere",		"mA" },
	{ PHIDUNIT_AMPERE,			"ampere",			"A" },
	{ PHIDUNIT_KILOPASCAL,		"kilopascal",		"kPa" },
	{ PHIDUNIT_VOLT,			"volt",				"V" },
	{ PHIDUNIT_DEGREE_CELCIUS,	"degree Celsius",	"\xC2\xB0""C" },
	{ PHIDUNIT_LUX,				"lux",				"lx" },
	{ PHIDUNIT_GAUSS,			"gauss",			"G" },
	{ PHIDUNIT_PH,				"pH",				"" },
	{ PHIDUNIT_WATT,			"watt",				"W" }
};

double
PhidgetAnalogSensor_getVoltageSensorValue(double voltage, PhidgetVoltageInput_SensorType sensorType, PhidgetVoltageInputSupportHandle voltageInputSupport) {

	if (voltage == PUNK_DBL)
		return (PUNK_DBL);

	switch (sensorType) {
	case SENSOR_TYPE_1114:
		return round_double(voltage / 0.02 - 50, 3); // Degrees Celcius
	case SENSOR_TYPE_1117:
		return round_double(voltage * 12 - 30, 3); // V
	case SENSOR_TYPE_1123:
		return round_double(voltage * 12 - 30, 3); // V
	case SENSOR_TYPE_1127:
		return round_double(voltage * 200, 2); // lux
	case SENSOR_TYPE_1130_PH:
		return round_double(voltage * 3.56 - 1.889, 4); // pH
	case SENSOR_TYPE_1130_ORP:
		return round_double((2.5 - voltage) / 1.037, 5); // ORP (V)
	case SENSOR_TYPE_1132:
		return round_double(voltage / 0.225, 4); // mA
	case SENSOR_TYPE_1133:
		return round_double(16.801 * log(voltage * 200) + 9.872, 4); // dB
	case SENSOR_TYPE_1135:
		return round_double((voltage - 2.5) / 0.0681, 3); // V
	case SENSOR_TYPE_1142:
		return round_double(voltage * 295.7554 + 33.67076, 2); // lux  NOTE: user should really calculate using calibration values
	case SENSOR_TYPE_1143:
		return round_double(exp(voltage * 4.77 - 0.56905), 4); // lux  NOTE: user should really calculate using calibration values
	case SENSOR_TYPE_3500:
		return round_double(voltage / 0.5, 4); // RMS Amps
	case SENSOR_TYPE_3501:
		return round_double(voltage / 0.2, 4); // RMS Amps
	case SENSOR_TYPE_3502:
		return round_double(voltage / 0.1, 4); // RMS Amps
	case SENSOR_TYPE_3503:
		return round_double(voltage / 0.05, 3); // RMS Amps
	case SENSOR_TYPE_3507:
		return round_double(voltage * 50, 3); // V AC
	case SENSOR_TYPE_3508:
		return round_double(voltage * 50, 3); // V AC
	case SENSOR_TYPE_3509:
		return round_double(voltage * 40, 3); // V DC
	case SENSOR_TYPE_3510:
		return round_double(voltage * 15, 4); // V DC
	case SENSOR_TYPE_3511:
		return round_double(voltage * 2, 4); // mA
	case SENSOR_TYPE_3512:
		return round_double(voltage * 20, 3); // mA
	case SENSOR_TYPE_3513:
		return round_double(voltage * 200, 2); // mA
	case SENSOR_TYPE_3514:
		return round_double(voltage * 1500, 1); // W  NOTE: User must determine offset
	case SENSOR_TYPE_3515:
		return round_double(voltage * 1500, 1); // W  NOTE: User must determine offset
	case SENSOR_TYPE_3516:
		return round_double(voltage * 250, 2); // W  NOTE: User must determine offset
	case SENSOR_TYPE_3517:
		return round_double(voltage * 250, 2); // W  NOTE: User must determine offset
	case SENSOR_TYPE_3518:
		return round_double(voltage * 110, 3); // W  NOTE: User must determine offset
	case SENSOR_TYPE_3519:
		return round_double(voltage * 330, 2); // W  NOTE: User must determine offset
	case SENSOR_TYPE_3584:
		return round_double(voltage * 10, 4); // DC Amps
	case SENSOR_TYPE_3585:
		return round_double(voltage * 20, 3); // DC Amps
	case SENSOR_TYPE_3586:
		return round_double(voltage * 50, 3); // DC Amps
	case SENSOR_TYPE_3587:
		return round_double(voltage * 20 - 50, 3); // DC Amps
	case SENSOR_TYPE_3588:
		return round_double(voltage * 40 - 100, 3); // DC Amps
	case SENSOR_TYPE_3589:
		return round_double(voltage * 100 - 250, 3); // DC Amps
	case SENSOR_TYPE_MOT2002_LOW:
		return PhidgetAnalogSensor_doMotionSensorCalculations(voltageInputSupport, 0.8);
	case SENSOR_TYPE_MOT2002_MED:
		return PhidgetAnalogSensor_doMotionSensorCalculations(voltageInputSupport, 0.4);
	case SENSOR_TYPE_MOT2002_HIGH:
		return PhidgetAnalogSensor_doMotionSensorCalculations(voltageInputSupport, 0.04);
	case SENSOR_TYPE_VCP4114:
		return round_double(((voltage - 2.5) / 0.0625), 3); // DC Amps
	case SENSOR_TYPE_VOLTAGE:
	default:
		return voltage;
	}
}

int
PhidgetAnalogSensor_getVoltageSensorValueInRange(double sensorValue, PhidgetVoltageInput_SensorType sensorType, PhidgetVoltageInputSupportHandle voltageInputSupport) {

	if (sensorValue == PUNK_DBL)
		return (PFALSE);

	switch (sensorType) {
	case SENSOR_TYPE_1114:
		return ((sensorValue >= -40.0) && (sensorValue <= 125.0)); // Degrees Celcius
	case SENSOR_TYPE_1117:
		return ((sensorValue >= -30.0) && (sensorValue <= 30.0)); // V
	case SENSOR_TYPE_1123:
		return ((sensorValue >= -30.0) && (sensorValue <= 30.0)); // V
	case SENSOR_TYPE_1127:
		return ((sensorValue >= 0.0) && (sensorValue <= 1000.0)); // lux
	case SENSOR_TYPE_1130_PH:
		return((sensorValue >= 0.0) && (sensorValue <= 14.0)); // pH
	case SENSOR_TYPE_1130_ORP:
		return ((sensorValue >= -2.0) && (sensorValue <= 2.0)); // ORP (V)
	case SENSOR_TYPE_1132:
		return ((sensorValue >= 4.0) && (sensorValue <= 20.0)); // mA
	case SENSOR_TYPE_1133:
		return ((sensorValue >= 50.0) && (sensorValue <= 100.0)); // dB
	case SENSOR_TYPE_1135:
		return ((sensorValue >= -30.0) && (sensorValue <= 30.0)); // V
	case SENSOR_TYPE_1142:
		return ((sensorValue >= 0.0) && (sensorValue <= 1000.0)); // lux  NOTE: user should really calculate using calibration values
	case SENSOR_TYPE_1143:
		return ((sensorValue >= 0.0) && (sensorValue <= 70000.0)); // lux  NOTE: user should really calculate using calibration values
	case SENSOR_TYPE_3500:
		return ((sensorValue >= 0.0) && (sensorValue <= 10.0)); // RMS Amps
	case SENSOR_TYPE_3501:
		return ((sensorValue >= 0.0) && (sensorValue <= 25.0)); // RMS Amps
	case SENSOR_TYPE_3502:
		return ((sensorValue >= 0.0) && (sensorValue <= 50.0)); // RMS Amps
	case SENSOR_TYPE_3503:
		return ((sensorValue >= 0.0) && (sensorValue <= 100.0)); // RMS Amps
	case SENSOR_TYPE_3507:
		return ((sensorValue >= 0.0) && (sensorValue <= 250.0)); // V AC
	case SENSOR_TYPE_3508:
		return ((sensorValue >= 0.0) && (sensorValue <= 250.0)); // V AC
	case SENSOR_TYPE_3509:
		return ((sensorValue >= 0.0) && (sensorValue <= 200.0)); // V DC
	case SENSOR_TYPE_3510:
		return ((sensorValue >= 0.0) && (sensorValue <= 75.0)); // V DC
	case SENSOR_TYPE_3511:
		return ((sensorValue >= 0.0) && (sensorValue <= 10.0)); // mA
	case SENSOR_TYPE_3512:
		return ((sensorValue >= 0.0) && (sensorValue <= 100.0)); // mA
	case SENSOR_TYPE_3513:
		return ((sensorValue >= 0.0) && (sensorValue <= 1000.0)); // mA
	case SENSOR_TYPE_3514:
		return ((sensorValue >= 0.0) && (sensorValue <= 7500.0)); // W  NOTE: User must determine offset
	case SENSOR_TYPE_3515:
		return ((sensorValue >= 0.0) && (sensorValue <= 7500.0)); // W  NOTE: User must determine offset
	case SENSOR_TYPE_3516:
		return ((sensorValue >= 0.0) && (sensorValue <= 1250.0)); // W  NOTE: User must determine offset
	case SENSOR_TYPE_3517:
		return ((sensorValue >= 0.0) && (sensorValue <= 1250.0)); // W  NOTE: User must determine offset
	case SENSOR_TYPE_3518:
		return ((sensorValue >= 0.0) && (sensorValue <= 550.0)); // W  NOTE: User must determine offset
	case SENSOR_TYPE_3519:
		return ((sensorValue >= 0.0) && (sensorValue <= 1650.0)); // W  NOTE: User must determine offset
	case SENSOR_TYPE_3584:
		return ((sensorValue >= 0.0) && (sensorValue <= 50.0)); // DC Amps
	case SENSOR_TYPE_3585:
		return ((sensorValue >= 0.0) && (sensorValue <= 100.0)); // DC Amps
	case SENSOR_TYPE_3586:
		return ((sensorValue >= 0.0) && (sensorValue <= 250.0)); // DC Amps
	case SENSOR_TYPE_3587:
		return ((sensorValue >= -50.0) && (sensorValue <= 50.0)); // DC Amps
	case SENSOR_TYPE_3588:
		return ((sensorValue >= -100.0) && (sensorValue <= 100.0)); // DC Amps
	case SENSOR_TYPE_3589:
		return ((sensorValue >= -250.0) && (sensorValue <= 250.0)); // DC Amps
	case SENSOR_TYPE_MOT2002_LOW:
	case SENSOR_TYPE_MOT2002_MED:
	case SENSOR_TYPE_MOT2002_HIGH:
		return (voltageInputSupport->motionSensorBaseline != PUNK_DBL);
	case SENSOR_TYPE_VCP4114:
		return ((sensorValue >= -25.0) && (sensorValue <= 25.0)); // DC Amps
	case SENSOR_TYPE_VOLTAGE:
	default:
		return (PTRUE);
	}
}

const Phidget_UnitInfo
PhidgetAnalogSensor_getVoltageSensorUnit(PhidgetVoltageInput_SensorType sensorType) {

	switch (sensorType) {
	case SENSOR_TYPE_1130_PH:
		return Phidget_Units[PHIDUNIT_PH];

	case SENSOR_TYPE_1114:
		return Phidget_Units[PHIDUNIT_DEGREE_CELCIUS];

	case SENSOR_TYPE_1127:
	case SENSOR_TYPE_1142:
	case SENSOR_TYPE_1143:
		return Phidget_Units[PHIDUNIT_LUX];

	case SENSOR_TYPE_1132:
	case SENSOR_TYPE_3511:
	case SENSOR_TYPE_3512:
	case SENSOR_TYPE_3513:
		return Phidget_Units[PHIDUNIT_MILLIAMPERE];

	case SENSOR_TYPE_1133:
		return Phidget_Units[PHIDUNIT_DECIBEL];

	case SENSOR_TYPE_3500:
	case SENSOR_TYPE_3501:
	case SENSOR_TYPE_3502:
	case SENSOR_TYPE_3503:
	case SENSOR_TYPE_3584:
	case SENSOR_TYPE_3585:
	case SENSOR_TYPE_3586:
	case SENSOR_TYPE_3587:
	case SENSOR_TYPE_3588:
	case SENSOR_TYPE_3589:
	case SENSOR_TYPE_VCP4114:
		return Phidget_Units[PHIDUNIT_AMPERE];

	case SENSOR_TYPE_3514:
	case SENSOR_TYPE_3515:
	case SENSOR_TYPE_3516:
	case SENSOR_TYPE_3517:
	case SENSOR_TYPE_3518:
	case SENSOR_TYPE_3519:
		return Phidget_Units[PHIDUNIT_WATT];

	case SENSOR_TYPE_MOT2002_LOW:
	case SENSOR_TYPE_MOT2002_MED:
	case SENSOR_TYPE_MOT2002_HIGH:
		return Phidget_Units[PHIDUNIT_NONE];

	case SENSOR_TYPE_1117:
	case SENSOR_TYPE_1123:
	case SENSOR_TYPE_1130_ORP:
	case SENSOR_TYPE_1135:
	case SENSOR_TYPE_3507:
	case SENSOR_TYPE_3508:
	case SENSOR_TYPE_3509:
	case SENSOR_TYPE_3510:
	case SENSOR_TYPE_VOLTAGE:
	default:
		return Phidget_Units[PHIDUNIT_VOLT];
	}
}

double
PhidgetAnalogSensor_getVoltageRatioSensorValue(double voltageRatio, PhidgetVoltageRatioInput_SensorType sensorType) {

	if (voltageRatio == PUNK_DBL)
		return (PUNK_DBL);

	switch (sensorType) {
	case SENSOR_TYPE_1101_SHARP_2D120X:
	case SENSOR_TYPE_3520:
		return round_double(2.076 / (voltageRatio - 0.011), 2); // cm
	case SENSOR_TYPE_1101_SHARP_2Y0A21:
	case SENSOR_TYPE_3521:
		return round_double(4.8 / (voltageRatio - 0.02), 2); // cm
	case SENSOR_TYPE_1101_SHARP_2Y0A02:
	case SENSOR_TYPE_3522:
		return round_double(9.462 / (voltageRatio - 0.01692), 2); // cm
	case SENSOR_TYPE_1102:
		return voltageRatio < 0.4; // true/false
	case SENSOR_TYPE_1103:
		return voltageRatio < 0.1; // true/false
	case SENSOR_TYPE_1104:
		return round_double(voltageRatio * 2 - 1, 5); // +- 1
	case SENSOR_TYPE_1105:
		return voltageRatio; // 0-1
	case SENSOR_TYPE_1106:
		return voltageRatio; // 0-1
	case SENSOR_TYPE_1107:
		return round_double((voltageRatio * 190.6) - 40.2, 3); // %RH
	case SENSOR_TYPE_1108:
		return round_double((0.5 - voltageRatio) * 1000, 2); // Gauss
	case SENSOR_TYPE_1109:
		return voltageRatio; // 0-1
	case SENSOR_TYPE_1110:
		return voltageRatio < 0.5; // true/false
	case SENSOR_TYPE_1111:
		return round_double(voltageRatio * 2 - 1, 5); // +- 1
	case SENSOR_TYPE_1112:
		return voltageRatio; // 0-1
	case SENSOR_TYPE_1113:
		return round_double(voltageRatio * 2 - 1, 5); // +- 1
	case SENSOR_TYPE_1115:
		return round_double(voltageRatio / 0.004 + 10, 3); // kPa
	case SENSOR_TYPE_1116:
		return voltageRatio; // 0-1
	case SENSOR_TYPE_1118_AC:
		return round_double(voltageRatio * 69.38, 3); // RMS Amps
	case SENSOR_TYPE_1118_DC:
		return round_double(voltageRatio / 0.008 - 62.5, 3); //DC Amps
	case SENSOR_TYPE_1119_AC:
		return round_double(voltageRatio * 27.75, 4); // RMS Amps
	case SENSOR_TYPE_1119_DC:
		return round_double(voltageRatio / 0.02 - 25, 4); //DC Amps
	case SENSOR_TYPE_1120:
		return voltageRatio; // 0-1
	case SENSOR_TYPE_1121:
		return voltageRatio; // 0-1
	case SENSOR_TYPE_1122_AC:
		return round_double(voltageRatio * 42.04, 3); // RMS Amps
	case SENSOR_TYPE_1122_DC:
		return round_double(voltageRatio / 0.0132 - 37.8787, 3); // DC Amps
	case SENSOR_TYPE_1124:
	case SENSOR_TYPE_1125_TEMPERATURE:
		return round_double(voltageRatio * 222.2 - 61.111, 3); // Degrees Celcius
	case SENSOR_TYPE_1125_HUMIDITY:
		return round_double(voltageRatio * 190.6 - 40.2, 3); // %RH
	case SENSOR_TYPE_1126:
		return round_double(voltageRatio / 0.018 - 27.7777, 3); // kPa
	case SENSOR_TYPE_1128:
		return round_double(voltageRatio * 1296, 2); // cm
	case SENSOR_TYPE_1129:
		return voltageRatio > 0.5; // true/false
	case SENSOR_TYPE_1131:
		return round_double(15.311 * exp(voltageRatio * 5.199), 2); // grams
	case SENSOR_TYPE_1134:
		return voltageRatio; // 0-1
	case SENSOR_TYPE_1136:
		return round_double(voltageRatio / 0.2 - 2.5, 4); // kPa
	case SENSOR_TYPE_1137:
		return round_double(voltageRatio / 0.057143 - 8.75, 4); // kPa
	case SENSOR_TYPE_1138:
		return round_double(voltageRatio / 0.018 - 2.222, 3); // kPa
	case SENSOR_TYPE_1139:
		return round_double(voltageRatio / 0.009 - 4.444, 3); // kPa
	case SENSOR_TYPE_1140:
		return round_double(voltageRatio / 0.002421 + 3.478, 2); // kPa
	case SENSOR_TYPE_1141:
		return round_double(voltageRatio / 0.0092 + 10.652, 2); // kPa
	case SENSOR_TYPE_1146:
		return round_double(1.3927 * exp(1.967 * voltageRatio), 2); // mm
	case SENSOR_TYPE_3120:
		return round_double(voltageRatio / 0.15432 - 0.647989, 4); // kg
	case SENSOR_TYPE_3121:
		return round_double(voltageRatio / 0.0617295 - 1.619971, 4); // kg
	case SENSOR_TYPE_3122:
		return round_double(voltageRatio / 0.0308647 - 3.239943, 3); // kg
	case SENSOR_TYPE_3123:
		return round_double(voltageRatio / 0.0154324 - 6.479886, 3); // kg
	case SENSOR_TYPE_3130:
		return round_double(voltageRatio * 190.6 - 40.2, 3); // %RH

	case SENSOR_TYPE_VOLTAGERATIO:
	default:
		return voltageRatio;
	}
}

int
PhidgetAnalogSensor_getVoltageRatioSensorValueInRange(double sensorValue, PhidgetVoltageRatioInput_SensorType sensorType) {

	if (sensorValue == PUNK_DBL)
		return (PFALSE);

	switch (sensorType) {
	case SENSOR_TYPE_1101_SHARP_2D120X:
	case SENSOR_TYPE_3520:
		return ((sensorValue >= 4.0) && (sensorValue <= 30.0)); // cm
	case SENSOR_TYPE_1101_SHARP_2Y0A21:
	case SENSOR_TYPE_3521:
		return ((sensorValue >= 10.0) && (sensorValue <= 80.0)); // cm
	case SENSOR_TYPE_1101_SHARP_2Y0A02:
	case SENSOR_TYPE_3522:
		return ((sensorValue >= 20.0) && (sensorValue <= 150.0)); // cm
	case SENSOR_TYPE_1102:
		return (PTRUE); // true/false
	case SENSOR_TYPE_1103:
		return (PTRUE); // true/false
	case SENSOR_TYPE_1104:
		return (PTRUE); // +- 1
	case SENSOR_TYPE_1105:
		return (PTRUE); // 0-1
	case SENSOR_TYPE_1106:
		return (PTRUE); // 0-1
	case SENSOR_TYPE_1107:
		return ((sensorValue >= 0.0) && (sensorValue <= 100.0)); // %RH
	case SENSOR_TYPE_1108:
		return ((sensorValue >= -500.0) && (sensorValue <= 500.0)); // Gauss
	case SENSOR_TYPE_1109:
		return (PTRUE); // 0-1
	case SENSOR_TYPE_1110:
		return (PTRUE); // true/false
	case SENSOR_TYPE_1111:
		return (PTRUE); // +- 1
	case SENSOR_TYPE_1112:
		return (PTRUE); // 0-1
	case SENSOR_TYPE_1113:
		return ((sensorValue >= -1.0) && (sensorValue <= 1.0)); // +- 1
	case SENSOR_TYPE_1115:
		return ((sensorValue >= 20.0) && (sensorValue <= 250.0)); // kPa
	case SENSOR_TYPE_1116:
		return (PTRUE); // 0-1
	case SENSOR_TYPE_1118_AC:
		return ((sensorValue >= 0.0) && (sensorValue <= 50.0)); // RMS Amps
	case SENSOR_TYPE_1118_DC:
		return ((sensorValue >= -50.0) && (sensorValue <= 50.0)); //DC Amps
	case SENSOR_TYPE_1119_AC:
		return ((sensorValue >= 0.0) && (sensorValue <= 20.0)); // RMS Amps
	case SENSOR_TYPE_1119_DC:
		return ((sensorValue >= -20.0) && (sensorValue <= 20.0)); //DC Amps
	case SENSOR_TYPE_1120:
		return (PTRUE); // 0-1
	case SENSOR_TYPE_1121:
		return (PTRUE); // 0-1
	case SENSOR_TYPE_1122_AC:
		return ((sensorValue >= 0.0) && (sensorValue <= 30.0)); // RMS Amps
	case SENSOR_TYPE_1122_DC:
		return ((sensorValue >= -30.0) && (sensorValue <= 30.0)); // DC Amps
	case SENSOR_TYPE_1124:
	case SENSOR_TYPE_1125_TEMPERATURE:
		return ((sensorValue >= -50.0) && (sensorValue <= 150.0)); // Degrees Celcius
	case SENSOR_TYPE_1125_HUMIDITY:
		return ((sensorValue >= 0.0) && (sensorValue <= 100.0)); // %RH
	case SENSOR_TYPE_1126:
		return ((sensorValue >= -25.0) && (sensorValue <= 25.0)); // kPa
	case SENSOR_TYPE_1128:
		return ((sensorValue >= 15.24) && (sensorValue <= 6500.0)); // cm
	case SENSOR_TYPE_1129:
		return (PTRUE); // true/false
	case SENSOR_TYPE_1131:
		return ((sensorValue >= 0.0) && (sensorValue <= 2000.0)); // grams
	case SENSOR_TYPE_1134:
		return (PTRUE); // 0-1
	case SENSOR_TYPE_1136:
		return ((sensorValue >= -2.0) && (sensorValue <= 2.0)); // kPa
	case SENSOR_TYPE_1137:
		return ((sensorValue >= -7.0) && (sensorValue <= 7.0)); // kPa
	case SENSOR_TYPE_1138:
		return ((sensorValue >= 0.0) && (sensorValue <= 50.0)); // kPa
	case SENSOR_TYPE_1139:
		return ((sensorValue >= 0.0) && (sensorValue <= 100.0)); // kPa
	case SENSOR_TYPE_1140:
		return ((sensorValue >= 20.0) && (sensorValue <= 400.0)); // kPa
	case SENSOR_TYPE_1141:
		return ((sensorValue >= 15.0) && (sensorValue <= 115.0)); // kPa
	case SENSOR_TYPE_1146:
		return ((sensorValue >= 1.5) && (sensorValue <= 4.0)); // mm
	case SENSOR_TYPE_3120:
		return ((sensorValue >= 0.0) && (sensorValue <= 4.5)); // kg
	case SENSOR_TYPE_3121:
		return ((sensorValue >= 0.0) && (sensorValue <= 11.3)); // kg
	case SENSOR_TYPE_3122:
		return ((sensorValue >= 0.0) && (sensorValue <= 22.7)); // kg
	case SENSOR_TYPE_3123:
		return ((sensorValue >= 0.0) && (sensorValue <= 45.3)); // kg
	case SENSOR_TYPE_3130:
		return ((sensorValue >= 0.0) && (sensorValue <= 100.0)); // %RH

	case SENSOR_TYPE_VOLTAGERATIO:
		return (PTRUE);
	default:
		return (PTRUE);
	}
}

const Phidget_UnitInfo
PhidgetAnalogSensor_getVoltageRatioSensorUnit(PhidgetVoltageRatioInput_SensorType sensorType) {

	switch (sensorType) {
	case SENSOR_TYPE_1146:
		return Phidget_Units[PHIDUNIT_MILLIMETER];

	case SENSOR_TYPE_1101_SHARP_2D120X:
	case SENSOR_TYPE_1101_SHARP_2Y0A21:
	case SENSOR_TYPE_1101_SHARP_2Y0A02:
	case SENSOR_TYPE_1128:
	case SENSOR_TYPE_3520:
	case SENSOR_TYPE_3521:
	case SENSOR_TYPE_3522:
		return Phidget_Units[PHIDUNIT_CENTIMETER];

	case SENSOR_TYPE_1102:
	case SENSOR_TYPE_1103:
	case SENSOR_TYPE_1110:
	case SENSOR_TYPE_1129:
		return Phidget_Units[PHIDUNIT_BOOLEAN];

	case SENSOR_TYPE_1107:
	case SENSOR_TYPE_1125_HUMIDITY:
	case SENSOR_TYPE_3130:
		return Phidget_Units[PHIDUNIT_PERCENT];

	case SENSOR_TYPE_1108:
		return Phidget_Units[PHIDUNIT_GAUSS];

	case SENSOR_TYPE_1115:
	case SENSOR_TYPE_1126:
	case SENSOR_TYPE_1136:
	case SENSOR_TYPE_1137:
	case SENSOR_TYPE_1138:
	case SENSOR_TYPE_1139:
	case SENSOR_TYPE_1140:
	case SENSOR_TYPE_1141:
		return Phidget_Units[PHIDUNIT_KILOPASCAL];

	case SENSOR_TYPE_1118_AC:
	case SENSOR_TYPE_1119_AC:
	case SENSOR_TYPE_1122_AC:
	case SENSOR_TYPE_1118_DC:
	case SENSOR_TYPE_1119_DC:
	case SENSOR_TYPE_1122_DC:
		return Phidget_Units[PHIDUNIT_AMPERE];

	case SENSOR_TYPE_1124:
	case SENSOR_TYPE_1125_TEMPERATURE:
		return Phidget_Units[PHIDUNIT_DEGREE_CELCIUS];

	case SENSOR_TYPE_1131:
		return Phidget_Units[PHIDUNIT_GRAM];

	case SENSOR_TYPE_3120:
	case SENSOR_TYPE_3121:
	case SENSOR_TYPE_3122:
	case SENSOR_TYPE_3123:
		return Phidget_Units[PHIDUNIT_KILOGRAM];

	case SENSOR_TYPE_VOLTAGERATIO:
	case SENSOR_TYPE_1104:
	case SENSOR_TYPE_1111:
	case SENSOR_TYPE_1113:
	case SENSOR_TYPE_1105:
	case SENSOR_TYPE_1106:
	case SENSOR_TYPE_1109:
	case SENSOR_TYPE_1112:
	case SENSOR_TYPE_1116:
	case SENSOR_TYPE_1120:
	case SENSOR_TYPE_1121:
	case SENSOR_TYPE_1134:
	default:
		return Phidget_Units[PHIDUNIT_NONE];
	}
}

int PhidgetAnalogSensor_doMotionSensorCalculations(PhidgetVoltageInputSupportHandle voltageInputSupport, double threshold) {

	double* voltageBuffer = voltageInputSupport->voltageBuffer;
	int index = voltageInputSupport->voltageBufferIndex;
	double startAvgDiff = 0;
	double endAvgDiff = 0;
	double longTermAvg = 0;
	double longTermDiff = 0;
	int triggered = 0;
	int i = 0;

	if (voltageInputSupport->voltageBufferReady) {

		for (i = 0; i < VOLTAGE_BUFFER_LEN; i++) {
			longTermAvg += voltageBuffer[i];
		}
		longTermAvg /= VOLTAGE_BUFFER_LEN;

		for (i = 0; i < VOLTAGE_BUFFER_LEN; i++) {
			longTermDiff += fabs(voltageBuffer[i] - longTermAvg);
		}
		longTermDiff /= VOLTAGE_BUFFER_LEN;

		if (longTermDiff < 0.1) {
			voltageInputSupport->motionSensorBaseline = longTermAvg;
		} else if (voltageInputSupport->motionSensorBaseline == PUNK_DBL) {
			return PUNK_BOOL;
		}

		for (i = 0; i < 5; i++) {
			startAvgDiff += fabs(voltageBuffer[((index + VOLTAGE_BUFFER_LEN) - (i+5)) % VOLTAGE_BUFFER_LEN] - voltageInputSupport->motionSensorBaseline);
			endAvgDiff += fabs(voltageBuffer[((index + VOLTAGE_BUFFER_LEN) - i) % VOLTAGE_BUFFER_LEN] - voltageInputSupport->motionSensorBaseline);
		}
		startAvgDiff /= 5;
		endAvgDiff /= 5;

		if (voltageInputSupport->motionSensorCountdown != 0) {
			voltageInputSupport->motionSensorCountdown--;
			triggered = 1;
		}

		if (startAvgDiff > threshold && endAvgDiff > threshold) {
			voltageInputSupport->motionSensorCountdown = 10;
			triggered = 1;
		}

		return triggered;
	}

	return PUNK_BOOL;


}
