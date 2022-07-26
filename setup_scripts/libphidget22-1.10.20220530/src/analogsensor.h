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

#ifndef _ANALOGSENSOR_H_
#define _ANALOGSENSOR_H_

#include "util/voltageinputsupport.h"

extern const Phidget_UnitInfo Phidget_Units[];

double PhidgetAnalogSensor_getVoltageSensorValue(double voltage, PhidgetVoltageInput_SensorType sensorType, PhidgetVoltageInputSupportHandle voltageInputSupport);
int PhidgetAnalogSensor_getVoltageSensorValueInRange(double sensorValue, PhidgetVoltageInput_SensorType sensorType, PhidgetVoltageInputSupportHandle voltageInputSupport);
const Phidget_UnitInfo PhidgetAnalogSensor_getVoltageSensorUnit(PhidgetVoltageInput_SensorType sensorType);
double PhidgetAnalogSensor_getVoltageRatioSensorValue(double voltageRatio, PhidgetVoltageRatioInput_SensorType sensorType);
int PhidgetAnalogSensor_getVoltageRatioSensorValueInRange(double sensorValue, PhidgetVoltageRatioInput_SensorType sensorType);
const Phidget_UnitInfo PhidgetAnalogSensor_getVoltageRatioSensorUnit(PhidgetVoltageRatioInput_SensorType sensorType);
int PhidgetAnalogSensor_doMotionSensorCalculations(PhidgetVoltageInputSupportHandle voltageInputSupport, double threshold);

#endif
