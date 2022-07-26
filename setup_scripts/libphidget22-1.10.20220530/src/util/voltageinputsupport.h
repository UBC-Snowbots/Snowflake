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

#ifndef __VOLTAGEINPUTSUPPORT
#define __VOLTAGEINPUTSUPPORT

#define VOLTAGE_BUFFER_LEN 25

typedef struct {

	/* Public Members */
	double voltageBuffer[VOLTAGE_BUFFER_LEN];
	int voltageBufferIndex;
	int voltageBufferReady;
	int motionSensorCountdown;
	double motionSensorBaseline;
	/* Private Members */

} PhidgetVoltageInputSupport, *PhidgetVoltageInputSupportHandle;

void PhidgetVoltageInputSupport_free(PhidgetVoltageInputSupportHandle *arg);
PhidgetReturnCode PhidgetVoltageInputSupport_create(PhidgetVoltageInputSupportHandle *arg);
void PhidgetVoltageInputSupport_init(PhidgetVoltageInputSupportHandle arg);

void PhidgetVoltageInputSupport_updateVoltageBuffer(PhidgetVoltageInputSupportHandle voltageInputSupport, double voltage);

#endif
