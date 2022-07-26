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
#include "util/voltageinputsupport.h"

// Access the PhidgetVoltageInputSupport struct via the channel private pointer
#define VOLTAGEINPUT_SUPPORT(ch) ((PhidgetVoltageInputSupportHandle)(((PhidgetChannelHandle)(ch))->private))

void
PhidgetVoltageInputSupport_free(PhidgetVoltageInputSupportHandle *arg) {

	if (arg == NULL || *arg == NULL)
		return;

	mos_free(*arg, sizeof(PhidgetVoltageInputSupport));
	*arg = NULL;
}

PhidgetReturnCode
PhidgetVoltageInputSupport_create(PhidgetVoltageInputSupportHandle *arg) {

	TESTPTR_PR(arg);
	*arg = mos_zalloc(sizeof(PhidgetVoltageInputSupport));

	return (EPHIDGET_OK);
}

void
PhidgetVoltageInputSupport_init(PhidgetVoltageInputSupportHandle arg) {

	assert(arg);

	arg->voltageBufferIndex = 0;
	arg->voltageBufferReady = 0;
	arg->motionSensorCountdown = 0;
	arg->motionSensorBaseline = PUNK_DBL;
}

// Access the PhidgetIRSupport struct via the channel private pointer
#define VOLTAGEINPUT_SUPPORT(ch) ((PhidgetVoltageInputSupportHandle)(((PhidgetChannelHandle)(ch))->private))

void PhidgetVoltageInputSupport_updateVoltageBuffer(PhidgetVoltageInputSupportHandle voltageInputSupport, double voltage) {

	voltageInputSupport->voltageBuffer[voltageInputSupport->voltageBufferIndex] = voltage;

	voltageInputSupport->voltageBufferIndex++;
	voltageInputSupport->voltageBufferIndex %= VOLTAGE_BUFFER_LEN;
	if (voltageInputSupport->voltageBufferIndex == 0)
		voltageInputSupport->voltageBufferReady = 1;
}