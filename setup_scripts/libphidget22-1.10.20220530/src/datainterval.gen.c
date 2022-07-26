#include "phidgetbase.h"
#include "phidget.h"
#include "mos/mos_os.h"
#include "class/voltageratioinput.gen.h"
#include "class/rcservo.gen.h"
#include "class/accelerometer.gen.h"
#include "class/voltageinput.gen.h"
#include "class/capacitivetouch.gen.h"
#include "class/gyroscope.gen.h"
#include "class/magnetometer.gen.h"
#include "class/spatial.gen.h"
#include "class/temperaturesensor.gen.h"
#include "class/encoder.gen.h"
#include "class/frequencycounter.gen.h"
#include "class/phsensor.gen.h"
#include "class/dcmotor.gen.h"
#include "class/currentinput.gen.h"
#include "class/stepper.gen.h"
#include "class/motorpositioncontroller.gen.h"
#include "class/bldcmotor.gen.h"
#include "class/distancesensor.gen.h"
#include "class/humiditysensor.gen.h"
#include "class/lightsensor.gen.h"
#include "class/pressuresensor.gen.h"
#include "class/resistanceinput.gen.h"
#include "class/soundsensor.gen.h"

API_PRETURN
Phidget_getDataInterval(PhidgetHandle ch, uint32_t *di) {
	Phidget_ChannelClass cclass;
	PhidgetReturnCode res;

	res = Phidget_getChannelClass(ch, &cclass);
	if (res != EPHIDGET_OK)
		return (PHID_RETURN(res));

	switch (cclass) {
	case PHIDCHCLASS_VOLTAGERATIOINPUT:
		return (PhidgetVoltageRatioInput_getDataInterval((PhidgetVoltageRatioInputHandle)ch, di));
	case PHIDCHCLASS_RCSERVO:
		return (PhidgetRCServo_getDataInterval((PhidgetRCServoHandle)ch, di));
	case PHIDCHCLASS_ACCELEROMETER:
		return (PhidgetAccelerometer_getDataInterval((PhidgetAccelerometerHandle)ch, di));
	case PHIDCHCLASS_VOLTAGEINPUT:
		return (PhidgetVoltageInput_getDataInterval((PhidgetVoltageInputHandle)ch, di));
	case PHIDCHCLASS_CAPACITIVETOUCH:
		return (PhidgetCapacitiveTouch_getDataInterval((PhidgetCapacitiveTouchHandle)ch, di));
	case PHIDCHCLASS_GYROSCOPE:
		return (PhidgetGyroscope_getDataInterval((PhidgetGyroscopeHandle)ch, di));
	case PHIDCHCLASS_MAGNETOMETER:
		return (PhidgetMagnetometer_getDataInterval((PhidgetMagnetometerHandle)ch, di));
	case PHIDCHCLASS_SPATIAL:
		return (PhidgetSpatial_getDataInterval((PhidgetSpatialHandle)ch, di));
	case PHIDCHCLASS_TEMPERATURESENSOR:
		return (PhidgetTemperatureSensor_getDataInterval((PhidgetTemperatureSensorHandle)ch, di));
	case PHIDCHCLASS_ENCODER:
		return (PhidgetEncoder_getDataInterval((PhidgetEncoderHandle)ch, di));
	case PHIDCHCLASS_FREQUENCYCOUNTER:
		return (PhidgetFrequencyCounter_getDataInterval((PhidgetFrequencyCounterHandle)ch, di));
	case PHIDCHCLASS_PHSENSOR:
		return (PhidgetPHSensor_getDataInterval((PhidgetPHSensorHandle)ch, di));
	case PHIDCHCLASS_DCMOTOR:
		return (PhidgetDCMotor_getDataInterval((PhidgetDCMotorHandle)ch, di));
	case PHIDCHCLASS_CURRENTINPUT:
		return (PhidgetCurrentInput_getDataInterval((PhidgetCurrentInputHandle)ch, di));
	case PHIDCHCLASS_STEPPER:
		return (PhidgetStepper_getDataInterval((PhidgetStepperHandle)ch, di));
	case PHIDCHCLASS_MOTORPOSITIONCONTROLLER:
		return (PhidgetMotorPositionController_getDataInterval((PhidgetMotorPositionControllerHandle)ch,
		  di));
	case PHIDCHCLASS_BLDCMOTOR:
		return (PhidgetBLDCMotor_getDataInterval((PhidgetBLDCMotorHandle)ch, di));
	case PHIDCHCLASS_DISTANCESENSOR:
		return (PhidgetDistanceSensor_getDataInterval((PhidgetDistanceSensorHandle)ch, di));
	case PHIDCHCLASS_HUMIDITYSENSOR:
		return (PhidgetHumiditySensor_getDataInterval((PhidgetHumiditySensorHandle)ch, di));
	case PHIDCHCLASS_LIGHTSENSOR:
		return (PhidgetLightSensor_getDataInterval((PhidgetLightSensorHandle)ch, di));
	case PHIDCHCLASS_PRESSURESENSOR:
		return (PhidgetPressureSensor_getDataInterval((PhidgetPressureSensorHandle)ch, di));
	case PHIDCHCLASS_RESISTANCEINPUT:
		return (PhidgetResistanceInput_getDataInterval((PhidgetResistanceInputHandle)ch, di));
	case PHIDCHCLASS_SOUNDSENSOR:
		return (PhidgetSoundSensor_getDataInterval((PhidgetSoundSensorHandle)ch, di));
	default:
		return (PHID_RETURN_ERRSTR(EPHIDGET_UNSUPPORTED,
		  "Data interval is not supported by this channel class."));
	}
}

API_PRETURN
Phidget_getDataRate(PhidgetHandle ch, double *dr) {
	Phidget_ChannelClass cclass;
	PhidgetReturnCode res;

	res = Phidget_getChannelClass(ch, &cclass);
	if (res != EPHIDGET_OK)
		return (PHID_RETURN(res));

	switch (cclass) {
	case PHIDCHCLASS_VOLTAGERATIOINPUT:
		return (PhidgetVoltageRatioInput_getDataRate((PhidgetVoltageRatioInputHandle)ch, dr));
	case PHIDCHCLASS_RCSERVO:
		return (PhidgetRCServo_getDataRate((PhidgetRCServoHandle)ch, dr));
	case PHIDCHCLASS_ACCELEROMETER:
		return (PhidgetAccelerometer_getDataRate((PhidgetAccelerometerHandle)ch, dr));
	case PHIDCHCLASS_VOLTAGEINPUT:
		return (PhidgetVoltageInput_getDataRate((PhidgetVoltageInputHandle)ch, dr));
	case PHIDCHCLASS_CAPACITIVETOUCH:
		return (PhidgetCapacitiveTouch_getDataRate((PhidgetCapacitiveTouchHandle)ch, dr));
	case PHIDCHCLASS_GYROSCOPE:
		return (PhidgetGyroscope_getDataRate((PhidgetGyroscopeHandle)ch, dr));
	case PHIDCHCLASS_MAGNETOMETER:
		return (PhidgetMagnetometer_getDataRate((PhidgetMagnetometerHandle)ch, dr));
	case PHIDCHCLASS_SPATIAL:
		return (PhidgetSpatial_getDataRate((PhidgetSpatialHandle)ch, dr));
	case PHIDCHCLASS_TEMPERATURESENSOR:
		return (PhidgetTemperatureSensor_getDataRate((PhidgetTemperatureSensorHandle)ch, dr));
	case PHIDCHCLASS_ENCODER:
		return (PhidgetEncoder_getDataRate((PhidgetEncoderHandle)ch, dr));
	case PHIDCHCLASS_FREQUENCYCOUNTER:
		return (PhidgetFrequencyCounter_getDataRate((PhidgetFrequencyCounterHandle)ch, dr));
	case PHIDCHCLASS_PHSENSOR:
		return (PhidgetPHSensor_getDataRate((PhidgetPHSensorHandle)ch, dr));
	case PHIDCHCLASS_DCMOTOR:
		return (PhidgetDCMotor_getDataRate((PhidgetDCMotorHandle)ch, dr));
	case PHIDCHCLASS_CURRENTINPUT:
		return (PhidgetCurrentInput_getDataRate((PhidgetCurrentInputHandle)ch, dr));
	case PHIDCHCLASS_STEPPER:
		return (PhidgetStepper_getDataRate((PhidgetStepperHandle)ch, dr));
	case PHIDCHCLASS_MOTORPOSITIONCONTROLLER:
		return (PhidgetMotorPositionController_getDataRate((PhidgetMotorPositionControllerHandle)ch,
		  dr));
	case PHIDCHCLASS_BLDCMOTOR:
		return (PhidgetBLDCMotor_getDataRate((PhidgetBLDCMotorHandle)ch, dr));
	case PHIDCHCLASS_DISTANCESENSOR:
		return (PhidgetDistanceSensor_getDataRate((PhidgetDistanceSensorHandle)ch, dr));
	case PHIDCHCLASS_HUMIDITYSENSOR:
		return (PhidgetHumiditySensor_getDataRate((PhidgetHumiditySensorHandle)ch, dr));
	case PHIDCHCLASS_LIGHTSENSOR:
		return (PhidgetLightSensor_getDataRate((PhidgetLightSensorHandle)ch, dr));
	case PHIDCHCLASS_PRESSURESENSOR:
		return (PhidgetPressureSensor_getDataRate((PhidgetPressureSensorHandle)ch, dr));
	case PHIDCHCLASS_RESISTANCEINPUT:
		return (PhidgetResistanceInput_getDataRate((PhidgetResistanceInputHandle)ch, dr));
	case PHIDCHCLASS_SOUNDSENSOR:
		return (PhidgetSoundSensor_getDataRate((PhidgetSoundSensorHandle)ch, dr));
	default:
		return (PHID_RETURN_ERRSTR(EPHIDGET_UNSUPPORTED,
		  "Data rate is not supported by this channel class."));
	}
}

API_PRETURN
Phidget_getMinDataInterval(PhidgetHandle ch, uint32_t *di) {
	Phidget_ChannelClass cclass;
	PhidgetReturnCode res;

	res = Phidget_getChannelClass(ch, &cclass);
	if (res != EPHIDGET_OK)
		return (PHID_RETURN(res));

	switch (cclass) {
	case PHIDCHCLASS_VOLTAGERATIOINPUT:
		return (PhidgetVoltageRatioInput_getMinDataInterval((PhidgetVoltageRatioInputHandle)ch, di));
	case PHIDCHCLASS_RCSERVO:
		return (PhidgetRCServo_getMinDataInterval((PhidgetRCServoHandle)ch, di));
	case PHIDCHCLASS_ACCELEROMETER:
		return (PhidgetAccelerometer_getMinDataInterval((PhidgetAccelerometerHandle)ch, di));
	case PHIDCHCLASS_VOLTAGEINPUT:
		return (PhidgetVoltageInput_getMinDataInterval((PhidgetVoltageInputHandle)ch, di));
	case PHIDCHCLASS_CAPACITIVETOUCH:
		return (PhidgetCapacitiveTouch_getMinDataInterval((PhidgetCapacitiveTouchHandle)ch, di));
	case PHIDCHCLASS_GYROSCOPE:
		return (PhidgetGyroscope_getMinDataInterval((PhidgetGyroscopeHandle)ch, di));
	case PHIDCHCLASS_MAGNETOMETER:
		return (PhidgetMagnetometer_getMinDataInterval((PhidgetMagnetometerHandle)ch, di));
	case PHIDCHCLASS_SPATIAL:
		return (PhidgetSpatial_getMinDataInterval((PhidgetSpatialHandle)ch, di));
	case PHIDCHCLASS_TEMPERATURESENSOR:
		return (PhidgetTemperatureSensor_getMinDataInterval((PhidgetTemperatureSensorHandle)ch, di));
	case PHIDCHCLASS_ENCODER:
		return (PhidgetEncoder_getMinDataInterval((PhidgetEncoderHandle)ch, di));
	case PHIDCHCLASS_FREQUENCYCOUNTER:
		return (PhidgetFrequencyCounter_getMinDataInterval((PhidgetFrequencyCounterHandle)ch, di));
	case PHIDCHCLASS_PHSENSOR:
		return (PhidgetPHSensor_getMinDataInterval((PhidgetPHSensorHandle)ch, di));
	case PHIDCHCLASS_DCMOTOR:
		return (PhidgetDCMotor_getMinDataInterval((PhidgetDCMotorHandle)ch, di));
	case PHIDCHCLASS_CURRENTINPUT:
		return (PhidgetCurrentInput_getMinDataInterval((PhidgetCurrentInputHandle)ch, di));
	case PHIDCHCLASS_STEPPER:
		return (PhidgetStepper_getMinDataInterval((PhidgetStepperHandle)ch, di));
	case PHIDCHCLASS_MOTORPOSITIONCONTROLLER:
		return (PhidgetMotorPositionController_getMinDataInterval((PhidgetMotorPositionControllerHandle)ch, di));
	case PHIDCHCLASS_BLDCMOTOR:
		return (PhidgetBLDCMotor_getMinDataInterval((PhidgetBLDCMotorHandle)ch, di));
	case PHIDCHCLASS_DISTANCESENSOR:
		return (PhidgetDistanceSensor_getMinDataInterval((PhidgetDistanceSensorHandle)ch, di));
	case PHIDCHCLASS_HUMIDITYSENSOR:
		return (PhidgetHumiditySensor_getMinDataInterval((PhidgetHumiditySensorHandle)ch, di));
	case PHIDCHCLASS_LIGHTSENSOR:
		return (PhidgetLightSensor_getMinDataInterval((PhidgetLightSensorHandle)ch, di));
	case PHIDCHCLASS_PRESSURESENSOR:
		return (PhidgetPressureSensor_getMinDataInterval((PhidgetPressureSensorHandle)ch, di));
	case PHIDCHCLASS_RESISTANCEINPUT:
		return (PhidgetResistanceInput_getMinDataInterval((PhidgetResistanceInputHandle)ch, di));
	case PHIDCHCLASS_SOUNDSENSOR:
		return (PhidgetSoundSensor_getMinDataInterval((PhidgetSoundSensorHandle)ch, di));
	default:
		return (PHID_RETURN_ERRSTR(EPHIDGET_UNSUPPORTED,
		  "Data interval is not supported by this channel class."));
	}
}

API_PRETURN
Phidget_getMinDataRate(PhidgetHandle ch, double *dr) {
	Phidget_ChannelClass cclass;
	PhidgetReturnCode res;

	res = Phidget_getChannelClass(ch, &cclass);
	if (res != EPHIDGET_OK)
		return (PHID_RETURN(res));

	switch (cclass) {
	case PHIDCHCLASS_VOLTAGERATIOINPUT:
		return (PhidgetVoltageRatioInput_getMinDataRate((PhidgetVoltageRatioInputHandle)ch, dr));
	case PHIDCHCLASS_RCSERVO:
		return (PhidgetRCServo_getMinDataRate((PhidgetRCServoHandle)ch, dr));
	case PHIDCHCLASS_ACCELEROMETER:
		return (PhidgetAccelerometer_getMinDataRate((PhidgetAccelerometerHandle)ch, dr));
	case PHIDCHCLASS_VOLTAGEINPUT:
		return (PhidgetVoltageInput_getMinDataRate((PhidgetVoltageInputHandle)ch, dr));
	case PHIDCHCLASS_CAPACITIVETOUCH:
		return (PhidgetCapacitiveTouch_getMinDataRate((PhidgetCapacitiveTouchHandle)ch, dr));
	case PHIDCHCLASS_GYROSCOPE:
		return (PhidgetGyroscope_getMinDataRate((PhidgetGyroscopeHandle)ch, dr));
	case PHIDCHCLASS_MAGNETOMETER:
		return (PhidgetMagnetometer_getMinDataRate((PhidgetMagnetometerHandle)ch, dr));
	case PHIDCHCLASS_SPATIAL:
		return (PhidgetSpatial_getMinDataRate((PhidgetSpatialHandle)ch, dr));
	case PHIDCHCLASS_TEMPERATURESENSOR:
		return (PhidgetTemperatureSensor_getMinDataRate((PhidgetTemperatureSensorHandle)ch, dr));
	case PHIDCHCLASS_ENCODER:
		return (PhidgetEncoder_getMinDataRate((PhidgetEncoderHandle)ch, dr));
	case PHIDCHCLASS_FREQUENCYCOUNTER:
		return (PhidgetFrequencyCounter_getMinDataRate((PhidgetFrequencyCounterHandle)ch, dr));
	case PHIDCHCLASS_PHSENSOR:
		return (PhidgetPHSensor_getMinDataRate((PhidgetPHSensorHandle)ch, dr));
	case PHIDCHCLASS_DCMOTOR:
		return (PhidgetDCMotor_getMinDataRate((PhidgetDCMotorHandle)ch, dr));
	case PHIDCHCLASS_CURRENTINPUT:
		return (PhidgetCurrentInput_getMinDataRate((PhidgetCurrentInputHandle)ch, dr));
	case PHIDCHCLASS_STEPPER:
		return (PhidgetStepper_getMinDataRate((PhidgetStepperHandle)ch, dr));
	case PHIDCHCLASS_MOTORPOSITIONCONTROLLER:
		return (PhidgetMotorPositionController_getMinDataRate((PhidgetMotorPositionControllerHandle)ch,
		  dr));
	case PHIDCHCLASS_BLDCMOTOR:
		return (PhidgetBLDCMotor_getMinDataRate((PhidgetBLDCMotorHandle)ch, dr));
	case PHIDCHCLASS_DISTANCESENSOR:
		return (PhidgetDistanceSensor_getMinDataRate((PhidgetDistanceSensorHandle)ch, dr));
	case PHIDCHCLASS_HUMIDITYSENSOR:
		return (PhidgetHumiditySensor_getMinDataRate((PhidgetHumiditySensorHandle)ch, dr));
	case PHIDCHCLASS_LIGHTSENSOR:
		return (PhidgetLightSensor_getMinDataRate((PhidgetLightSensorHandle)ch, dr));
	case PHIDCHCLASS_PRESSURESENSOR:
		return (PhidgetPressureSensor_getMinDataRate((PhidgetPressureSensorHandle)ch, dr));
	case PHIDCHCLASS_RESISTANCEINPUT:
		return (PhidgetResistanceInput_getMinDataRate((PhidgetResistanceInputHandle)ch, dr));
	case PHIDCHCLASS_SOUNDSENSOR:
		return (PhidgetSoundSensor_getMinDataRate((PhidgetSoundSensorHandle)ch, dr));
	default:
		return (PHID_RETURN_ERRSTR(EPHIDGET_UNSUPPORTED,
		  "Data rate is not supported by this channel class."));
	}
}

API_PRETURN
Phidget_getMaxDataInterval(PhidgetHandle ch, uint32_t *di) {
	Phidget_ChannelClass cclass;
	PhidgetReturnCode res;

	res = Phidget_getChannelClass(ch, &cclass);
	if (res != EPHIDGET_OK)
		return (PHID_RETURN(res));

	switch (cclass) {
	case PHIDCHCLASS_VOLTAGERATIOINPUT:
		return (PhidgetVoltageRatioInput_getMaxDataInterval((PhidgetVoltageRatioInputHandle)ch, di));
	case PHIDCHCLASS_RCSERVO:
		return (PhidgetRCServo_getMaxDataInterval((PhidgetRCServoHandle)ch, di));
	case PHIDCHCLASS_ACCELEROMETER:
		return (PhidgetAccelerometer_getMaxDataInterval((PhidgetAccelerometerHandle)ch, di));
	case PHIDCHCLASS_VOLTAGEINPUT:
		return (PhidgetVoltageInput_getMaxDataInterval((PhidgetVoltageInputHandle)ch, di));
	case PHIDCHCLASS_CAPACITIVETOUCH:
		return (PhidgetCapacitiveTouch_getMaxDataInterval((PhidgetCapacitiveTouchHandle)ch, di));
	case PHIDCHCLASS_GYROSCOPE:
		return (PhidgetGyroscope_getMaxDataInterval((PhidgetGyroscopeHandle)ch, di));
	case PHIDCHCLASS_MAGNETOMETER:
		return (PhidgetMagnetometer_getMaxDataInterval((PhidgetMagnetometerHandle)ch, di));
	case PHIDCHCLASS_SPATIAL:
		return (PhidgetSpatial_getMaxDataInterval((PhidgetSpatialHandle)ch, di));
	case PHIDCHCLASS_TEMPERATURESENSOR:
		return (PhidgetTemperatureSensor_getMaxDataInterval((PhidgetTemperatureSensorHandle)ch, di));
	case PHIDCHCLASS_ENCODER:
		return (PhidgetEncoder_getMaxDataInterval((PhidgetEncoderHandle)ch, di));
	case PHIDCHCLASS_FREQUENCYCOUNTER:
		return (PhidgetFrequencyCounter_getMaxDataInterval((PhidgetFrequencyCounterHandle)ch, di));
	case PHIDCHCLASS_PHSENSOR:
		return (PhidgetPHSensor_getMaxDataInterval((PhidgetPHSensorHandle)ch, di));
	case PHIDCHCLASS_DCMOTOR:
		return (PhidgetDCMotor_getMaxDataInterval((PhidgetDCMotorHandle)ch, di));
	case PHIDCHCLASS_CURRENTINPUT:
		return (PhidgetCurrentInput_getMaxDataInterval((PhidgetCurrentInputHandle)ch, di));
	case PHIDCHCLASS_STEPPER:
		return (PhidgetStepper_getMaxDataInterval((PhidgetStepperHandle)ch, di));
	case PHIDCHCLASS_MOTORPOSITIONCONTROLLER:
		return (PhidgetMotorPositionController_getMaxDataInterval((PhidgetMotorPositionControllerHandle)ch, di));
	case PHIDCHCLASS_BLDCMOTOR:
		return (PhidgetBLDCMotor_getMaxDataInterval((PhidgetBLDCMotorHandle)ch, di));
	case PHIDCHCLASS_DISTANCESENSOR:
		return (PhidgetDistanceSensor_getMaxDataInterval((PhidgetDistanceSensorHandle)ch, di));
	case PHIDCHCLASS_HUMIDITYSENSOR:
		return (PhidgetHumiditySensor_getMaxDataInterval((PhidgetHumiditySensorHandle)ch, di));
	case PHIDCHCLASS_LIGHTSENSOR:
		return (PhidgetLightSensor_getMaxDataInterval((PhidgetLightSensorHandle)ch, di));
	case PHIDCHCLASS_PRESSURESENSOR:
		return (PhidgetPressureSensor_getMaxDataInterval((PhidgetPressureSensorHandle)ch, di));
	case PHIDCHCLASS_RESISTANCEINPUT:
		return (PhidgetResistanceInput_getMaxDataInterval((PhidgetResistanceInputHandle)ch, di));
	case PHIDCHCLASS_SOUNDSENSOR:
		return (PhidgetSoundSensor_getMaxDataInterval((PhidgetSoundSensorHandle)ch, di));
	default:
		return (PHID_RETURN_ERRSTR(EPHIDGET_UNSUPPORTED,
		  "Data interval is not supported by this channel class."));
	}
}

API_PRETURN
Phidget_getMaxDataRate(PhidgetHandle ch, double *dr) {
	Phidget_ChannelClass cclass;
	PhidgetReturnCode res;

	res = Phidget_getChannelClass(ch, &cclass);
	if (res != EPHIDGET_OK)
		return (PHID_RETURN(res));

	switch (cclass) {
	case PHIDCHCLASS_VOLTAGERATIOINPUT:
		return (PhidgetVoltageRatioInput_getMaxDataRate((PhidgetVoltageRatioInputHandle)ch, dr));
	case PHIDCHCLASS_RCSERVO:
		return (PhidgetRCServo_getMaxDataRate((PhidgetRCServoHandle)ch, dr));
	case PHIDCHCLASS_ACCELEROMETER:
		return (PhidgetAccelerometer_getMaxDataRate((PhidgetAccelerometerHandle)ch, dr));
	case PHIDCHCLASS_VOLTAGEINPUT:
		return (PhidgetVoltageInput_getMaxDataRate((PhidgetVoltageInputHandle)ch, dr));
	case PHIDCHCLASS_CAPACITIVETOUCH:
		return (PhidgetCapacitiveTouch_getMaxDataRate((PhidgetCapacitiveTouchHandle)ch, dr));
	case PHIDCHCLASS_GYROSCOPE:
		return (PhidgetGyroscope_getMaxDataRate((PhidgetGyroscopeHandle)ch, dr));
	case PHIDCHCLASS_MAGNETOMETER:
		return (PhidgetMagnetometer_getMaxDataRate((PhidgetMagnetometerHandle)ch, dr));
	case PHIDCHCLASS_SPATIAL:
		return (PhidgetSpatial_getMaxDataRate((PhidgetSpatialHandle)ch, dr));
	case PHIDCHCLASS_TEMPERATURESENSOR:
		return (PhidgetTemperatureSensor_getMaxDataRate((PhidgetTemperatureSensorHandle)ch, dr));
	case PHIDCHCLASS_ENCODER:
		return (PhidgetEncoder_getMaxDataRate((PhidgetEncoderHandle)ch, dr));
	case PHIDCHCLASS_FREQUENCYCOUNTER:
		return (PhidgetFrequencyCounter_getMaxDataRate((PhidgetFrequencyCounterHandle)ch, dr));
	case PHIDCHCLASS_PHSENSOR:
		return (PhidgetPHSensor_getMaxDataRate((PhidgetPHSensorHandle)ch, dr));
	case PHIDCHCLASS_DCMOTOR:
		return (PhidgetDCMotor_getMaxDataRate((PhidgetDCMotorHandle)ch, dr));
	case PHIDCHCLASS_CURRENTINPUT:
		return (PhidgetCurrentInput_getMaxDataRate((PhidgetCurrentInputHandle)ch, dr));
	case PHIDCHCLASS_STEPPER:
		return (PhidgetStepper_getMaxDataRate((PhidgetStepperHandle)ch, dr));
	case PHIDCHCLASS_MOTORPOSITIONCONTROLLER:
		return (PhidgetMotorPositionController_getMaxDataRate((PhidgetMotorPositionControllerHandle)ch,
		  dr));
	case PHIDCHCLASS_BLDCMOTOR:
		return (PhidgetBLDCMotor_getMaxDataRate((PhidgetBLDCMotorHandle)ch, dr));
	case PHIDCHCLASS_DISTANCESENSOR:
		return (PhidgetDistanceSensor_getMaxDataRate((PhidgetDistanceSensorHandle)ch, dr));
	case PHIDCHCLASS_HUMIDITYSENSOR:
		return (PhidgetHumiditySensor_getMaxDataRate((PhidgetHumiditySensorHandle)ch, dr));
	case PHIDCHCLASS_LIGHTSENSOR:
		return (PhidgetLightSensor_getMaxDataRate((PhidgetLightSensorHandle)ch, dr));
	case PHIDCHCLASS_PRESSURESENSOR:
		return (PhidgetPressureSensor_getMaxDataRate((PhidgetPressureSensorHandle)ch, dr));
	case PHIDCHCLASS_RESISTANCEINPUT:
		return (PhidgetResistanceInput_getMaxDataRate((PhidgetResistanceInputHandle)ch, dr));
	case PHIDCHCLASS_SOUNDSENSOR:
		return (PhidgetSoundSensor_getMaxDataRate((PhidgetSoundSensorHandle)ch, dr));
	default:
		return (PHID_RETURN_ERRSTR(EPHIDGET_UNSUPPORTED,
		  "Data rate is not supported by this channel class."));
	}
}

API_PRETURN
Phidget_setDataInterval(PhidgetHandle ch, uint32_t di) {
	Phidget_ChannelClass cclass;
	PhidgetReturnCode res;

	res = Phidget_getChannelClass(ch, &cclass);
	if (res != EPHIDGET_OK)
		return (PHID_RETURN(res));

	switch (cclass) {
	case PHIDCHCLASS_VOLTAGERATIOINPUT:
		return (PhidgetVoltageRatioInput_setDataInterval((PhidgetVoltageRatioInputHandle)ch, di));
	case PHIDCHCLASS_RCSERVO:
		return (PhidgetRCServo_setDataInterval((PhidgetRCServoHandle)ch, di));
	case PHIDCHCLASS_ACCELEROMETER:
		return (PhidgetAccelerometer_setDataInterval((PhidgetAccelerometerHandle)ch, di));
	case PHIDCHCLASS_VOLTAGEINPUT:
		return (PhidgetVoltageInput_setDataInterval((PhidgetVoltageInputHandle)ch, di));
	case PHIDCHCLASS_CAPACITIVETOUCH:
		return (PhidgetCapacitiveTouch_setDataInterval((PhidgetCapacitiveTouchHandle)ch, di));
	case PHIDCHCLASS_GYROSCOPE:
		return (PhidgetGyroscope_setDataInterval((PhidgetGyroscopeHandle)ch, di));
	case PHIDCHCLASS_MAGNETOMETER:
		return (PhidgetMagnetometer_setDataInterval((PhidgetMagnetometerHandle)ch, di));
	case PHIDCHCLASS_SPATIAL:
		return (PhidgetSpatial_setDataInterval((PhidgetSpatialHandle)ch, di));
	case PHIDCHCLASS_TEMPERATURESENSOR:
		return (PhidgetTemperatureSensor_setDataInterval((PhidgetTemperatureSensorHandle)ch, di));
	case PHIDCHCLASS_ENCODER:
		return (PhidgetEncoder_setDataInterval((PhidgetEncoderHandle)ch, di));
	case PHIDCHCLASS_FREQUENCYCOUNTER:
		return (PhidgetFrequencyCounter_setDataInterval((PhidgetFrequencyCounterHandle)ch, di));
	case PHIDCHCLASS_PHSENSOR:
		return (PhidgetPHSensor_setDataInterval((PhidgetPHSensorHandle)ch, di));
	case PHIDCHCLASS_DCMOTOR:
		return (PhidgetDCMotor_setDataInterval((PhidgetDCMotorHandle)ch, di));
	case PHIDCHCLASS_CURRENTINPUT:
		return (PhidgetCurrentInput_setDataInterval((PhidgetCurrentInputHandle)ch, di));
	case PHIDCHCLASS_STEPPER:
		return (PhidgetStepper_setDataInterval((PhidgetStepperHandle)ch, di));
	case PHIDCHCLASS_MOTORPOSITIONCONTROLLER:
		return (PhidgetMotorPositionController_setDataInterval((PhidgetMotorPositionControllerHandle)ch,
		  di));
	case PHIDCHCLASS_BLDCMOTOR:
		return (PhidgetBLDCMotor_setDataInterval((PhidgetBLDCMotorHandle)ch, di));
	case PHIDCHCLASS_DISTANCESENSOR:
		return (PhidgetDistanceSensor_setDataInterval((PhidgetDistanceSensorHandle)ch, di));
	case PHIDCHCLASS_HUMIDITYSENSOR:
		return (PhidgetHumiditySensor_setDataInterval((PhidgetHumiditySensorHandle)ch, di));
	case PHIDCHCLASS_LIGHTSENSOR:
		return (PhidgetLightSensor_setDataInterval((PhidgetLightSensorHandle)ch, di));
	case PHIDCHCLASS_PRESSURESENSOR:
		return (PhidgetPressureSensor_setDataInterval((PhidgetPressureSensorHandle)ch, di));
	case PHIDCHCLASS_RESISTANCEINPUT:
		return (PhidgetResistanceInput_setDataInterval((PhidgetResistanceInputHandle)ch, di));
	case PHIDCHCLASS_SOUNDSENSOR:
		return (PhidgetSoundSensor_setDataInterval((PhidgetSoundSensorHandle)ch, di));
	default:
		return (PHID_RETURN_ERRSTR(EPHIDGET_UNSUPPORTED,
		  "Data interval is not supported by this channel class."));
	}
}

API_PRETURN
Phidget_setDataRate(PhidgetHandle ch, double dr) {
	Phidget_ChannelClass cclass;
	PhidgetReturnCode res;

	res = Phidget_getChannelClass(ch, &cclass);
	if (res != EPHIDGET_OK)
		return (PHID_RETURN(res));

	switch (cclass) {
	case PHIDCHCLASS_VOLTAGERATIOINPUT:
		return (PhidgetVoltageRatioInput_setDataRate((PhidgetVoltageRatioInputHandle)ch, dr));
	case PHIDCHCLASS_RCSERVO:
		return (PhidgetRCServo_setDataRate((PhidgetRCServoHandle)ch, dr));
	case PHIDCHCLASS_ACCELEROMETER:
		return (PhidgetAccelerometer_setDataRate((PhidgetAccelerometerHandle)ch, dr));
	case PHIDCHCLASS_VOLTAGEINPUT:
		return (PhidgetVoltageInput_setDataRate((PhidgetVoltageInputHandle)ch, dr));
	case PHIDCHCLASS_CAPACITIVETOUCH:
		return (PhidgetCapacitiveTouch_setDataRate((PhidgetCapacitiveTouchHandle)ch, dr));
	case PHIDCHCLASS_GYROSCOPE:
		return (PhidgetGyroscope_setDataRate((PhidgetGyroscopeHandle)ch, dr));
	case PHIDCHCLASS_MAGNETOMETER:
		return (PhidgetMagnetometer_setDataRate((PhidgetMagnetometerHandle)ch, dr));
	case PHIDCHCLASS_SPATIAL:
		return (PhidgetSpatial_setDataRate((PhidgetSpatialHandle)ch, dr));
	case PHIDCHCLASS_TEMPERATURESENSOR:
		return (PhidgetTemperatureSensor_setDataRate((PhidgetTemperatureSensorHandle)ch, dr));
	case PHIDCHCLASS_ENCODER:
		return (PhidgetEncoder_setDataRate((PhidgetEncoderHandle)ch, dr));
	case PHIDCHCLASS_FREQUENCYCOUNTER:
		return (PhidgetFrequencyCounter_setDataRate((PhidgetFrequencyCounterHandle)ch, dr));
	case PHIDCHCLASS_PHSENSOR:
		return (PhidgetPHSensor_setDataRate((PhidgetPHSensorHandle)ch, dr));
	case PHIDCHCLASS_DCMOTOR:
		return (PhidgetDCMotor_setDataRate((PhidgetDCMotorHandle)ch, dr));
	case PHIDCHCLASS_CURRENTINPUT:
		return (PhidgetCurrentInput_setDataRate((PhidgetCurrentInputHandle)ch, dr));
	case PHIDCHCLASS_STEPPER:
		return (PhidgetStepper_setDataRate((PhidgetStepperHandle)ch, dr));
	case PHIDCHCLASS_MOTORPOSITIONCONTROLLER:
		return (PhidgetMotorPositionController_setDataRate((PhidgetMotorPositionControllerHandle)ch,
		  dr));
	case PHIDCHCLASS_BLDCMOTOR:
		return (PhidgetBLDCMotor_setDataRate((PhidgetBLDCMotorHandle)ch, dr));
	case PHIDCHCLASS_DISTANCESENSOR:
		return (PhidgetDistanceSensor_setDataRate((PhidgetDistanceSensorHandle)ch, dr));
	case PHIDCHCLASS_HUMIDITYSENSOR:
		return (PhidgetHumiditySensor_setDataRate((PhidgetHumiditySensorHandle)ch, dr));
	case PHIDCHCLASS_LIGHTSENSOR:
		return (PhidgetLightSensor_setDataRate((PhidgetLightSensorHandle)ch, dr));
	case PHIDCHCLASS_PRESSURESENSOR:
		return (PhidgetPressureSensor_setDataRate((PhidgetPressureSensorHandle)ch, dr));
	case PHIDCHCLASS_RESISTANCEINPUT:
		return (PhidgetResistanceInput_setDataRate((PhidgetResistanceInputHandle)ch, dr));
	case PHIDCHCLASS_SOUNDSENSOR:
		return (PhidgetSoundSensor_setDataRate((PhidgetSoundSensorHandle)ch, dr));
	default:
		return (PHID_RETURN_ERRSTR(EPHIDGET_UNSUPPORTED,
		  "Data rate is not supported by this channel class."));
	}
}
