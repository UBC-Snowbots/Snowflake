#ifndef __PHLABVIEW
#define __PHLABVIEW

#ifdef _WINDOWS
#if _WIN64
#define LABVIEW_STRUCT_PACK 8
#else
#define LABVIEW_STRUCT_PACK 1
#endif
#pragma pack(push, LABVIEW_STRUCT_PACK)
#endif

typedef struct {
	int32_t length;	// Length of array
	void *data;		// Array
} lvArr, *lvArrHandle;

typedef struct {
	int64 ch;
} lvPhidgetAttachArgs;

API_PRETURN_HDR Phidget_setOnAttachLabviewHandler(PhidgetHandle ch, LVUserEventRef *lvEventRef);

typedef struct {
	int64 ch;
} lvPhidgetDetachArgs;

API_PRETURN_HDR Phidget_setOnDetachLabviewHandler(PhidgetHandle ch, LVUserEventRef *lvEventRef);

typedef struct {
	int64 ch;
	Phidget_ErrorEventCode code;
	LStrHandle description;
} lvPhidgetErrorArgs;

API_PRETURN_HDR Phidget_setOnErrorLabviewHandler(PhidgetHandle ch, LVUserEventRef *lvEventRef);

typedef struct {
	int64 ch;
	LStrHandle propertyName;
} lvPhidgetPropertyChangeArgs;

API_PRETURN_HDR Phidget_setOnPropertyChangeLabviewHandler(PhidgetHandle ch, LVUserEventRef *lvEventRef);

typedef struct {
	int64 man;
	int64 channel;
} lvPhidgetManagerAttachArgs;

API_PRETURN_HDR PhidgetManager_setOnAttachLabviewHandler(PhidgetManagerHandle man,
  LVUserEventRef *lvEventRef);

typedef struct {
	int64 man;
	int64 channel;
} lvPhidgetManagerDetachArgs;

API_PRETURN_HDR PhidgetManager_setOnDetachLabviewHandler(PhidgetManagerHandle man,
  LVUserEventRef *lvEventRef);

typedef struct {
	PhidgetServer server;
	void * kv;
} lvPhidgetNetServerAddedArgs;

API_PRETURN_HDR PhidgetNet_setOnServerAddedLabviewHandler(LVUserEventRef *lvEventRef);

typedef struct {
	PhidgetServer server;
} lvPhidgetNetServerRemovedArgs;

API_PRETURN_HDR PhidgetNet_setOnServerRemovedLabviewHandler(LVUserEventRef *lvEventRef);

typedef struct {
	int64 ch;
	double sensorValue;
	Phidget_UnitInfo sensorUnit;
} lvPhidgetVoltageRatioInputSensorChangeArgs;

API_PRETURN_HDR PhidgetVoltageRatioInput_setOnSensorChangeLabviewHandler(PhidgetVoltageRatioInputHandle ch, LVUserEventRef *lvEventRef);

typedef struct {
	int64 ch;
	double voltageRatio;
} lvPhidgetVoltageRatioInputVoltageRatioChangeArgs;

API_PRETURN_HDR PhidgetVoltageRatioInput_setOnVoltageRatioChangeLabviewHandler(PhidgetVoltageRatioInputHandle ch, LVUserEventRef *lvEventRef);

typedef struct {
	int64 ch;
	int state;
} lvPhidgetDigitalInputStateChangeArgs;

API_PRETURN_HDR PhidgetDigitalInput_setOnStateChangeLabviewHandler(PhidgetDigitalInputHandle ch,
  LVUserEventRef *lvEventRef);

typedef struct {
	int64 ch;
	double position;
} lvPhidgetRCServoPositionChangeArgs;

API_PRETURN_HDR PhidgetRCServo_setOnPositionChangeLabviewHandler(PhidgetRCServoHandle ch,
  LVUserEventRef *lvEventRef);

typedef struct {
	int64 ch;
	double position;
} lvPhidgetRCServoTargetPositionReachedArgs;

API_PRETURN_HDR PhidgetRCServo_setOnTargetPositionReachedLabviewHandler(PhidgetRCServoHandle ch,
  LVUserEventRef *lvEventRef);

typedef struct {
	int64 ch;
	double velocity;
} lvPhidgetRCServoVelocityChangeArgs;

API_PRETURN_HDR PhidgetRCServo_setOnVelocityChangeLabviewHandler(PhidgetRCServoHandle ch,
  LVUserEventRef *lvEventRef);

typedef struct {
	int64 ch;
	lvArrHandle* acceleration;
	double timestamp;
} lvPhidgetAccelerometerAccelerationChangeArgs;

API_PRETURN_HDR PhidgetAccelerometer_setOnAccelerationChangeLabviewHandler(PhidgetAccelerometerHandle ch,
  LVUserEventRef *lvEventRef);

typedef struct {
	int64 ch;
	double sensorValue;
	Phidget_UnitInfo sensorUnit;
} lvPhidgetVoltageInputSensorChangeArgs;

API_PRETURN_HDR PhidgetVoltageInput_setOnSensorChangeLabviewHandler(PhidgetVoltageInputHandle ch,
  LVUserEventRef *lvEventRef);

typedef struct {
	int64 ch;
	double voltage;
} lvPhidgetVoltageInputVoltageChangeArgs;

API_PRETURN_HDR PhidgetVoltageInput_setOnVoltageChangeLabviewHandler(PhidgetVoltageInputHandle ch,
  LVUserEventRef *lvEventRef);

typedef struct {
	int64 ch;
	double touchValue;
} lvPhidgetCapacitiveTouchTouchArgs;

API_PRETURN_HDR PhidgetCapacitiveTouch_setOnTouchLabviewHandler(PhidgetCapacitiveTouchHandle ch,
  LVUserEventRef *lvEventRef);

typedef struct {
	int64 ch;
} lvPhidgetCapacitiveTouchTouchEndArgs;

API_PRETURN_HDR PhidgetCapacitiveTouch_setOnTouchEndLabviewHandler(PhidgetCapacitiveTouchHandle ch,
  LVUserEventRef *lvEventRef);

typedef struct {
	int64 ch;
	LStrHandle tag;
	PhidgetRFID_Protocol protocol;
} lvPhidgetRFIDTagArgs;

API_PRETURN_HDR PhidgetRFID_setOnTagLabviewHandler(PhidgetRFIDHandle ch, LVUserEventRef *lvEventRef);

typedef struct {
	int64 ch;
	LStrHandle tag;
	PhidgetRFID_Protocol protocol;
} lvPhidgetRFIDTagLostArgs;

API_PRETURN_HDR PhidgetRFID_setOnTagLostLabviewHandler(PhidgetRFIDHandle ch,
  LVUserEventRef *lvEventRef);

typedef struct {
	int64 ch;
	double heading;
	double velocity;
} lvPhidgetGPSHeadingChangeArgs;

API_PRETURN_HDR PhidgetGPS_setOnHeadingChangeLabviewHandler(PhidgetGPSHandle ch,
  LVUserEventRef *lvEventRef);

typedef struct {
	int64 ch;
	double latitude;
	double longitude;
	double altitude;
} lvPhidgetGPSPositionChangeArgs;

API_PRETURN_HDR PhidgetGPS_setOnPositionChangeLabviewHandler(PhidgetGPSHandle ch,
  LVUserEventRef *lvEventRef);

typedef struct {
	int64 ch;
	int positionFixState;
} lvPhidgetGPSPositionFixStateChangeArgs;

API_PRETURN_HDR PhidgetGPS_setOnPositionFixStateChangeLabviewHandler(PhidgetGPSHandle ch,
  LVUserEventRef *lvEventRef);

typedef struct {
	int64 ch;
	lvArrHandle* angularRate;
	double timestamp;
} lvPhidgetGyroscopeAngularRateUpdateArgs;

API_PRETURN_HDR PhidgetGyroscope_setOnAngularRateUpdateLabviewHandler(PhidgetGyroscopeHandle ch,
  LVUserEventRef *lvEventRef);

typedef struct {
	int64 ch;
	lvArrHandle* magneticField;
	double timestamp;
} lvPhidgetMagnetometerMagneticFieldChangeArgs;

API_PRETURN_HDR PhidgetMagnetometer_setOnMagneticFieldChangeLabviewHandler(PhidgetMagnetometerHandle ch,
  LVUserEventRef *lvEventRef);

typedef struct {
	int64 ch;
	lvArrHandle* quaternion;
	double timestamp;
} lvPhidgetSpatialAlgorithmDataArgs;

API_PRETURN_HDR PhidgetSpatial_setOnAlgorithmDataLabviewHandler(PhidgetSpatialHandle ch,
  LVUserEventRef *lvEventRef);

typedef struct {
	int64 ch;
	lvArrHandle* acceleration;
	lvArrHandle* angularRate;
	lvArrHandle* magneticField;
	double timestamp;
} lvPhidgetSpatialSpatialDataArgs;

API_PRETURN_HDR PhidgetSpatial_setOnSpatialDataLabviewHandler(PhidgetSpatialHandle ch,
  LVUserEventRef *lvEventRef);

typedef struct {
	int64 ch;
	double temperature;
} lvPhidgetTemperatureSensorTemperatureChangeArgs;

API_PRETURN_HDR PhidgetTemperatureSensor_setOnTemperatureChangeLabviewHandler(PhidgetTemperatureSensorHandle ch, LVUserEventRef *lvEventRef);

typedef struct {
	int64 ch;
	int positionChange;
	double timeChange;
	int indexTriggered;
} lvPhidgetEncoderPositionChangeArgs;

API_PRETURN_HDR PhidgetEncoder_setOnPositionChangeLabviewHandler(PhidgetEncoderHandle ch,
  LVUserEventRef *lvEventRef);

typedef struct {
	int64 ch;
	uint64_t counts;
	double timeChange;
} lvPhidgetFrequencyCounterCountChangeArgs;

API_PRETURN_HDR PhidgetFrequencyCounter_setOnCountChangeLabviewHandler(PhidgetFrequencyCounterHandle ch,
  LVUserEventRef *lvEventRef);

typedef struct {
	int64 ch;
	double frequency;
} lvPhidgetFrequencyCounterFrequencyChangeArgs;

API_PRETURN_HDR PhidgetFrequencyCounter_setOnFrequencyChangeLabviewHandler(PhidgetFrequencyCounterHandle ch, LVUserEventRef *lvEventRef);

typedef struct {
	int64 ch;
	LStrHandle code;
	uint32_t bitCount;
	int isRepeat;
} lvPhidgetIRCodeArgs;

API_PRETURN_HDR PhidgetIR_setOnCodeLabviewHandler(PhidgetIRHandle ch, LVUserEventRef *lvEventRef);

typedef struct {
	int64 ch;
	LStrHandle code;
	PhidgetIR_CodeInfo codeInfo;
} lvPhidgetIRLearnArgs;

API_PRETURN_HDR PhidgetIR_setOnLearnLabviewHandler(PhidgetIRHandle ch, LVUserEventRef *lvEventRef);

typedef struct {
	int64 ch;
	lvArrHandle* data;
} lvPhidgetIRRawDataArgs;

API_PRETURN_HDR PhidgetIR_setOnRawDataLabviewHandler(PhidgetIRHandle ch, LVUserEventRef *lvEventRef);

typedef struct {
	int64 ch;
	double PH;
} lvPhidgetPHSensorPHChangeArgs;

API_PRETURN_HDR PhidgetPHSensor_setOnPHChangeLabviewHandler(PhidgetPHSensorHandle ch,
  LVUserEventRef *lvEventRef);

typedef struct {
	int64 ch;
	double backEMF;
} lvPhidgetDCMotorBackEMFChangeArgs;

API_PRETURN_HDR PhidgetDCMotor_setOnBackEMFChangeLabviewHandler(PhidgetDCMotorHandle ch,
  LVUserEventRef *lvEventRef);

typedef struct {
	int64 ch;
	double brakingStrength;
} lvPhidgetDCMotorBrakingStrengthChangeArgs;

API_PRETURN_HDR PhidgetDCMotor_setOnBrakingStrengthChangeLabviewHandler(PhidgetDCMotorHandle ch,
  LVUserEventRef *lvEventRef);

typedef struct {
	int64 ch;
	double velocity;
} lvPhidgetDCMotorVelocityUpdateArgs;

API_PRETURN_HDR PhidgetDCMotor_setOnVelocityUpdateLabviewHandler(PhidgetDCMotorHandle ch,
  LVUserEventRef *lvEventRef);

typedef struct {
	int64 ch;
	double current;
} lvPhidgetCurrentInputCurrentChangeArgs;

API_PRETURN_HDR PhidgetCurrentInput_setOnCurrentChangeLabviewHandler(PhidgetCurrentInputHandle ch,
  LVUserEventRef *lvEventRef);

typedef struct {
	int64 ch;
	double position;
} lvPhidgetStepperPositionChangeArgs;

API_PRETURN_HDR PhidgetStepper_setOnPositionChangeLabviewHandler(PhidgetStepperHandle ch,
  LVUserEventRef *lvEventRef);

typedef struct {
	int64 ch;
} lvPhidgetStepperStoppedArgs;

API_PRETURN_HDR PhidgetStepper_setOnStoppedLabviewHandler(PhidgetStepperHandle ch,
  LVUserEventRef *lvEventRef);

typedef struct {
	int64 ch;
	double velocity;
} lvPhidgetStepperVelocityChangeArgs;

API_PRETURN_HDR PhidgetStepper_setOnVelocityChangeLabviewHandler(PhidgetStepperHandle ch,
  LVUserEventRef *lvEventRef);

typedef struct {
	int64 ch;
	lvArrHandle* data;
	PhidgetDataAdapter_PacketErrorCode error;
} lvPhidgetDataAdapterPacketArgs;

API_PRETURN_HDR PhidgetDataAdapter_setOnPacketLabviewHandler(PhidgetDataAdapterHandle ch,
  LVUserEventRef *lvEventRef);

typedef struct {
	int64 ch;
	double dutyCycle;
} lvPhidgetMotorPositionControllerDutyCycleUpdateArgs;

API_PRETURN_HDR PhidgetMotorPositionController_setOnDutyCycleUpdateLabviewHandler(PhidgetMotorPositionControllerHandle ch, LVUserEventRef *lvEventRef);

typedef struct {
	int64 ch;
	double position;
} lvPhidgetMotorPositionControllerPositionChangeArgs;

API_PRETURN_HDR PhidgetMotorPositionController_setOnPositionChangeLabviewHandler(PhidgetMotorPositionControllerHandle ch, LVUserEventRef *lvEventRef);

typedef struct {
	int64 ch;
	double brakingStrength;
} lvPhidgetBLDCMotorBrakingStrengthChangeArgs;

API_PRETURN_HDR PhidgetBLDCMotor_setOnBrakingStrengthChangeLabviewHandler(PhidgetBLDCMotorHandle ch,
  LVUserEventRef *lvEventRef);

typedef struct {
	int64 ch;
	double position;
} lvPhidgetBLDCMotorPositionChangeArgs;

API_PRETURN_HDR PhidgetBLDCMotor_setOnPositionChangeLabviewHandler(PhidgetBLDCMotorHandle ch,
  LVUserEventRef *lvEventRef);

typedef struct {
	int64 ch;
	double velocity;
} lvPhidgetBLDCMotorVelocityUpdateArgs;

API_PRETURN_HDR PhidgetBLDCMotor_setOnVelocityUpdateLabviewHandler(PhidgetBLDCMotorHandle ch,
  LVUserEventRef *lvEventRef);

typedef struct {
	int64 ch;
	uint32_t distance;
} lvPhidgetDistanceSensorDistanceChangeArgs;

API_PRETURN_HDR PhidgetDistanceSensor_setOnDistanceChangeLabviewHandler(PhidgetDistanceSensorHandle ch,
  LVUserEventRef *lvEventRef);

typedef struct {
	int64 ch;
	lvArrHandle* distances;
	lvArrHandle* amplitudes;
	uint32_t count;
} lvPhidgetDistanceSensorSonarReflectionsUpdateArgs;

API_PRETURN_HDR PhidgetDistanceSensor_setOnSonarReflectionsUpdateLabviewHandler(PhidgetDistanceSensorHandle ch, LVUserEventRef *lvEventRef);

typedef struct {
	int64 ch;
	double humidity;
} lvPhidgetHumiditySensorHumidityChangeArgs;

API_PRETURN_HDR PhidgetHumiditySensor_setOnHumidityChangeLabviewHandler(PhidgetHumiditySensorHandle ch,
  LVUserEventRef *lvEventRef);

typedef struct {
	int64 ch;
	double illuminance;
} lvPhidgetLightSensorIlluminanceChangeArgs;

API_PRETURN_HDR PhidgetLightSensor_setOnIlluminanceChangeLabviewHandler(PhidgetLightSensorHandle ch,
  LVUserEventRef *lvEventRef);

typedef struct {
	int64 ch;
	double pressure;
} lvPhidgetPressureSensorPressureChangeArgs;

API_PRETURN_HDR PhidgetPressureSensor_setOnPressureChangeLabviewHandler(PhidgetPressureSensorHandle ch,
  LVUserEventRef *lvEventRef);

typedef struct {
	int64 ch;
	double resistance;
} lvPhidgetResistanceInputResistanceChangeArgs;

API_PRETURN_HDR PhidgetResistanceInput_setOnResistanceChangeLabviewHandler(PhidgetResistanceInputHandle ch, LVUserEventRef *lvEventRef);

typedef struct {
	int64 ch;
	double dB;
	double dBA;
	double dBC;
	lvArrHandle* octaves;
} lvPhidgetSoundSensorSPLChangeArgs;

API_PRETURN_HDR PhidgetSoundSensor_setOnSPLChangeLabviewHandler(PhidgetSoundSensorHandle ch,
  LVUserEventRef *lvEventRef);

typedef struct {
	int64 ch;
	double progress;
} lvPhidgetFirmwareUpgradeProgressChangeArgs;

API_PRETURN_HDR PhidgetFirmwareUpgrade_setOnProgressChangeLabviewHandler(PhidgetFirmwareUpgradeHandle ch,
  LVUserEventRef *lvEventRef);

typedef struct {
	int64 ch;
	LStrHandle key;
	LStrHandle value;
} lvPhidgetDictionaryAddArgs;

API_PRETURN_HDR PhidgetDictionary_setOnAddLabviewHandler(PhidgetDictionaryHandle ch,
  LVUserEventRef *lvEventRef);

typedef struct {
	int64 ch;
	LStrHandle key;
} lvPhidgetDictionaryRemoveArgs;

API_PRETURN_HDR PhidgetDictionary_setOnRemoveLabviewHandler(PhidgetDictionaryHandle ch,
  LVUserEventRef *lvEventRef);

typedef struct {
	int64 ch;
	LStrHandle key;
	LStrHandle value;
} lvPhidgetDictionaryUpdateArgs;

API_PRETURN_HDR PhidgetDictionary_setOnUpdateLabviewHandler(PhidgetDictionaryHandle ch,
  LVUserEventRef *lvEventRef);


#ifdef _WINDOWS
#pragma pack(pop)
#endif

#endif
