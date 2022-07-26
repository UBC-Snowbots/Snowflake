/* Generated: Mon May 30 2022 10:16:30 GMT-0600 (Mountain Daylight Time) */

#include "device/servodevice.h"
#include "device/advancedservodevice.h"
static void CCONV PhidgetRCServo_errorHandler(PhidgetChannelHandle ch, Phidget_ErrorEventCode code);
static void CCONV PhidgetRCServo_free(PhidgetChannelHandle *ch);
static PhidgetReturnCode CCONV PhidgetRCServo_bridgeInput(PhidgetChannelHandle phid, BridgePacket *bp);
static PhidgetReturnCode CCONV PhidgetRCServo_setStatus(PhidgetChannelHandle phid, BridgePacket *bp);
static PhidgetReturnCode CCONV PhidgetRCServo_getStatus(PhidgetChannelHandle phid, BridgePacket **bp);
static PhidgetReturnCode CCONV PhidgetRCServo_initAfterOpen(PhidgetChannelHandle phid);
static PhidgetReturnCode CCONV PhidgetRCServo_setDefaults(PhidgetChannelHandle phid);
static void CCONV PhidgetRCServo_fireInitialEvents(PhidgetChannelHandle phid);
static int CCONV PhidgetRCServo_hasInitialState(PhidgetChannelHandle phid);

struct _PhidgetRCServo {
	struct _PhidgetChannel phid;
	double acceleration;
	double minAcceleration;
	double maxAcceleration;
	double dataInterval;
	uint32_t minDataInterval;
	uint32_t maxDataInterval;
	double minDataRate;
	double maxDataRate;
	int engaged;
	uint32_t minFailsafeTime;
	uint32_t maxFailsafeTime;
	int isMoving;
	double position;
	double minPosition;
	double maxPosition;
	double minPulseWidth;
	double maxPulseWidth;
	double minPulseWidthLimit;
	double maxPulseWidthLimit;
	int speedRampingState;
	double targetPosition;
	double torque;
	double minTorque;
	double maxTorque;
	double velocity;
	double velocityLimit;
	double minVelocityLimit;
	double maxVelocityLimit;
	PhidgetRCServo_Voltage voltage;
	PhidgetRCServo_OnPositionChangeCallback PositionChange;
	void *PositionChangeCtx;
	PhidgetRCServo_OnTargetPositionReachedCallback TargetPositionReached;
	void *TargetPositionReachedCtx;
	PhidgetRCServo_OnVelocityChangeCallback VelocityChange;
	void *VelocityChangeCtx;
};

static PhidgetReturnCode CCONV
_setStatus(PhidgetChannelHandle phid, BridgePacket *bp) {
	PhidgetRCServoHandle ch;
	int version;

	ch = (PhidgetRCServoHandle)phid;

	version = getBridgePacketUInt32ByName(bp, "_class_version_");
	if (version != 5) {
		loginfo("%"PRIphid": server/client class version mismatch: %d != 5 - functionality may be limited.", phid, version);
	}

	if (version >= 2)
		ch->acceleration = getBridgePacketDoubleByName(bp, "acceleration");
	if (version >= 0)
		ch->minAcceleration = getBridgePacketDoubleByName(bp, "minAcceleration");
	if (version >= 0)
		ch->maxAcceleration = getBridgePacketDoubleByName(bp, "maxAcceleration");
	if (version >= 0)
		ch->minDataInterval = getBridgePacketUInt32ByName(bp, "minDataInterval");
	if (version >= 0)
		ch->maxDataInterval = getBridgePacketUInt32ByName(bp, "maxDataInterval");
	if (version >= 5)
		ch->dataInterval = getBridgePacketDoubleByName(bp, "dataIntervalDbl");
	else if(version >= 0)
		ch->dataInterval = (double)getBridgePacketUInt32ByName(bp, "dataInterval");
	if (version >= 5)
		ch->minDataRate = getBridgePacketDoubleByName(bp, "minDataRate");
	else if(version >= 0)
		ch->minDataRate = (double)(1000.0 / getBridgePacketUInt32ByName(bp, "maxDataInterval"));
	if (version >= 5)
		ch->maxDataRate = getBridgePacketDoubleByName(bp, "maxDataRate");
	else if(version >= 0)
		ch->maxDataRate = (double)(1000.0 / getBridgePacketUInt32ByName(bp, "minDataInterval"));
	if (version >= 0)
		ch->engaged = getBridgePacketInt32ByName(bp, "engaged");
	if (version >= 4)
		ch->minFailsafeTime = getBridgePacketUInt32ByName(bp, "minFailsafeTime");
	if (version >= 4)
		ch->maxFailsafeTime = getBridgePacketUInt32ByName(bp, "maxFailsafeTime");
	if (version >= 3)
		ch->isMoving = getBridgePacketInt32ByName(bp, "isMoving");
	if (version >= 0)
		ch->position = getBridgePacketDoubleByName(bp, "position");
	if (version >= 0)
		ch->minPosition = getBridgePacketDoubleByName(bp, "minPosition");
	if (version >= 0)
		ch->maxPosition = getBridgePacketDoubleByName(bp, "maxPosition");
	if (version >= 0)
		ch->minPulseWidth = getBridgePacketDoubleByName(bp, "minPulseWidth");
	if (version >= 0)
		ch->maxPulseWidth = getBridgePacketDoubleByName(bp, "maxPulseWidth");
	if (version >= 0)
		ch->minPulseWidthLimit = getBridgePacketDoubleByName(bp, "minPulseWidthLimit");
	if (version >= 0)
		ch->maxPulseWidthLimit = getBridgePacketDoubleByName(bp, "maxPulseWidthLimit");
	if (version >= 0)
		ch->speedRampingState = getBridgePacketInt32ByName(bp, "speedRampingState");
	if (version >= 2)
		ch->targetPosition = getBridgePacketDoubleByName(bp, "targetPosition");
	if (version >= 0)
		ch->torque = getBridgePacketDoubleByName(bp, "torque");
	if (version >= 0)
		ch->minTorque = getBridgePacketDoubleByName(bp, "minTorque");
	if (version >= 0)
		ch->maxTorque = getBridgePacketDoubleByName(bp, "maxTorque");
	if (version >= 2)
		ch->velocity = getBridgePacketDoubleByName(bp, "velocity");
	if (version >= 2)
		ch->velocityLimit = getBridgePacketDoubleByName(bp, "velocityLimit");
	if (version >= 0)
		ch->minVelocityLimit = getBridgePacketDoubleByName(bp, "minVelocityLimit");
	if (version >= 0)
		ch->maxVelocityLimit = getBridgePacketDoubleByName(bp, "maxVelocityLimit");
	if (version >= 0)
		ch->voltage = getBridgePacketInt32ByName(bp, "voltage");

	return (EPHIDGET_OK);
}

static PhidgetReturnCode CCONV
_getStatus(PhidgetChannelHandle phid, BridgePacket **bp) {
	PhidgetRCServoHandle ch;

	ch = (PhidgetRCServoHandle)phid;

	return (createBridgePacket(bp, 0, "_class_version_=%u"
	  ",acceleration=%g"
	  ",minAcceleration=%g"
	  ",maxAcceleration=%g"
	  ",dataInterval=%u"
	  ",minDataInterval=%u"
	  ",maxDataInterval=%u"
	  ",dataIntervalDbl=%g"
	  ",minDataRate=%g"
	  ",maxDataRate=%g"
	  ",engaged=%d"
	  ",minFailsafeTime=%u"
	  ",maxFailsafeTime=%u"
	  ",isMoving=%d"
	  ",position=%g"
	  ",minPosition=%g"
	  ",maxPosition=%g"
	  ",minPulseWidth=%g"
	  ",maxPulseWidth=%g"
	  ",minPulseWidthLimit=%g"
	  ",maxPulseWidthLimit=%g"
	  ",speedRampingState=%d"
	  ",targetPosition=%g"
	  ",torque=%g"
	  ",minTorque=%g"
	  ",maxTorque=%g"
	  ",velocity=%g"
	  ",velocityLimit=%g"
	  ",minVelocityLimit=%g"
	  ",maxVelocityLimit=%g"
	  ",voltage=%d"
	  ,5 /* class version */
	  ,ch->acceleration
	  ,ch->minAcceleration
	  ,ch->maxAcceleration
	  ,(uint32_t)round(ch->dataInterval)
	  ,ch->minDataInterval
	  ,ch->maxDataInterval
	  ,ch->dataInterval
	  ,ch->minDataRate
	  ,ch->maxDataRate
	  ,ch->engaged
	  ,ch->minFailsafeTime
	  ,ch->maxFailsafeTime
	  ,ch->isMoving
	  ,ch->position
	  ,ch->minPosition
	  ,ch->maxPosition
	  ,ch->minPulseWidth
	  ,ch->maxPulseWidth
	  ,ch->minPulseWidthLimit
	  ,ch->maxPulseWidthLimit
	  ,ch->speedRampingState
	  ,ch->targetPosition
	  ,ch->torque
	  ,ch->minTorque
	  ,ch->maxTorque
	  ,ch->velocity
	  ,ch->velocityLimit
	  ,ch->minVelocityLimit
	  ,ch->maxVelocityLimit
	  ,ch->voltage
	));
}

static PhidgetReturnCode CCONV
_bridgeInput(PhidgetChannelHandle phid, BridgePacket *bp) {
	PhidgetRCServoHandle ch;
	PhidgetReturnCode res;

	ch = (PhidgetRCServoHandle)phid;
	res = EPHIDGET_OK;

	switch (bp->vpkt) {
	case BP_SETFAILSAFETIME:
		res = DEVBRIDGEINPUT(phid, bp);
		break;
	case BP_FAILSAFERESET:
		res = DEVBRIDGEINPUT(phid, bp);
		break;
	case BP_SETACCELERATION:
		TESTRANGE_IOP(bp->iop, "%lf", getBridgePacketDouble(bp, 0), ch->minAcceleration,
		  ch->maxAcceleration);
		res = DEVBRIDGEINPUT(phid, bp);
		if (res != EPHIDGET_OK)
			break;
		ch->acceleration = getBridgePacketDouble(bp, 0);
		if (bridgePacketIsFromNet(bp)) {
			FIRE_PROPERTYCHANGE(ch, "Acceleration");
		}
		break;
	case BP_SETDATAINTERVAL:
		if (bp->entrycnt > 1)
			TESTRANGE_IOP(bp->iop, "%lf", round_double((1000.0 / getBridgePacketDouble(bp, 1)), 4),
			  ch->minDataRate, ch->maxDataRate);
		else
			TESTRANGE_IOP(bp->iop, "%"PRIu32, getBridgePacketUInt32(bp, 0), ch->minDataInterval,
			  ch->maxDataInterval);
		res = DEVBRIDGEINPUT(phid, bp);
		if (res != EPHIDGET_OK)
			break;
		if (bp->entrycnt > 1)
			ch->dataInterval = getBridgePacketDouble(bp, 1);
		else
			ch->dataInterval = (double)getBridgePacketUInt32(bp, 0);
		if (bridgePacketIsFromNet(bp)) {
			FIRE_PROPERTYCHANGE(ch, "DataInterval");
			FIRE_PROPERTYCHANGE(ch, "DataRate");
		}
		break;
	case BP_SETENGAGED:
		TESTBOOL_IOP(bp->iop, getBridgePacketInt32(bp, 0));
		res = DEVBRIDGEINPUT(phid, bp);
		if (res != EPHIDGET_OK)
			break;
		ch->engaged = getBridgePacketInt32(bp, 0);
		if (bridgePacketIsFromNet(bp)) {
			FIRE_PROPERTYCHANGE(ch, "Engaged");
		}
		break;
	case BP_SETMINPULSEWIDTH:
		res = DEVBRIDGEINPUT(phid, bp);
		if (res != EPHIDGET_OK)
			break;
		ch->minPulseWidth = getBridgePacketDouble(bp, 0);
		if (bridgePacketIsFromNet(bp)) {
			FIRE_PROPERTYCHANGE(ch, "MinPulseWidth");
		}
		break;
	case BP_SETMAXPULSEWIDTH:
		res = DEVBRIDGEINPUT(phid, bp);
		if (res != EPHIDGET_OK)
			break;
		ch->maxPulseWidth = getBridgePacketDouble(bp, 0);
		if (bridgePacketIsFromNet(bp)) {
			FIRE_PROPERTYCHANGE(ch, "MaxPulseWidth");
		}
		break;
	case BP_SETSPEEDRAMPINGSTATE:
		TESTBOOL_IOP(bp->iop, getBridgePacketInt32(bp, 0));
		res = DEVBRIDGEINPUT(phid, bp);
		if (res != EPHIDGET_OK)
			break;
		ch->speedRampingState = getBridgePacketInt32(bp, 0);
		if (bridgePacketIsFromNet(bp)) {
			FIRE_PROPERTYCHANGE(ch, "SpeedRampingState");
		}
		break;
	case BP_SETTARGETPOSITION:
		res = DEVBRIDGEINPUT(phid, bp);
		if (res != EPHIDGET_OK)
			break;
		ch->targetPosition = getBridgePacketDouble(bp, 0);
		if (bridgePacketIsFromNet(bp)) {
			FIRE_PROPERTYCHANGE(ch, "TargetPosition");
		}
		break;
	case BP_SETDUTYCYCLE:
		TESTRANGE_IOP(bp->iop, "%lf", getBridgePacketDouble(bp, 0), ch->minTorque, ch->maxTorque);
		res = DEVBRIDGEINPUT(phid, bp);
		if (res != EPHIDGET_OK)
			break;
		ch->torque = getBridgePacketDouble(bp, 0);
		if (bridgePacketIsFromNet(bp)) {
			FIRE_PROPERTYCHANGE(ch, "Torque");
		}
		break;
	case BP_SETVELOCITYLIMIT:
		TESTRANGE_IOP(bp->iop, "%lf", getBridgePacketDouble(bp, 0), ch->minVelocityLimit,
		  ch->maxVelocityLimit);
		res = DEVBRIDGEINPUT(phid, bp);
		if (res != EPHIDGET_OK)
			break;
		ch->velocityLimit = getBridgePacketDouble(bp, 0);
		if (bridgePacketIsFromNet(bp)) {
			FIRE_PROPERTYCHANGE(ch, "VelocityLimit");
		}
		break;
	case BP_SETVOLTAGE:
		if (!supportedRCServoVoltage(phid, (PhidgetRCServo_Voltage)getBridgePacketInt32(bp, 0)))
			return (MOS_ERROR(bp->iop, EPHIDGET_INVALIDARG,
			  "Specified RCServoVoltage is unsupported by this device."));
		res = DEVBRIDGEINPUT(phid, bp);
		if (res != EPHIDGET_OK)
			break;
		ch->voltage = getBridgePacketInt32(bp, 0);
		if (bridgePacketIsFromNet(bp)) {
			FIRE_PROPERTYCHANGE(ch, "Voltage");
		}
		break;
	default:
		logerr("%"PRIphid": unsupported bridge packet:0x%x", phid, bp->vpkt);
		res = EPHIDGET_UNSUPPORTED;
	}

	return (res);
}

static PhidgetReturnCode CCONV
_initAfterOpen(PhidgetChannelHandle phid) {
	PhidgetAdvancedServoDeviceHandle parentAdvancedServo;
	PhidgetServoDeviceHandle parentServo;
	PhidgetRCServoHandle ch;
	PhidgetReturnCode ret;

	TESTPTR(phid);
	ch = (PhidgetRCServoHandle)phid;

	ret = EPHIDGET_OK;

	parentAdvancedServo = (PhidgetAdvancedServoDeviceHandle)phid->parent;
	parentServo = (PhidgetServoDeviceHandle)phid->parent;

	switch (phid->UCD->uid) {
	case PHIDCHUID_1000_RCSERVO_OLD1_200:
		ch->maxPulseWidthLimit = 2550;
		ch->maxPulseWidth = 2450;
		ch->maxPosition = 180;
		ch->minPulseWidthLimit = 1;
		ch->minPulseWidth = 550;
		ch->minPosition = 0;
		ch->position = parentServo->position[ch->phid.index];
		ch->targetPosition = PUNK_DBL;
		ch->engaged = 0;
		ch->speedRampingState = 0;
		ch->voltage = RCSERVO_VOLTAGE_5V;
		break;
	case PHIDCHUID_1000_RCSERVO_OLD2_200:
		ch->maxPulseWidthLimit = 2550;
		ch->maxPulseWidth = 2450;
		ch->maxPosition = 180;
		ch->minPulseWidthLimit = 1;
		ch->minPulseWidth = 550;
		ch->minPosition = 0;
		ch->position = parentServo->position[ch->phid.index];
		ch->targetPosition = PUNK_DBL;
		ch->engaged = 0;
		ch->speedRampingState = 0;
		ch->voltage = RCSERVO_VOLTAGE_5V;
		break;
	case PHIDCHUID_1000_RCSERVO_300:
		ch->maxPulseWidthLimit = 4095;
		ch->maxPulseWidth = 2450;
		ch->maxPosition = 180;
		ch->minPulseWidthLimit = 1;
		ch->minPulseWidth = 550;
		ch->minPosition = 0;
		ch->position = parentServo->position[ch->phid.index];
		ch->targetPosition = PUNK_DBL;
		ch->engaged = 0;
		ch->speedRampingState = 0;
		ch->voltage = RCSERVO_VOLTAGE_5V;
		break;
	case PHIDCHUID_1000_RCSERVO_313:
		ch->maxPulseWidthLimit = 4095;
		ch->maxPulseWidth = 2450;
		ch->maxPosition = 180;
		ch->minPulseWidthLimit = 1;
		ch->minPulseWidth = 550;
		ch->minPosition = 0;
		ch->position = parentServo->position[ch->phid.index];
		ch->targetPosition = PUNK_DBL;
		ch->engaged = 0;
		ch->speedRampingState = 0;
		ch->voltage = RCSERVO_VOLTAGE_5V;
		break;
	case PHIDCHUID_1001_RCSERVO_OLD1_200:
		ch->maxPulseWidthLimit = 2550;
		ch->maxPulseWidth = 2450;
		ch->maxPosition = 180;
		ch->minPulseWidthLimit = 1;
		ch->minPulseWidth = 550;
		ch->minPosition = 0;
		ch->position = parentServo->position[ch->phid.index];
		ch->targetPosition = PUNK_DBL;
		ch->engaged = 0;
		ch->speedRampingState = 0;
		ch->voltage = RCSERVO_VOLTAGE_5V;
		break;
	case PHIDCHUID_1001_RCSERVO_OLD2_200:
		ch->maxPulseWidthLimit = 2550;
		ch->maxPulseWidth = 2450;
		ch->maxPosition = 180;
		ch->minPulseWidthLimit = 1;
		ch->minPulseWidth = 550;
		ch->minPosition = 0;
		ch->position = parentServo->position[ch->phid.index];
		ch->targetPosition = PUNK_DBL;
		ch->engaged = 0;
		ch->speedRampingState = 0;
		ch->voltage = RCSERVO_VOLTAGE_5V;
		break;
	case PHIDCHUID_1001_RCSERVO_313:
		ch->maxPulseWidthLimit = 4095;
		ch->maxPulseWidth = 2450;
		ch->maxPosition = 180;
		ch->minPulseWidthLimit = 1;
		ch->minPulseWidth = 550;
		ch->minPosition = 0;
		ch->position = parentServo->position[ch->phid.index];
		ch->targetPosition = PUNK_DBL;
		ch->engaged = 0;
		ch->speedRampingState = 0;
		ch->voltage = RCSERVO_VOLTAGE_5V;
		break;
	case PHIDCHUID_1001_RCSERVO_400:
		ch->maxPulseWidthLimit = 4095;
		ch->maxPulseWidth = 2450;
		ch->maxPosition = 180;
		ch->minPulseWidthLimit = 1;
		ch->minPulseWidth = 550;
		ch->minPosition = 0;
		ch->position = parentServo->position[ch->phid.index];
		ch->targetPosition = PUNK_DBL;
		ch->engaged = 0;
		ch->speedRampingState = 0;
		ch->voltage = RCSERVO_VOLTAGE_5V;
		break;
	case PHIDCHUID_1061_RCSERVO_100:
		ch->maxPulseWidthLimit = 2730.666;
		ch->maxPulseWidth = 2450;
		ch->maxPosition = 180;
		ch->maxVelocityLimit = 68266.666;
		ch->maxAcceleration = 3413333.333;
		ch->minPulseWidthLimit = 0.083;
		ch->minPulseWidth = 550;
		ch->minPosition = 0;
		ch->minVelocityLimit = 0;
		ch->minAcceleration = 208.334;
		ch->position = parentAdvancedServo->position[ch->phid.index];
		ch->targetPosition = PUNK_DBL;
		ch->velocityLimit = 1900;
		ch->velocity = parentAdvancedServo->velocity[ch->phid.index];
		ch->acceleration = 3800;
		ch->isMoving = parentAdvancedServo->isMoving[ch->phid.index];
		ch->engaged = 0;
		ch->speedRampingState = 1;
		ch->voltage = RCSERVO_VOLTAGE_5V;
		ch->dataInterval = 256;
		ch->maxDataInterval = 60000;
		ch->minDataInterval = 32;
		ch->minDataRate = 0.016666666666666666;
		ch->maxDataRate = 31.25;
		break;
	case PHIDCHUID_1061_RCSERVO_200:
		ch->maxPulseWidthLimit = 2730.666;
		ch->maxPulseWidth = 2450;
		ch->maxPosition = 180;
		ch->maxVelocityLimit = 68266.666;
		ch->maxAcceleration = 3413333.333;
		ch->minPulseWidthLimit = 0.083;
		ch->minPulseWidth = 550;
		ch->minPosition = 0;
		ch->minVelocityLimit = 0;
		ch->minAcceleration = 208.334;
		ch->position = parentAdvancedServo->position[ch->phid.index];
		ch->targetPosition = PUNK_DBL;
		ch->velocityLimit = 1900;
		ch->velocity = parentAdvancedServo->velocity[ch->phid.index];
		ch->acceleration = 3800;
		ch->isMoving = parentAdvancedServo->isMoving[ch->phid.index];
		ch->engaged = 0;
		ch->speedRampingState = 1;
		ch->voltage = RCSERVO_VOLTAGE_5V;
		ch->dataInterval = 256;
		ch->maxDataInterval = 60000;
		ch->minDataInterval = 32;
		ch->minDataRate = 0.016666666666666666;
		ch->maxDataRate = 31.25;
		break;
	case PHIDCHUID_1061_RCSERVO_300:
		ch->maxPulseWidthLimit = 2730.666;
		ch->maxPulseWidth = 2450;
		ch->maxPosition = 180;
		ch->maxVelocityLimit = 68266.666;
		ch->maxAcceleration = 3413333.333;
		ch->minPulseWidthLimit = 0.083;
		ch->minPulseWidth = 550;
		ch->minPosition = 0;
		ch->minVelocityLimit = 0;
		ch->minAcceleration = 208.334;
		ch->position = parentAdvancedServo->position[ch->phid.index];
		ch->targetPosition = PUNK_DBL;
		ch->velocityLimit = 1900;
		ch->velocity = parentAdvancedServo->velocity[ch->phid.index];
		ch->acceleration = 3800;
		ch->isMoving = parentAdvancedServo->isMoving[ch->phid.index];
		ch->engaged = 0;
		ch->speedRampingState = 1;
		ch->voltage = RCSERVO_VOLTAGE_5V;
		ch->dataInterval = 256;
		ch->maxDataInterval = 60000;
		ch->minDataInterval = 32;
		ch->minDataRate = 0.016666666666666666;
		ch->maxDataRate = 31.25;
		break;
	case PHIDCHUID_RCC0004_RCSERVO_400:
		ch->maxPulseWidthLimit = 2730.666;
		ch->maxPulseWidth = 2450;
		ch->maxPosition = 180;
		ch->maxVelocityLimit = 68266.666;
		ch->maxAcceleration = 3413333.333;
		ch->minPulseWidthLimit = 0.083;
		ch->minPulseWidth = 550;
		ch->minPosition = 0;
		ch->minVelocityLimit = 0;
		ch->minAcceleration = 208.334;
		ch->position = parentAdvancedServo->position[ch->phid.index];
		ch->targetPosition = PUNK_DBL;
		ch->velocityLimit = 1900;
		ch->velocity = parentAdvancedServo->velocity[ch->phid.index];
		ch->acceleration = 3800;
		ch->isMoving = parentAdvancedServo->isMoving[ch->phid.index];
		ch->engaged = 0;
		ch->speedRampingState = 1;
		ch->voltage = RCSERVO_VOLTAGE_5V;
		ch->dataInterval = 256;
		ch->maxDataInterval = 60000;
		ch->minDataInterval = 32;
		ch->minDataRate = 0.016666666666666666;
		ch->maxDataRate = 31.25;
		break;
	case PHIDCHUID_1066_RCSERVO_100:
		ch->maxPulseWidthLimit = 2730.666;
		ch->maxPulseWidth = 2450;
		ch->maxPosition = 180;
		ch->maxVelocityLimit = 68266.666;
		ch->maxAcceleration = 3413333.333;
		ch->minPulseWidthLimit = 0.083;
		ch->minPulseWidth = 550;
		ch->minPosition = 0;
		ch->minVelocityLimit = 0;
		ch->minAcceleration = 208.334;
		ch->position = parentAdvancedServo->position[ch->phid.index];
		ch->targetPosition = PUNK_DBL;
		ch->velocityLimit = 1900;
		ch->velocity = parentAdvancedServo->velocity[ch->phid.index];
		ch->acceleration = 3800;
		ch->isMoving = parentAdvancedServo->isMoving[ch->phid.index];
		ch->engaged = 0;
		ch->speedRampingState = 1;
		ch->voltage = RCSERVO_VOLTAGE_5V;
		ch->dataInterval = 256;
		ch->maxDataInterval = 60000;
		ch->minDataInterval = 32;
		ch->minDataRate = 0.016666666666666666;
		ch->maxDataRate = 31.25;
		break;
	case PHIDCHUID_RCC1000_RCSERVO_100:
		ch->maxPulseWidthLimit = 4000;
		ch->maxPulseWidth = 2450;
		ch->maxPosition = 180;
		ch->maxVelocityLimit = 781250;
		ch->maxAcceleration = 3906250;
		ch->minPulseWidthLimit = 0.063;
		ch->minPulseWidth = 550;
		ch->minPosition = 0;
		ch->minVelocityLimit = 0;
		ch->minAcceleration = 156.25;
		ch->position = PUNK_DBL;
		ch->targetPosition = PUNK_DBL;
		ch->velocityLimit = 1900;
		ch->acceleration = 3800;
		ch->isMoving = PUNK_BOOL;
		ch->engaged = 0;
		ch->speedRampingState = 1;
		ch->voltage = RCSERVO_VOLTAGE_5V;
		break;
	case PHIDCHUID_RCC1000_RCSERVO_110:
		ch->maxPulseWidthLimit = 4000;
		ch->maxPulseWidth = 2450;
		ch->maxPosition = 180;
		ch->maxVelocityLimit = 781250;
		ch->maxAcceleration = 3906250;
		ch->minPulseWidthLimit = 0.063;
		ch->minPulseWidth = 550;
		ch->minPosition = 0;
		ch->minVelocityLimit = 0;
		ch->minAcceleration = 156.25;
		ch->position = PUNK_DBL;
		ch->targetPosition = PUNK_DBL;
		ch->velocityLimit = 1900;
		ch->acceleration = 3800;
		ch->isMoving = PUNK_BOOL;
		ch->engaged = 0;
		ch->speedRampingState = 1;
		ch->voltage = RCSERVO_VOLTAGE_5V;
		ch->maxFailsafeTime = 30000;
		ch->minFailsafeTime = 500;
		break;
	default:
		MOS_PANIC("Unsupported Channel");
	}


	return (ret);
}

static PhidgetReturnCode CCONV
_setDefaults(PhidgetChannelHandle phid) {
	PhidgetRCServoHandle ch;
	PhidgetReturnCode ret;

	TESTPTR(phid);

	ch = (PhidgetRCServoHandle)phid;
	ret = EPHIDGET_OK;

	switch (phid->UCD->uid) {
	case PHIDCHUID_1000_RCSERVO_OLD1_200:
		ret = bridgeSendToDevice(phid, BP_SETMAXPULSEWIDTH, NULL, NULL, "%g", ch->maxPulseWidth);
		if (ret != EPHIDGET_OK)
			break;
		ret = bridgeSendToDevice(phid, BP_SETMINPULSEWIDTH, NULL, NULL, "%g", ch->minPulseWidth);
		if (ret != EPHIDGET_OK)
			break;
		break;
	case PHIDCHUID_1000_RCSERVO_OLD2_200:
		ret = bridgeSendToDevice(phid, BP_SETMAXPULSEWIDTH, NULL, NULL, "%g", ch->maxPulseWidth);
		if (ret != EPHIDGET_OK)
			break;
		ret = bridgeSendToDevice(phid, BP_SETMINPULSEWIDTH, NULL, NULL, "%g", ch->minPulseWidth);
		if (ret != EPHIDGET_OK)
			break;
		break;
	case PHIDCHUID_1000_RCSERVO_300:
		ret = bridgeSendToDevice(phid, BP_SETMAXPULSEWIDTH, NULL, NULL, "%g", ch->maxPulseWidth);
		if (ret != EPHIDGET_OK)
			break;
		ret = bridgeSendToDevice(phid, BP_SETMINPULSEWIDTH, NULL, NULL, "%g", ch->minPulseWidth);
		if (ret != EPHIDGET_OK)
			break;
		break;
	case PHIDCHUID_1000_RCSERVO_313:
		ret = bridgeSendToDevice(phid, BP_SETMAXPULSEWIDTH, NULL, NULL, "%g", ch->maxPulseWidth);
		if (ret != EPHIDGET_OK)
			break;
		ret = bridgeSendToDevice(phid, BP_SETMINPULSEWIDTH, NULL, NULL, "%g", ch->minPulseWidth);
		if (ret != EPHIDGET_OK)
			break;
		break;
	case PHIDCHUID_1001_RCSERVO_OLD1_200:
		ret = bridgeSendToDevice(phid, BP_SETMAXPULSEWIDTH, NULL, NULL, "%g", ch->maxPulseWidth);
		if (ret != EPHIDGET_OK)
			break;
		ret = bridgeSendToDevice(phid, BP_SETMINPULSEWIDTH, NULL, NULL, "%g", ch->minPulseWidth);
		if (ret != EPHIDGET_OK)
			break;
		break;
	case PHIDCHUID_1001_RCSERVO_OLD2_200:
		ret = bridgeSendToDevice(phid, BP_SETMAXPULSEWIDTH, NULL, NULL, "%g", ch->maxPulseWidth);
		if (ret != EPHIDGET_OK)
			break;
		ret = bridgeSendToDevice(phid, BP_SETMINPULSEWIDTH, NULL, NULL, "%g", ch->minPulseWidth);
		if (ret != EPHIDGET_OK)
			break;
		break;
	case PHIDCHUID_1001_RCSERVO_313:
		ret = bridgeSendToDevice(phid, BP_SETMAXPULSEWIDTH, NULL, NULL, "%g", ch->maxPulseWidth);
		if (ret != EPHIDGET_OK)
			break;
		ret = bridgeSendToDevice(phid, BP_SETMINPULSEWIDTH, NULL, NULL, "%g", ch->minPulseWidth);
		if (ret != EPHIDGET_OK)
			break;
		break;
	case PHIDCHUID_1001_RCSERVO_400:
		ret = bridgeSendToDevice(phid, BP_SETMAXPULSEWIDTH, NULL, NULL, "%g", ch->maxPulseWidth);
		if (ret != EPHIDGET_OK)
			break;
		ret = bridgeSendToDevice(phid, BP_SETMINPULSEWIDTH, NULL, NULL, "%g", ch->minPulseWidth);
		if (ret != EPHIDGET_OK)
			break;
		break;
	case PHIDCHUID_1061_RCSERVO_100:
		ret = bridgeSendToDevice(phid, BP_SETMAXPULSEWIDTH, NULL, NULL, "%g", ch->maxPulseWidth);
		if (ret != EPHIDGET_OK)
			break;
		ret = bridgeSendToDevice(phid, BP_SETMINPULSEWIDTH, NULL, NULL, "%g", ch->minPulseWidth);
		if (ret != EPHIDGET_OK)
			break;
		ret = bridgeSendToDevice(phid, BP_SETVELOCITYLIMIT, NULL, NULL, "%g", ch->velocityLimit);
		if (ret != EPHIDGET_OK)
			break;
		ret = bridgeSendToDevice(phid, BP_SETACCELERATION, NULL, NULL, "%g", ch->acceleration);
		if (ret != EPHIDGET_OK)
			break;
		ret = bridgeSendToDevice(phid, BP_SETSPEEDRAMPINGSTATE, NULL, NULL, "%d",
		  ch->speedRampingState);
		if (ret != EPHIDGET_OK)
			break;
		ret = bridgeSendToDevice(phid, BP_SETDATAINTERVAL, NULL, NULL, "%u",
		  (uint32_t)round(ch->dataInterval));
		if (ret != EPHIDGET_OK)
			break;
		break;
	case PHIDCHUID_1061_RCSERVO_200:
		ret = bridgeSendToDevice(phid, BP_SETMAXPULSEWIDTH, NULL, NULL, "%g", ch->maxPulseWidth);
		if (ret != EPHIDGET_OK)
			break;
		ret = bridgeSendToDevice(phid, BP_SETMINPULSEWIDTH, NULL, NULL, "%g", ch->minPulseWidth);
		if (ret != EPHIDGET_OK)
			break;
		ret = bridgeSendToDevice(phid, BP_SETVELOCITYLIMIT, NULL, NULL, "%g", ch->velocityLimit);
		if (ret != EPHIDGET_OK)
			break;
		ret = bridgeSendToDevice(phid, BP_SETACCELERATION, NULL, NULL, "%g", ch->acceleration);
		if (ret != EPHIDGET_OK)
			break;
		ret = bridgeSendToDevice(phid, BP_SETSPEEDRAMPINGSTATE, NULL, NULL, "%d",
		  ch->speedRampingState);
		if (ret != EPHIDGET_OK)
			break;
		ret = bridgeSendToDevice(phid, BP_SETDATAINTERVAL, NULL, NULL, "%u",
		  (uint32_t)round(ch->dataInterval));
		if (ret != EPHIDGET_OK)
			break;
		break;
	case PHIDCHUID_1061_RCSERVO_300:
		ret = bridgeSendToDevice(phid, BP_SETMAXPULSEWIDTH, NULL, NULL, "%g", ch->maxPulseWidth);
		if (ret != EPHIDGET_OK)
			break;
		ret = bridgeSendToDevice(phid, BP_SETMINPULSEWIDTH, NULL, NULL, "%g", ch->minPulseWidth);
		if (ret != EPHIDGET_OK)
			break;
		ret = bridgeSendToDevice(phid, BP_SETVELOCITYLIMIT, NULL, NULL, "%g", ch->velocityLimit);
		if (ret != EPHIDGET_OK)
			break;
		ret = bridgeSendToDevice(phid, BP_SETACCELERATION, NULL, NULL, "%g", ch->acceleration);
		if (ret != EPHIDGET_OK)
			break;
		ret = bridgeSendToDevice(phid, BP_SETSPEEDRAMPINGSTATE, NULL, NULL, "%d",
		  ch->speedRampingState);
		if (ret != EPHIDGET_OK)
			break;
		ret = bridgeSendToDevice(phid, BP_SETDATAINTERVAL, NULL, NULL, "%u",
		  (uint32_t)round(ch->dataInterval));
		if (ret != EPHIDGET_OK)
			break;
		break;
	case PHIDCHUID_RCC0004_RCSERVO_400:
		ret = bridgeSendToDevice(phid, BP_SETMAXPULSEWIDTH, NULL, NULL, "%g", ch->maxPulseWidth);
		if (ret != EPHIDGET_OK)
			break;
		ret = bridgeSendToDevice(phid, BP_SETMINPULSEWIDTH, NULL, NULL, "%g", ch->minPulseWidth);
		if (ret != EPHIDGET_OK)
			break;
		ret = bridgeSendToDevice(phid, BP_SETVELOCITYLIMIT, NULL, NULL, "%g", ch->velocityLimit);
		if (ret != EPHIDGET_OK)
			break;
		ret = bridgeSendToDevice(phid, BP_SETACCELERATION, NULL, NULL, "%g", ch->acceleration);
		if (ret != EPHIDGET_OK)
			break;
		ret = bridgeSendToDevice(phid, BP_SETSPEEDRAMPINGSTATE, NULL, NULL, "%d",
		  ch->speedRampingState);
		if (ret != EPHIDGET_OK)
			break;
		ret = bridgeSendToDevice(phid, BP_SETVOLTAGE, NULL, NULL, "%d", ch->voltage);
		if (ret != EPHIDGET_OK)
			break;
		ret = bridgeSendToDevice(phid, BP_SETDATAINTERVAL, NULL, NULL, "%u",
		  (uint32_t)round(ch->dataInterval));
		if (ret != EPHIDGET_OK)
			break;
		break;
	case PHIDCHUID_1066_RCSERVO_100:
		ret = bridgeSendToDevice(phid, BP_SETMAXPULSEWIDTH, NULL, NULL, "%g", ch->maxPulseWidth);
		if (ret != EPHIDGET_OK)
			break;
		ret = bridgeSendToDevice(phid, BP_SETMINPULSEWIDTH, NULL, NULL, "%g", ch->minPulseWidth);
		if (ret != EPHIDGET_OK)
			break;
		ret = bridgeSendToDevice(phid, BP_SETVELOCITYLIMIT, NULL, NULL, "%g", ch->velocityLimit);
		if (ret != EPHIDGET_OK)
			break;
		ret = bridgeSendToDevice(phid, BP_SETACCELERATION, NULL, NULL, "%g", ch->acceleration);
		if (ret != EPHIDGET_OK)
			break;
		ret = bridgeSendToDevice(phid, BP_SETSPEEDRAMPINGSTATE, NULL, NULL, "%d",
		  ch->speedRampingState);
		if (ret != EPHIDGET_OK)
			break;
		ret = bridgeSendToDevice(phid, BP_SETDATAINTERVAL, NULL, NULL, "%u",
		  (uint32_t)round(ch->dataInterval));
		if (ret != EPHIDGET_OK)
			break;
		break;
	case PHIDCHUID_RCC1000_RCSERVO_100:
		break;
	case PHIDCHUID_RCC1000_RCSERVO_110:
		break;
	default:
		MOS_PANIC("Unsupported Channel");
	}

	return (ret);
}

static void CCONV
_fireInitialEvents(PhidgetChannelHandle phid) {

}

static int CCONV
_hasInitialState(PhidgetChannelHandle phid) {

	return (PTRUE);
}

static void CCONV
_free(PhidgetChannelHandle *ch) {

	mos_free(*ch, sizeof (struct _PhidgetRCServo));
}

static PhidgetReturnCode CCONV
_create(PhidgetRCServoHandle *phidp) {

	CHANNELCREATE_BODY(RCServo, PHIDCHCLASS_RCSERVO);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetRCServo_delete(PhidgetRCServoHandle *phidp) {

	return (Phidget_delete((PhidgetHandle *)phidp));
}

API_PRETURN
PhidgetRCServo_enableFailsafe(PhidgetRCServoHandle ch, uint32_t failsafeTime) {

	TESTPTR_PR(ch);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_RCSERVO);
	TESTATTACHED_PR(ch);

	return bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETFAILSAFETIME, NULL, NULL, "%u",
	  failsafeTime);
}

API_PRETURN
PhidgetRCServo_resetFailsafe(PhidgetRCServoHandle ch) {

	TESTPTR_PR(ch);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_RCSERVO);
	TESTATTACHED_PR(ch);

	return bridgeSendToDevice((PhidgetChannelHandle)ch, BP_FAILSAFERESET, NULL, NULL, NULL);
}

API_PRETURN
PhidgetRCServo_setDataInterval(PhidgetRCServoHandle ch, uint32_t dataInterval) {

	TESTPTR_PR(ch);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_RCSERVO);
	TESTATTACHED_PR(ch);

	return (bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETDATAINTERVAL, NULL, NULL, "%u",
	  dataInterval));
}

API_PRETURN
PhidgetRCServo_getDataInterval(PhidgetRCServoHandle ch, uint32_t *dataInterval) {

	TESTPTR_PR(ch);
	TESTPTR_PR(dataInterval);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_RCSERVO);
	TESTATTACHED_PR(ch);

	switch (ch->phid.UCD->uid) {
	case PHIDCHUID_1000_RCSERVO_OLD1_200:
	case PHIDCHUID_1000_RCSERVO_OLD2_200:
	case PHIDCHUID_1000_RCSERVO_300:
	case PHIDCHUID_1000_RCSERVO_313:
	case PHIDCHUID_1001_RCSERVO_OLD1_200:
	case PHIDCHUID_1001_RCSERVO_OLD2_200:
	case PHIDCHUID_1001_RCSERVO_313:
	case PHIDCHUID_1001_RCSERVO_400:
	case PHIDCHUID_RCC1000_RCSERVO_100:
	case PHIDCHUID_RCC1000_RCSERVO_110:
		return (PHID_RETURN(EPHIDGET_UNSUPPORTED));
	default:
		break;
	}

	*dataInterval = (uint32_t)round(ch->dataInterval);
	if (ch->dataInterval == (double)PUNK_DBL)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetRCServo_getMinDataInterval(PhidgetRCServoHandle ch, uint32_t *minDataInterval) {

	TESTPTR_PR(ch);
	TESTPTR_PR(minDataInterval);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_RCSERVO);
	TESTATTACHED_PR(ch);

	switch (ch->phid.UCD->uid) {
	case PHIDCHUID_1000_RCSERVO_OLD1_200:
	case PHIDCHUID_1000_RCSERVO_OLD2_200:
	case PHIDCHUID_1000_RCSERVO_300:
	case PHIDCHUID_1000_RCSERVO_313:
	case PHIDCHUID_1001_RCSERVO_OLD1_200:
	case PHIDCHUID_1001_RCSERVO_OLD2_200:
	case PHIDCHUID_1001_RCSERVO_313:
	case PHIDCHUID_1001_RCSERVO_400:
	case PHIDCHUID_RCC1000_RCSERVO_100:
	case PHIDCHUID_RCC1000_RCSERVO_110:
		return (PHID_RETURN(EPHIDGET_UNSUPPORTED));
	default:
		break;
	}

	*minDataInterval = ch->minDataInterval;
	if (ch->minDataInterval == (uint32_t)PUNK_UINT32)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetRCServo_getMaxDataInterval(PhidgetRCServoHandle ch, uint32_t *maxDataInterval) {

	TESTPTR_PR(ch);
	TESTPTR_PR(maxDataInterval);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_RCSERVO);
	TESTATTACHED_PR(ch);

	switch (ch->phid.UCD->uid) {
	case PHIDCHUID_1000_RCSERVO_OLD1_200:
	case PHIDCHUID_1000_RCSERVO_OLD2_200:
	case PHIDCHUID_1000_RCSERVO_300:
	case PHIDCHUID_1000_RCSERVO_313:
	case PHIDCHUID_1001_RCSERVO_OLD1_200:
	case PHIDCHUID_1001_RCSERVO_OLD2_200:
	case PHIDCHUID_1001_RCSERVO_313:
	case PHIDCHUID_1001_RCSERVO_400:
	case PHIDCHUID_RCC1000_RCSERVO_100:
	case PHIDCHUID_RCC1000_RCSERVO_110:
		return (PHID_RETURN(EPHIDGET_UNSUPPORTED));
	default:
		break;
	}

	*maxDataInterval = ch->maxDataInterval;
	if (ch->maxDataInterval == (uint32_t)PUNK_UINT32)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetRCServo_setDataRate(PhidgetRCServoHandle ch, double dataRate) {

	TESTPTR_PR(ch);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_RCSERVO);
	TESTATTACHED_PR(ch);

	return bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETDATAINTERVAL, NULL, NULL, "%u%g",
	  (uint32_t)round(1000.0 / dataRate), (double)(1000.0 / dataRate));
}

API_PRETURN
PhidgetRCServo_getDataRate(PhidgetRCServoHandle ch, double *dataRate) {

	TESTPTR_PR(ch);
	TESTPTR_PR(dataRate);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_RCSERVO);
	TESTATTACHED_PR(ch);

	switch (ch->phid.UCD->uid) {
	case PHIDCHUID_1000_RCSERVO_OLD1_200:
	case PHIDCHUID_1000_RCSERVO_OLD2_200:
	case PHIDCHUID_1000_RCSERVO_300:
	case PHIDCHUID_1000_RCSERVO_313:
	case PHIDCHUID_1001_RCSERVO_OLD1_200:
	case PHIDCHUID_1001_RCSERVO_OLD2_200:
	case PHIDCHUID_1001_RCSERVO_313:
	case PHIDCHUID_1001_RCSERVO_400:
	case PHIDCHUID_RCC1000_RCSERVO_100:
	case PHIDCHUID_RCC1000_RCSERVO_110:
		return (PHID_RETURN(EPHIDGET_UNSUPPORTED));
	default:
		break;
	}

	*dataRate = round_double((1000.0 / ch->dataInterval), 4);
	if (ch->dataInterval == (double)PUNK_DBL)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetRCServo_getMinDataRate(PhidgetRCServoHandle ch, double *minDataRate) {

	TESTPTR_PR(ch);
	TESTPTR_PR(minDataRate);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_RCSERVO);
	TESTATTACHED_PR(ch);

	switch (ch->phid.UCD->uid) {
	case PHIDCHUID_1000_RCSERVO_OLD1_200:
	case PHIDCHUID_1000_RCSERVO_OLD2_200:
	case PHIDCHUID_1000_RCSERVO_300:
	case PHIDCHUID_1000_RCSERVO_313:
	case PHIDCHUID_1001_RCSERVO_OLD1_200:
	case PHIDCHUID_1001_RCSERVO_OLD2_200:
	case PHIDCHUID_1001_RCSERVO_313:
	case PHIDCHUID_1001_RCSERVO_400:
	case PHIDCHUID_RCC1000_RCSERVO_100:
	case PHIDCHUID_RCC1000_RCSERVO_110:
		return (PHID_RETURN(EPHIDGET_UNSUPPORTED));
	default:
		break;
	}

	*minDataRate = ch->minDataRate;
	if (ch->minDataRate == (double)PUNK_DBL)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetRCServo_getMaxDataRate(PhidgetRCServoHandle ch, double *maxDataRate) {

	TESTPTR_PR(ch);
	TESTPTR_PR(maxDataRate);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_RCSERVO);
	TESTATTACHED_PR(ch);

	switch (ch->phid.UCD->uid) {
	case PHIDCHUID_1000_RCSERVO_OLD1_200:
	case PHIDCHUID_1000_RCSERVO_OLD2_200:
	case PHIDCHUID_1000_RCSERVO_300:
	case PHIDCHUID_1000_RCSERVO_313:
	case PHIDCHUID_1001_RCSERVO_OLD1_200:
	case PHIDCHUID_1001_RCSERVO_OLD2_200:
	case PHIDCHUID_1001_RCSERVO_313:
	case PHIDCHUID_1001_RCSERVO_400:
	case PHIDCHUID_RCC1000_RCSERVO_100:
	case PHIDCHUID_RCC1000_RCSERVO_110:
		return (PHID_RETURN(EPHIDGET_UNSUPPORTED));
	default:
		break;
	}

	*maxDataRate = ch->maxDataRate;
	if (ch->maxDataRate == (double)PUNK_DBL)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetRCServo_setEngaged(PhidgetRCServoHandle ch, int engaged) {

	TESTPTR_PR(ch);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_RCSERVO);
	TESTATTACHED_PR(ch);

	return (bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETENGAGED, NULL, NULL, "%d", engaged));
}

API_PRETURN
PhidgetRCServo_getEngaged(PhidgetRCServoHandle ch, int *engaged) {

	TESTPTR_PR(ch);
	TESTPTR_PR(engaged);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_RCSERVO);
	TESTATTACHED_PR(ch);

	*engaged = ch->engaged;
	if (ch->engaged == (int)PUNK_BOOL)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetRCServo_getMinFailsafeTime(PhidgetRCServoHandle ch, uint32_t *minFailsafeTime) {

	TESTPTR_PR(ch);
	TESTPTR_PR(minFailsafeTime);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_RCSERVO);
	TESTATTACHED_PR(ch);

	switch (ch->phid.UCD->uid) {
	case PHIDCHUID_1000_RCSERVO_OLD1_200:
	case PHIDCHUID_1000_RCSERVO_OLD2_200:
	case PHIDCHUID_1000_RCSERVO_300:
	case PHIDCHUID_1000_RCSERVO_313:
	case PHIDCHUID_1001_RCSERVO_OLD1_200:
	case PHIDCHUID_1001_RCSERVO_OLD2_200:
	case PHIDCHUID_1001_RCSERVO_313:
	case PHIDCHUID_1001_RCSERVO_400:
	case PHIDCHUID_1061_RCSERVO_100:
	case PHIDCHUID_1061_RCSERVO_200:
	case PHIDCHUID_1061_RCSERVO_300:
	case PHIDCHUID_RCC0004_RCSERVO_400:
	case PHIDCHUID_1066_RCSERVO_100:
	case PHIDCHUID_RCC1000_RCSERVO_100:
		return (PHID_RETURN(EPHIDGET_UNSUPPORTED));
	default:
		break;
	}

	*minFailsafeTime = ch->minFailsafeTime;
	if (ch->minFailsafeTime == (uint32_t)PUNK_UINT32)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetRCServo_getMaxFailsafeTime(PhidgetRCServoHandle ch, uint32_t *maxFailsafeTime) {

	TESTPTR_PR(ch);
	TESTPTR_PR(maxFailsafeTime);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_RCSERVO);
	TESTATTACHED_PR(ch);

	switch (ch->phid.UCD->uid) {
	case PHIDCHUID_1000_RCSERVO_OLD1_200:
	case PHIDCHUID_1000_RCSERVO_OLD2_200:
	case PHIDCHUID_1000_RCSERVO_300:
	case PHIDCHUID_1000_RCSERVO_313:
	case PHIDCHUID_1001_RCSERVO_OLD1_200:
	case PHIDCHUID_1001_RCSERVO_OLD2_200:
	case PHIDCHUID_1001_RCSERVO_313:
	case PHIDCHUID_1001_RCSERVO_400:
	case PHIDCHUID_1061_RCSERVO_100:
	case PHIDCHUID_1061_RCSERVO_200:
	case PHIDCHUID_1061_RCSERVO_300:
	case PHIDCHUID_RCC0004_RCSERVO_400:
	case PHIDCHUID_1066_RCSERVO_100:
	case PHIDCHUID_RCC1000_RCSERVO_100:
		return (PHID_RETURN(EPHIDGET_UNSUPPORTED));
	default:
		break;
	}

	*maxFailsafeTime = ch->maxFailsafeTime;
	if (ch->maxFailsafeTime == (uint32_t)PUNK_UINT32)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetRCServo_getIsMoving(PhidgetRCServoHandle ch, int *isMoving) {

	TESTPTR_PR(ch);
	TESTPTR_PR(isMoving);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_RCSERVO);
	TESTATTACHED_PR(ch);

	switch (ch->phid.UCD->uid) {
	case PHIDCHUID_1000_RCSERVO_OLD1_200:
	case PHIDCHUID_1000_RCSERVO_OLD2_200:
	case PHIDCHUID_1000_RCSERVO_300:
	case PHIDCHUID_1000_RCSERVO_313:
	case PHIDCHUID_1001_RCSERVO_OLD1_200:
	case PHIDCHUID_1001_RCSERVO_OLD2_200:
	case PHIDCHUID_1001_RCSERVO_313:
	case PHIDCHUID_1001_RCSERVO_400:
		return (PHID_RETURN(EPHIDGET_UNSUPPORTED));
	default:
		break;
	}

	*isMoving = ch->isMoving;
	if (ch->isMoving == (int)PUNK_BOOL)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetRCServo_getMinPosition(PhidgetRCServoHandle ch, double *minPosition) {

	TESTPTR_PR(ch);
	TESTPTR_PR(minPosition);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_RCSERVO);
	TESTATTACHED_PR(ch);

	*minPosition = ch->minPosition;
	if (ch->minPosition == (double)PUNK_DBL)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetRCServo_getMaxPosition(PhidgetRCServoHandle ch, double *maxPosition) {

	TESTPTR_PR(ch);
	TESTPTR_PR(maxPosition);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_RCSERVO);
	TESTATTACHED_PR(ch);

	*maxPosition = ch->maxPosition;
	if (ch->maxPosition == (double)PUNK_DBL)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetRCServo_setMinPulseWidth(PhidgetRCServoHandle ch, double minPulseWidth) {

	TESTPTR_PR(ch);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_RCSERVO);
	TESTATTACHED_PR(ch);

	return (bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETMINPULSEWIDTH, NULL, NULL, "%g",
	  minPulseWidth));
}

API_PRETURN
PhidgetRCServo_getMinPulseWidth(PhidgetRCServoHandle ch, double *minPulseWidth) {

	TESTPTR_PR(ch);
	TESTPTR_PR(minPulseWidth);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_RCSERVO);
	TESTATTACHED_PR(ch);

	*minPulseWidth = ch->minPulseWidth;
	if (ch->minPulseWidth == (double)PUNK_DBL)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetRCServo_setMaxPulseWidth(PhidgetRCServoHandle ch, double maxPulseWidth) {

	TESTPTR_PR(ch);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_RCSERVO);
	TESTATTACHED_PR(ch);

	return (bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETMAXPULSEWIDTH, NULL, NULL, "%g",
	  maxPulseWidth));
}

API_PRETURN
PhidgetRCServo_getMaxPulseWidth(PhidgetRCServoHandle ch, double *maxPulseWidth) {

	TESTPTR_PR(ch);
	TESTPTR_PR(maxPulseWidth);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_RCSERVO);
	TESTATTACHED_PR(ch);

	*maxPulseWidth = ch->maxPulseWidth;
	if (ch->maxPulseWidth == (double)PUNK_DBL)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetRCServo_getMinPulseWidthLimit(PhidgetRCServoHandle ch, double *minPulseWidthLimit) {

	TESTPTR_PR(ch);
	TESTPTR_PR(minPulseWidthLimit);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_RCSERVO);
	TESTATTACHED_PR(ch);

	*minPulseWidthLimit = ch->minPulseWidthLimit;
	if (ch->minPulseWidthLimit == (double)PUNK_DBL)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetRCServo_getMaxPulseWidthLimit(PhidgetRCServoHandle ch, double *maxPulseWidthLimit) {

	TESTPTR_PR(ch);
	TESTPTR_PR(maxPulseWidthLimit);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_RCSERVO);
	TESTATTACHED_PR(ch);

	*maxPulseWidthLimit = ch->maxPulseWidthLimit;
	if (ch->maxPulseWidthLimit == (double)PUNK_DBL)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetRCServo_setSpeedRampingState(PhidgetRCServoHandle ch, int speedRampingState) {

	TESTPTR_PR(ch);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_RCSERVO);
	TESTATTACHED_PR(ch);

	return (bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETSPEEDRAMPINGSTATE, NULL, NULL, "%d",
	  speedRampingState));
}

API_PRETURN
PhidgetRCServo_getSpeedRampingState(PhidgetRCServoHandle ch, int *speedRampingState) {

	TESTPTR_PR(ch);
	TESTPTR_PR(speedRampingState);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_RCSERVO);
	TESTATTACHED_PR(ch);

	*speedRampingState = ch->speedRampingState;
	if (ch->speedRampingState == (int)PUNK_BOOL)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetRCServo_setTorque(PhidgetRCServoHandle ch, double torque) {

	TESTPTR_PR(ch);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_RCSERVO);
	TESTATTACHED_PR(ch);

	return (bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETDUTYCYCLE, NULL, NULL, "%g", torque));
}

API_PRETURN
PhidgetRCServo_getTorque(PhidgetRCServoHandle ch, double *torque) {

	TESTPTR_PR(ch);
	TESTPTR_PR(torque);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_RCSERVO);
	TESTATTACHED_PR(ch);

	switch (ch->phid.UCD->uid) {
	case PHIDCHUID_1000_RCSERVO_OLD1_200:
	case PHIDCHUID_1000_RCSERVO_OLD2_200:
	case PHIDCHUID_1000_RCSERVO_300:
	case PHIDCHUID_1000_RCSERVO_313:
	case PHIDCHUID_1001_RCSERVO_OLD1_200:
	case PHIDCHUID_1001_RCSERVO_OLD2_200:
	case PHIDCHUID_1001_RCSERVO_313:
	case PHIDCHUID_1001_RCSERVO_400:
	case PHIDCHUID_1061_RCSERVO_100:
	case PHIDCHUID_1061_RCSERVO_200:
	case PHIDCHUID_1061_RCSERVO_300:
	case PHIDCHUID_RCC0004_RCSERVO_400:
	case PHIDCHUID_1066_RCSERVO_100:
	case PHIDCHUID_RCC1000_RCSERVO_100:
	case PHIDCHUID_RCC1000_RCSERVO_110:
		return (PHID_RETURN(EPHIDGET_UNSUPPORTED));
	default:
		break;
	}

	*torque = ch->torque;
	if (ch->torque == (double)PUNK_DBL)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetRCServo_getMinTorque(PhidgetRCServoHandle ch, double *minTorque) {

	TESTPTR_PR(ch);
	TESTPTR_PR(minTorque);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_RCSERVO);
	TESTATTACHED_PR(ch);

	switch (ch->phid.UCD->uid) {
	case PHIDCHUID_1000_RCSERVO_OLD1_200:
	case PHIDCHUID_1000_RCSERVO_OLD2_200:
	case PHIDCHUID_1000_RCSERVO_300:
	case PHIDCHUID_1000_RCSERVO_313:
	case PHIDCHUID_1001_RCSERVO_OLD1_200:
	case PHIDCHUID_1001_RCSERVO_OLD2_200:
	case PHIDCHUID_1001_RCSERVO_313:
	case PHIDCHUID_1001_RCSERVO_400:
	case PHIDCHUID_1061_RCSERVO_100:
	case PHIDCHUID_1061_RCSERVO_200:
	case PHIDCHUID_1061_RCSERVO_300:
	case PHIDCHUID_RCC0004_RCSERVO_400:
	case PHIDCHUID_1066_RCSERVO_100:
	case PHIDCHUID_RCC1000_RCSERVO_100:
	case PHIDCHUID_RCC1000_RCSERVO_110:
		return (PHID_RETURN(EPHIDGET_UNSUPPORTED));
	default:
		break;
	}

	*minTorque = ch->minTorque;
	if (ch->minTorque == (double)PUNK_DBL)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetRCServo_getMaxTorque(PhidgetRCServoHandle ch, double *maxTorque) {

	TESTPTR_PR(ch);
	TESTPTR_PR(maxTorque);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_RCSERVO);
	TESTATTACHED_PR(ch);

	switch (ch->phid.UCD->uid) {
	case PHIDCHUID_1000_RCSERVO_OLD1_200:
	case PHIDCHUID_1000_RCSERVO_OLD2_200:
	case PHIDCHUID_1000_RCSERVO_300:
	case PHIDCHUID_1000_RCSERVO_313:
	case PHIDCHUID_1001_RCSERVO_OLD1_200:
	case PHIDCHUID_1001_RCSERVO_OLD2_200:
	case PHIDCHUID_1001_RCSERVO_313:
	case PHIDCHUID_1001_RCSERVO_400:
	case PHIDCHUID_1061_RCSERVO_100:
	case PHIDCHUID_1061_RCSERVO_200:
	case PHIDCHUID_1061_RCSERVO_300:
	case PHIDCHUID_RCC0004_RCSERVO_400:
	case PHIDCHUID_1066_RCSERVO_100:
	case PHIDCHUID_RCC1000_RCSERVO_100:
	case PHIDCHUID_RCC1000_RCSERVO_110:
		return (PHID_RETURN(EPHIDGET_UNSUPPORTED));
	default:
		break;
	}

	*maxTorque = ch->maxTorque;
	if (ch->maxTorque == (double)PUNK_DBL)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetRCServo_setVoltage(PhidgetRCServoHandle ch, PhidgetRCServo_Voltage voltage) {

	TESTPTR_PR(ch);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_RCSERVO);
	TESTATTACHED_PR(ch);

	return (bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETVOLTAGE, NULL, NULL, "%d", voltage));
}

API_PRETURN
PhidgetRCServo_getVoltage(PhidgetRCServoHandle ch, PhidgetRCServo_Voltage *voltage) {

	TESTPTR_PR(ch);
	TESTPTR_PR(voltage);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_RCSERVO);
	TESTATTACHED_PR(ch);

	*voltage = ch->voltage;
	if (ch->voltage == (PhidgetRCServo_Voltage)PUNK_ENUM)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetRCServo_setOnPositionChangeHandler(PhidgetRCServoHandle ch,
  PhidgetRCServo_OnPositionChangeCallback fptr, void *ctx) {

	TESTPTR_PR(ch);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_RCSERVO);

	ch->PositionChange = fptr;
	ch->PositionChangeCtx = ctx;

	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetRCServo_setOnTargetPositionReachedHandler(PhidgetRCServoHandle ch,
  PhidgetRCServo_OnTargetPositionReachedCallback fptr, void *ctx) {

	TESTPTR_PR(ch);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_RCSERVO);

	ch->TargetPositionReached = fptr;
	ch->TargetPositionReachedCtx = ctx;

	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetRCServo_setOnVelocityChangeHandler(PhidgetRCServoHandle ch,
  PhidgetRCServo_OnVelocityChangeCallback fptr, void *ctx) {

	TESTPTR_PR(ch);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_RCSERVO);

	ch->VelocityChange = fptr;
	ch->VelocityChangeCtx = ctx;

	return (EPHIDGET_OK);
}
