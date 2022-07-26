/* Generated: Mon May 30 2022 10:16:30 GMT-0600 (Mountain Daylight Time) */

#include "device/stepperdevice.h"
static void CCONV PhidgetStepper_errorHandler(PhidgetChannelHandle ch, Phidget_ErrorEventCode code);
static void CCONV PhidgetStepper_free(PhidgetChannelHandle *ch);
static PhidgetReturnCode CCONV PhidgetStepper_bridgeInput(PhidgetChannelHandle phid, BridgePacket *bp);
static PhidgetReturnCode CCONV PhidgetStepper_setStatus(PhidgetChannelHandle phid, BridgePacket *bp);
static PhidgetReturnCode CCONV PhidgetStepper_getStatus(PhidgetChannelHandle phid, BridgePacket **bp);
static PhidgetReturnCode CCONV PhidgetStepper_initAfterOpen(PhidgetChannelHandle phid);
static PhidgetReturnCode CCONV PhidgetStepper_setDefaults(PhidgetChannelHandle phid);
static void CCONV PhidgetStepper_fireInitialEvents(PhidgetChannelHandle phid);
static int CCONV PhidgetStepper_hasInitialState(PhidgetChannelHandle phid);

struct _PhidgetStepper {
	struct _PhidgetChannel phid;
	int64_t positionOffset;
	double acceleration;
	double minAcceleration;
	double maxAcceleration;
	PhidgetStepper_ControlMode controlMode;
	double currentLimit;
	double minCurrentLimit;
	double maxCurrentLimit;
	double dataInterval;
	uint32_t minDataInterval;
	uint32_t maxDataInterval;
	double minDataRate;
	double maxDataRate;
	int engaged;
	uint32_t minFailsafeTime;
	uint32_t maxFailsafeTime;
	double holdingCurrentLimit;
	int isMoving;
	int64_t position;
	int64_t minPosition;
	int64_t maxPosition;
	double rescaleFactor;
	int64_t targetPosition;
	double velocity;
	double velocityLimit;
	double minVelocityLimit;
	double maxVelocityLimit;
	PhidgetStepper_OnPositionChangeCallback PositionChange;
	void *PositionChangeCtx;
	PhidgetStepper_OnStoppedCallback Stopped;
	void *StoppedCtx;
	PhidgetStepper_OnVelocityChangeCallback VelocityChange;
	void *VelocityChangeCtx;
};

static PhidgetReturnCode CCONV
_setStatus(PhidgetChannelHandle phid, BridgePacket *bp) {
	PhidgetStepperHandle ch;
	int version;

	ch = (PhidgetStepperHandle)phid;

	version = getBridgePacketUInt32ByName(bp, "_class_version_");
	if (version != 4) {
		loginfo("%"PRIphid": server/client class version mismatch: %d != 4 - functionality may be limited.", phid, version);
	}

	if(version >= 0)
		ch->positionOffset = getBridgePacketInt64ByName(bp, "positionOffset");
	if (version >= 0)
		ch->acceleration = getBridgePacketDoubleByName(bp, "acceleration");
	if (version >= 0)
		ch->minAcceleration = getBridgePacketDoubleByName(bp, "minAcceleration");
	if (version >= 0)
		ch->maxAcceleration = getBridgePacketDoubleByName(bp, "maxAcceleration");
	if (version >= 0)
		ch->controlMode = getBridgePacketInt32ByName(bp, "controlMode");
	if (version >= 0)
		ch->currentLimit = getBridgePacketDoubleByName(bp, "currentLimit");
	if (version >= 0)
		ch->minCurrentLimit = getBridgePacketDoubleByName(bp, "minCurrentLimit");
	if (version >= 0)
		ch->maxCurrentLimit = getBridgePacketDoubleByName(bp, "maxCurrentLimit");
	if (version >= 0)
		ch->minDataInterval = getBridgePacketUInt32ByName(bp, "minDataInterval");
	if (version >= 0)
		ch->maxDataInterval = getBridgePacketUInt32ByName(bp, "maxDataInterval");
	if (version >= 4)
		ch->dataInterval = getBridgePacketDoubleByName(bp, "dataIntervalDbl");
	else if(version >= 0)
		ch->dataInterval = (double)getBridgePacketUInt32ByName(bp, "dataInterval");
	if (version >= 4)
		ch->minDataRate = getBridgePacketDoubleByName(bp, "minDataRate");
	else if(version >= 0)
		ch->minDataRate = (double)(1000.0 / getBridgePacketUInt32ByName(bp, "maxDataInterval"));
	if (version >= 4)
		ch->maxDataRate = getBridgePacketDoubleByName(bp, "maxDataRate");
	else if(version >= 0)
		ch->maxDataRate = (double)(1000.0 / getBridgePacketUInt32ByName(bp, "minDataInterval"));
	if (version >= 0)
		ch->engaged = getBridgePacketInt32ByName(bp, "engaged");
	if (version >= 3)
		ch->minFailsafeTime = getBridgePacketUInt32ByName(bp, "minFailsafeTime");
	if (version >= 3)
		ch->maxFailsafeTime = getBridgePacketUInt32ByName(bp, "maxFailsafeTime");
	if (version >= 0)
		ch->holdingCurrentLimit = getBridgePacketDoubleByName(bp, "holdingCurrentLimit");
	if (version >= 2)
		ch->isMoving = getBridgePacketInt32ByName(bp, "isMoving");
	if (version >= 0)
		ch->position = getBridgePacketInt64ByName(bp, "position");
	if (version >= 0)
		ch->minPosition = getBridgePacketInt64ByName(bp, "minPosition");
	if (version >= 0)
		ch->maxPosition = getBridgePacketInt64ByName(bp, "maxPosition");
	if (version >= 0)
		ch->rescaleFactor = getBridgePacketDoubleByName(bp, "rescaleFactor");
	if (version >= 0)
		ch->targetPosition = getBridgePacketInt64ByName(bp, "targetPosition");
	if (version >= 0)
		ch->velocity = getBridgePacketDoubleByName(bp, "velocity");
	if (version >= 0)
		ch->velocityLimit = getBridgePacketDoubleByName(bp, "velocityLimit");
	if (version >= 0)
		ch->minVelocityLimit = getBridgePacketDoubleByName(bp, "minVelocityLimit");
	if (version >= 0)
		ch->maxVelocityLimit = getBridgePacketDoubleByName(bp, "maxVelocityLimit");

	return (EPHIDGET_OK);
}

static PhidgetReturnCode CCONV
_getStatus(PhidgetChannelHandle phid, BridgePacket **bp) {
	PhidgetStepperHandle ch;

	ch = (PhidgetStepperHandle)phid;

	return (createBridgePacket(bp, 0, "_class_version_=%u"
	  ",positionOffset=%l"
	  ",acceleration=%g"
	  ",minAcceleration=%g"
	  ",maxAcceleration=%g"
	  ",controlMode=%d"
	  ",currentLimit=%g"
	  ",minCurrentLimit=%g"
	  ",maxCurrentLimit=%g"
	  ",dataInterval=%u"
	  ",minDataInterval=%u"
	  ",maxDataInterval=%u"
	  ",dataIntervalDbl=%g"
	  ",minDataRate=%g"
	  ",maxDataRate=%g"
	  ",engaged=%d"
	  ",minFailsafeTime=%u"
	  ",maxFailsafeTime=%u"
	  ",holdingCurrentLimit=%g"
	  ",isMoving=%d"
	  ",position=%l"
	  ",minPosition=%l"
	  ",maxPosition=%l"
	  ",rescaleFactor=%g"
	  ",targetPosition=%l"
	  ",velocity=%g"
	  ",velocityLimit=%g"
	  ",minVelocityLimit=%g"
	  ",maxVelocityLimit=%g"
	  ,4 /* class version */
	  ,ch->positionOffset
	  ,ch->acceleration
	  ,ch->minAcceleration
	  ,ch->maxAcceleration
	  ,ch->controlMode
	  ,ch->currentLimit
	  ,ch->minCurrentLimit
	  ,ch->maxCurrentLimit
	  ,(uint32_t)round(ch->dataInterval)
	  ,ch->minDataInterval
	  ,ch->maxDataInterval
	  ,ch->dataInterval
	  ,ch->minDataRate
	  ,ch->maxDataRate
	  ,ch->engaged
	  ,ch->minFailsafeTime
	  ,ch->maxFailsafeTime
	  ,ch->holdingCurrentLimit
	  ,ch->isMoving
	  ,ch->position
	  ,ch->minPosition
	  ,ch->maxPosition
	  ,ch->rescaleFactor
	  ,ch->targetPosition
	  ,ch->velocity
	  ,ch->velocityLimit
	  ,ch->minVelocityLimit
	  ,ch->maxVelocityLimit
	));
}

static PhidgetReturnCode CCONV
_bridgeInput(PhidgetChannelHandle phid, BridgePacket *bp) {
	PhidgetStepperHandle ch;
	PhidgetReturnCode res;

	ch = (PhidgetStepperHandle)phid;
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
	case BP_SETCONTROLMODE:
		if (!supportedControlMode(phid, (PhidgetStepper_ControlMode)getBridgePacketInt32(bp, 0)))
			return (MOS_ERROR(bp->iop, EPHIDGET_INVALIDARG,
			  "Specified ControlMode is unsupported by this device."));
		res = DEVBRIDGEINPUT(phid, bp);
		if (res != EPHIDGET_OK)
			break;
		ch->controlMode = getBridgePacketInt32(bp, 0);
		if (bridgePacketIsFromNet(bp)) {
			FIRE_PROPERTYCHANGE(ch, "ControlMode");
		}
		break;
	case BP_SETCURRENTLIMIT:
		TESTRANGE_IOP(bp->iop, "%lf", getBridgePacketDouble(bp, 0), ch->minCurrentLimit,
		  ch->maxCurrentLimit);
		res = DEVBRIDGEINPUT(phid, bp);
		if (res != EPHIDGET_OK)
			break;
		ch->currentLimit = getBridgePacketDouble(bp, 0);
		if (bridgePacketIsFromNet(bp)) {
			FIRE_PROPERTYCHANGE(ch, "CurrentLimit");
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
	case BP_SETHOLDINGCURRENTLIMIT:
		res = DEVBRIDGEINPUT(phid, bp);
		if (res != EPHIDGET_OK)
			break;
		ch->holdingCurrentLimit = getBridgePacketDouble(bp, 0);
		if (bridgePacketIsFromNet(bp)) {
			FIRE_PROPERTYCHANGE(ch, "HoldingCurrentLimit");
		}
		break;
	case BP_SETTARGETPOSITION:
		res = DEVBRIDGEINPUT(phid, bp);
		if (res != EPHIDGET_OK)
			break;
		ch->targetPosition = getBridgePacketInt64(bp, 0);
		if (bridgePacketIsFromNet(bp)) {
			FIRE_PROPERTYCHANGE(ch, "TargetPosition");
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
	case BP_STOPPED:
		FIRECH0(ch, Stopped);
		break;
	default:
		logerr("%"PRIphid": unsupported bridge packet:0x%x", phid, bp->vpkt);
		res = EPHIDGET_UNSUPPORTED;
	}

	return (res);
}

static PhidgetReturnCode CCONV
_initAfterOpen(PhidgetChannelHandle phid) {
	PhidgetStepperDeviceHandle parentStepper;
	PhidgetStepperHandle ch;
	PhidgetReturnCode ret;

	TESTPTR(phid);
	ch = (PhidgetStepperHandle)phid;

	ret = EPHIDGET_OK;

	parentStepper = (PhidgetStepperDeviceHandle)phid->parent;

	switch (phid->UCD->uid) {
	case PHIDCHUID_1062_STEPPER_100:
		ch->dataInterval = 256;
		ch->minDataRate = 0.016666666666666666;
		ch->maxDataRate = 62.5;
		ch->minDataInterval = 16;
		ch->maxDataInterval = 60000;
		ch->maxPosition = 549755813887;
		ch->maxVelocityLimit = 383.25;
		ch->maxAcceleration = 8859.375;
		ch->minPosition = -549755813887;
		ch->minVelocityLimit = 0;
		ch->minAcceleration = 140.625;
		ch->controlMode = CONTROL_MODE_STEP;
		ch->position = parentStepper->position[ch->phid.index];
		ch->targetPosition = 0;
		ch->velocityLimit = 200;
		ch->velocity = parentStepper->velocity[ch->phid.index];
		ch->acceleration = 200;
		ch->isMoving = parentStepper->isMoving[ch->phid.index];
		ch->engaged = 0;
		ch->rescaleFactor = 1;
		break;
	case PHIDCHUID_1063_STEPPER_100:
		ch->dataInterval = 256;
		ch->minDataRate = 0.016666666666666666;
		ch->maxDataRate = 125;
		ch->minDataInterval = 8;
		ch->maxDataInterval = 60000;
		ch->maxPosition = 549755813887;
		ch->maxVelocityLimit = 32768;
		ch->maxAcceleration = 1020000;
		ch->maxCurrentLimit = 2.492;
		ch->minPosition = -549755813887;
		ch->minVelocityLimit = 0;
		ch->minAcceleration = 4000;
		ch->minCurrentLimit = 0.0542;
		ch->controlMode = CONTROL_MODE_STEP;
		ch->position = parentStepper->position[ch->phid.index];
		ch->targetPosition = 0;
		ch->velocityLimit = 10000;
		ch->velocity = parentStepper->velocity[ch->phid.index];
		ch->acceleration = 10000;
		ch->currentLimit = 1;
		ch->isMoving = parentStepper->isMoving[ch->phid.index];
		ch->engaged = 0;
		ch->rescaleFactor = 1;
		break;
	case PHIDCHUID_1067_STEPPER_200:
		ch->dataInterval = 256;
		ch->minDataRate = 0.016666666666666666;
		ch->maxDataRate = 125;
		ch->minDataInterval = 8;
		ch->maxDataInterval = 60000;
		ch->maxPosition = 1000000000000000;
		ch->maxVelocityLimit = 250000;
		ch->maxAcceleration = 10000000;
		ch->maxCurrentLimit = 4;
		ch->minPosition = -1000000000000000;
		ch->minVelocityLimit = 0;
		ch->minAcceleration = 2;
		ch->minCurrentLimit = 0;
		ch->controlMode = CONTROL_MODE_STEP;
		ch->position = parentStepper->position[ch->phid.index];
		ch->targetPosition = 0;
		ch->velocityLimit = 10000;
		ch->velocity = parentStepper->velocity[ch->phid.index];
		ch->acceleration = 10000;
		ch->currentLimit = 1;
		ch->isMoving = parentStepper->isMoving[ch->phid.index];
		ch->engaged = 0;
		ch->rescaleFactor = 1;
		break;
	case PHIDCHUID_1067_STEPPER_300:
		ch->dataInterval = 256;
		ch->minDataRate = 0.016666666666666666;
		ch->maxDataRate = 125;
		ch->minDataInterval = 8;
		ch->maxDataInterval = 60000;
		ch->maxPosition = 1000000000000000;
		ch->maxVelocityLimit = 250000;
		ch->maxAcceleration = 10000000;
		ch->maxCurrentLimit = 4;
		ch->minPosition = -1000000000000000;
		ch->minVelocityLimit = 0;
		ch->minAcceleration = 2;
		ch->minCurrentLimit = 0;
		ch->controlMode = CONTROL_MODE_STEP;
		ch->position = parentStepper->position[ch->phid.index];
		ch->targetPosition = 0;
		ch->velocityLimit = 10000;
		ch->velocity = parentStepper->velocity[ch->phid.index];
		ch->acceleration = 10000;
		ch->currentLimit = 1;
		ch->isMoving = parentStepper->isMoving[ch->phid.index];
		ch->engaged = 0;
		ch->rescaleFactor = 1;
		break;
	case PHIDCHUID_STC1000_STEPPER_100:
		ch->dataInterval = 250;
		ch->minDataRate = 0.016666666666666666;
		ch->maxDataRate = 10;
		ch->holdingCurrentLimit = PUNK_DBL;
		ch->minDataInterval = 100;
		ch->maxDataInterval = 60000;
		ch->maxPosition = 1000000000000000;
		ch->maxVelocityLimit = 115000;
		ch->maxAcceleration = 10000000;
		ch->maxCurrentLimit = 4;
		ch->minPosition = -1000000000000000;
		ch->minVelocityLimit = 0;
		ch->minAcceleration = 2;
		ch->minCurrentLimit = 0;
		ch->controlMode = CONTROL_MODE_STEP;
		ch->position = PUNK_INT64;
		ch->targetPosition = 0;
		ch->velocityLimit = 10000;
		ch->velocity = PUNK_DBL;
		ch->acceleration = 10000;
		ch->currentLimit = 1;
		ch->isMoving = PUNK_BOOL;
		ch->engaged = 0;
		ch->rescaleFactor = 1;
		break;
	case PHIDCHUID_STC1000_STEPPER_110:
		ch->dataInterval = 250;
		ch->minDataRate = 0.016666666666666666;
		ch->maxDataRate = 10;
		ch->holdingCurrentLimit = PUNK_DBL;
		ch->minDataInterval = 100;
		ch->maxDataInterval = 60000;
		ch->maxPosition = 1000000000000000;
		ch->maxVelocityLimit = 115000;
		ch->maxAcceleration = 10000000;
		ch->maxCurrentLimit = 4;
		ch->minPosition = -1000000000000000;
		ch->minVelocityLimit = 0;
		ch->minAcceleration = 2;
		ch->minCurrentLimit = 0;
		ch->controlMode = CONTROL_MODE_STEP;
		ch->position = PUNK_INT64;
		ch->targetPosition = 0;
		ch->velocityLimit = 10000;
		ch->velocity = PUNK_DBL;
		ch->acceleration = 10000;
		ch->currentLimit = 1;
		ch->isMoving = PUNK_BOOL;
		ch->engaged = 0;
		ch->rescaleFactor = 1;
		ch->maxFailsafeTime = 30000;
		ch->minFailsafeTime = 500;
		break;
	case PHIDCHUID_STC1001_STEPPER_100:
		ch->dataInterval = 250;
		ch->minDataRate = 0.016666666666666666;
		ch->maxDataRate = 10;
		ch->holdingCurrentLimit = PUNK_DBL;
		ch->minDataInterval = 100;
		ch->maxDataInterval = 60000;
		ch->maxPosition = 1000000000000000;
		ch->maxVelocityLimit = 115000;
		ch->maxAcceleration = 10000000;
		ch->maxCurrentLimit = 2.5;
		ch->minPosition = -1000000000000000;
		ch->minVelocityLimit = 0;
		ch->minAcceleration = 2;
		ch->minCurrentLimit = 0;
		ch->controlMode = CONTROL_MODE_STEP;
		ch->position = PUNK_INT64;
		ch->targetPosition = 0;
		ch->velocityLimit = 10000;
		ch->velocity = PUNK_DBL;
		ch->acceleration = 10000;
		ch->currentLimit = 1;
		ch->isMoving = PUNK_BOOL;
		ch->engaged = 0;
		ch->rescaleFactor = 1;
		break;
	case PHIDCHUID_STC1001_STEPPER_110:
		ch->dataInterval = 250;
		ch->minDataRate = 0.016666666666666666;
		ch->maxDataRate = 10;
		ch->holdingCurrentLimit = PUNK_DBL;
		ch->minDataInterval = 100;
		ch->maxDataInterval = 60000;
		ch->maxPosition = 1000000000000000;
		ch->maxVelocityLimit = 115000;
		ch->maxAcceleration = 10000000;
		ch->maxCurrentLimit = 2.5;
		ch->minPosition = -1000000000000000;
		ch->minVelocityLimit = 0;
		ch->minAcceleration = 2;
		ch->minCurrentLimit = 0;
		ch->controlMode = CONTROL_MODE_STEP;
		ch->position = PUNK_INT64;
		ch->targetPosition = 0;
		ch->velocityLimit = 10000;
		ch->velocity = PUNK_DBL;
		ch->acceleration = 10000;
		ch->currentLimit = 1;
		ch->isMoving = PUNK_BOOL;
		ch->engaged = 0;
		ch->rescaleFactor = 1;
		ch->maxFailsafeTime = 30000;
		ch->minFailsafeTime = 500;
		break;
	case PHIDCHUID_STC1002_STEPPER_100:
		ch->dataInterval = 250;
		ch->minDataRate = 0.016666666666666666;
		ch->maxDataRate = 10;
		ch->holdingCurrentLimit = PUNK_DBL;
		ch->minDataInterval = 100;
		ch->maxDataInterval = 60000;
		ch->maxPosition = 1000000000000000;
		ch->maxVelocityLimit = 115000;
		ch->maxAcceleration = 10000000;
		ch->maxCurrentLimit = 8;
		ch->minPosition = -1000000000000000;
		ch->minVelocityLimit = 0;
		ch->minAcceleration = 2;
		ch->minCurrentLimit = 0;
		ch->controlMode = CONTROL_MODE_STEP;
		ch->position = PUNK_INT64;
		ch->targetPosition = 0;
		ch->velocityLimit = 10000;
		ch->velocity = PUNK_DBL;
		ch->acceleration = 10000;
		ch->currentLimit = 1;
		ch->isMoving = PUNK_BOOL;
		ch->engaged = 0;
		ch->rescaleFactor = 1;
		break;
	case PHIDCHUID_STC1002_STEPPER_110:
		ch->dataInterval = 250;
		ch->minDataRate = 0.016666666666666666;
		ch->maxDataRate = 10;
		ch->holdingCurrentLimit = PUNK_DBL;
		ch->minDataInterval = 100;
		ch->maxDataInterval = 60000;
		ch->maxPosition = 1000000000000000;
		ch->maxVelocityLimit = 115000;
		ch->maxAcceleration = 10000000;
		ch->maxCurrentLimit = 8;
		ch->minPosition = -1000000000000000;
		ch->minVelocityLimit = 0;
		ch->minAcceleration = 2;
		ch->minCurrentLimit = 0;
		ch->controlMode = CONTROL_MODE_STEP;
		ch->position = PUNK_INT64;
		ch->targetPosition = 0;
		ch->velocityLimit = 10000;
		ch->velocity = PUNK_DBL;
		ch->acceleration = 10000;
		ch->currentLimit = 1;
		ch->isMoving = PUNK_BOOL;
		ch->engaged = 0;
		ch->rescaleFactor = 1;
		ch->maxFailsafeTime = 30000;
		ch->minFailsafeTime = 500;
		break;
	case PHIDCHUID_STC1003_STEPPER_100:
		ch->dataInterval = 250;
		ch->minDataRate = 0.016666666666666666;
		ch->maxDataRate = 10;
		ch->holdingCurrentLimit = PUNK_DBL;
		ch->minDataInterval = 100;
		ch->maxDataInterval = 60000;
		ch->maxPosition = 1000000000000000;
		ch->maxVelocityLimit = 115000;
		ch->maxAcceleration = 10000000;
		ch->maxCurrentLimit = 4;
		ch->minPosition = -1000000000000000;
		ch->minVelocityLimit = 0;
		ch->minAcceleration = 2;
		ch->minCurrentLimit = 0;
		ch->controlMode = CONTROL_MODE_STEP;
		ch->position = PUNK_INT64;
		ch->targetPosition = 0;
		ch->velocityLimit = 10000;
		ch->velocity = PUNK_DBL;
		ch->acceleration = 10000;
		ch->currentLimit = 1;
		ch->isMoving = PUNK_BOOL;
		ch->engaged = 0;
		ch->rescaleFactor = 1;
		break;
	case PHIDCHUID_STC1003_STEPPER_110:
		ch->dataInterval = 250;
		ch->minDataRate = 0.016666666666666666;
		ch->maxDataRate = 10;
		ch->holdingCurrentLimit = PUNK_DBL;
		ch->minDataInterval = 100;
		ch->maxDataInterval = 60000;
		ch->maxPosition = 1000000000000000;
		ch->maxVelocityLimit = 115000;
		ch->maxAcceleration = 10000000;
		ch->maxCurrentLimit = 4;
		ch->minPosition = -1000000000000000;
		ch->minVelocityLimit = 0;
		ch->minAcceleration = 2;
		ch->minCurrentLimit = 0;
		ch->controlMode = CONTROL_MODE_STEP;
		ch->position = PUNK_INT64;
		ch->targetPosition = 0;
		ch->velocityLimit = 10000;
		ch->velocity = PUNK_DBL;
		ch->acceleration = 10000;
		ch->currentLimit = 1;
		ch->isMoving = PUNK_BOOL;
		ch->engaged = 0;
		ch->rescaleFactor = 1;
		ch->maxFailsafeTime = 30000;
		ch->minFailsafeTime = 500;
		break;
	case PHIDCHUID_STC1003_STEPPER_200:
		ch->dataInterval = 250;
		ch->minDataRate = 0.016666666666666666;
		ch->maxDataRate = 10;
		ch->holdingCurrentLimit = PUNK_DBL;
		ch->minDataInterval = 100;
		ch->maxDataInterval = 60000;
		ch->maxPosition = 1000000000000000;
		ch->maxVelocityLimit = 115000;
		ch->maxAcceleration = 10000000;
		ch->maxCurrentLimit = 4;
		ch->minPosition = -1000000000000000;
		ch->minVelocityLimit = 0;
		ch->minAcceleration = 2;
		ch->minCurrentLimit = 0;
		ch->controlMode = CONTROL_MODE_STEP;
		ch->position = PUNK_INT64;
		ch->targetPosition = 0;
		ch->velocityLimit = 10000;
		ch->velocity = PUNK_DBL;
		ch->acceleration = 10000;
		ch->currentLimit = 1;
		ch->isMoving = PUNK_BOOL;
		ch->engaged = 0;
		ch->rescaleFactor = 1;
		ch->maxFailsafeTime = 30000;
		ch->minFailsafeTime = 500;
		break;
	default:
		MOS_PANIC("Unsupported Channel");
	}

	ch->positionOffset = 0;

	return (ret);
}

static PhidgetReturnCode CCONV
_setDefaults(PhidgetChannelHandle phid) {
	PhidgetStepperHandle ch;
	PhidgetReturnCode ret;

	TESTPTR(phid);

	ch = (PhidgetStepperHandle)phid;
	ret = EPHIDGET_OK;

	switch (phid->UCD->uid) {
	case PHIDCHUID_1062_STEPPER_100:
		ret = bridgeSendToDevice(phid, BP_SETDATAINTERVAL, NULL, NULL, "%u",
		  (uint32_t)round(ch->dataInterval));
		if (ret != EPHIDGET_OK)
			break;
		ret = bridgeSendToDevice(phid, BP_SETCONTROLMODE, NULL, NULL, "%d", ch->controlMode);
		if (ret != EPHIDGET_OK)
			break;
		ret = bridgeSendToDevice(phid, BP_SETVELOCITYLIMIT, NULL, NULL, "%g", ch->velocityLimit);
		if (ret != EPHIDGET_OK)
			break;
		ret = bridgeSendToDevice(phid, BP_SETACCELERATION, NULL, NULL, "%g", ch->acceleration);
		if (ret != EPHIDGET_OK)
			break;
		break;
	case PHIDCHUID_1063_STEPPER_100:
		ret = bridgeSendToDevice(phid, BP_SETDATAINTERVAL, NULL, NULL, "%u",
		  (uint32_t)round(ch->dataInterval));
		if (ret != EPHIDGET_OK)
			break;
		ret = bridgeSendToDevice(phid, BP_SETCONTROLMODE, NULL, NULL, "%d", ch->controlMode);
		if (ret != EPHIDGET_OK)
			break;
		ret = bridgeSendToDevice(phid, BP_SETVELOCITYLIMIT, NULL, NULL, "%g", ch->velocityLimit);
		if (ret != EPHIDGET_OK)
			break;
		ret = bridgeSendToDevice(phid, BP_SETACCELERATION, NULL, NULL, "%g", ch->acceleration);
		if (ret != EPHIDGET_OK)
			break;
		ret = bridgeSendToDevice(phid, BP_SETCURRENTLIMIT, NULL, NULL, "%g", ch->currentLimit);
		if (ret != EPHIDGET_OK)
			break;
		break;
	case PHIDCHUID_1067_STEPPER_200:
		ret = bridgeSendToDevice(phid, BP_SETDATAINTERVAL, NULL, NULL, "%u",
		  (uint32_t)round(ch->dataInterval));
		if (ret != EPHIDGET_OK)
			break;
		ret = bridgeSendToDevice(phid, BP_SETCONTROLMODE, NULL, NULL, "%d", ch->controlMode);
		if (ret != EPHIDGET_OK)
			break;
		ret = bridgeSendToDevice(phid, BP_SETVELOCITYLIMIT, NULL, NULL, "%g", ch->velocityLimit);
		if (ret != EPHIDGET_OK)
			break;
		ret = bridgeSendToDevice(phid, BP_SETACCELERATION, NULL, NULL, "%g", ch->acceleration);
		if (ret != EPHIDGET_OK)
			break;
		ret = bridgeSendToDevice(phid, BP_SETCURRENTLIMIT, NULL, NULL, "%g", ch->currentLimit);
		if (ret != EPHIDGET_OK)
			break;
		break;
	case PHIDCHUID_1067_STEPPER_300:
		ret = bridgeSendToDevice(phid, BP_SETDATAINTERVAL, NULL, NULL, "%u",
		  (uint32_t)round(ch->dataInterval));
		if (ret != EPHIDGET_OK)
			break;
		ret = bridgeSendToDevice(phid, BP_SETCONTROLMODE, NULL, NULL, "%d", ch->controlMode);
		if (ret != EPHIDGET_OK)
			break;
		ret = bridgeSendToDevice(phid, BP_SETVELOCITYLIMIT, NULL, NULL, "%g", ch->velocityLimit);
		if (ret != EPHIDGET_OK)
			break;
		ret = bridgeSendToDevice(phid, BP_SETACCELERATION, NULL, NULL, "%g", ch->acceleration);
		if (ret != EPHIDGET_OK)
			break;
		ret = bridgeSendToDevice(phid, BP_SETCURRENTLIMIT, NULL, NULL, "%g", ch->currentLimit);
		if (ret != EPHIDGET_OK)
			break;
		break;
	case PHIDCHUID_STC1000_STEPPER_100:
		ret = bridgeSendToDevice(phid, BP_SETDATAINTERVAL, NULL, NULL, "%u",
		  (uint32_t)round(ch->dataInterval));
		if (ret != EPHIDGET_OK)
			break;
		ret = bridgeSendToDevice(phid, BP_SETCONTROLMODE, NULL, NULL, "%d", ch->controlMode);
		if (ret != EPHIDGET_OK)
			break;
		ret = bridgeSendToDevice(phid, BP_SETVELOCITYLIMIT, NULL, NULL, "%g", ch->velocityLimit);
		if (ret != EPHIDGET_OK)
			break;
		ret = bridgeSendToDevice(phid, BP_SETACCELERATION, NULL, NULL, "%g", ch->acceleration);
		if (ret != EPHIDGET_OK)
			break;
		ret = bridgeSendToDevice(phid, BP_SETCURRENTLIMIT, NULL, NULL, "%g", ch->currentLimit);
		if (ret != EPHIDGET_OK)
			break;
		break;
	case PHIDCHUID_STC1000_STEPPER_110:
		ret = bridgeSendToDevice(phid, BP_SETDATAINTERVAL, NULL, NULL, "%u",
		  (uint32_t)round(ch->dataInterval));
		if (ret != EPHIDGET_OK)
			break;
		ret = bridgeSendToDevice(phid, BP_SETCONTROLMODE, NULL, NULL, "%d", ch->controlMode);
		if (ret != EPHIDGET_OK)
			break;
		ret = bridgeSendToDevice(phid, BP_SETVELOCITYLIMIT, NULL, NULL, "%g", ch->velocityLimit);
		if (ret != EPHIDGET_OK)
			break;
		ret = bridgeSendToDevice(phid, BP_SETACCELERATION, NULL, NULL, "%g", ch->acceleration);
		if (ret != EPHIDGET_OK)
			break;
		ret = bridgeSendToDevice(phid, BP_SETCURRENTLIMIT, NULL, NULL, "%g", ch->currentLimit);
		if (ret != EPHIDGET_OK)
			break;
		break;
	case PHIDCHUID_STC1001_STEPPER_100:
		ret = bridgeSendToDevice(phid, BP_SETDATAINTERVAL, NULL, NULL, "%u",
		  (uint32_t)round(ch->dataInterval));
		if (ret != EPHIDGET_OK)
			break;
		ret = bridgeSendToDevice(phid, BP_SETCONTROLMODE, NULL, NULL, "%d", ch->controlMode);
		if (ret != EPHIDGET_OK)
			break;
		ret = bridgeSendToDevice(phid, BP_SETVELOCITYLIMIT, NULL, NULL, "%g", ch->velocityLimit);
		if (ret != EPHIDGET_OK)
			break;
		ret = bridgeSendToDevice(phid, BP_SETACCELERATION, NULL, NULL, "%g", ch->acceleration);
		if (ret != EPHIDGET_OK)
			break;
		ret = bridgeSendToDevice(phid, BP_SETCURRENTLIMIT, NULL, NULL, "%g", ch->currentLimit);
		if (ret != EPHIDGET_OK)
			break;
		break;
	case PHIDCHUID_STC1001_STEPPER_110:
		ret = bridgeSendToDevice(phid, BP_SETDATAINTERVAL, NULL, NULL, "%u",
		  (uint32_t)round(ch->dataInterval));
		if (ret != EPHIDGET_OK)
			break;
		ret = bridgeSendToDevice(phid, BP_SETCONTROLMODE, NULL, NULL, "%d", ch->controlMode);
		if (ret != EPHIDGET_OK)
			break;
		ret = bridgeSendToDevice(phid, BP_SETVELOCITYLIMIT, NULL, NULL, "%g", ch->velocityLimit);
		if (ret != EPHIDGET_OK)
			break;
		ret = bridgeSendToDevice(phid, BP_SETACCELERATION, NULL, NULL, "%g", ch->acceleration);
		if (ret != EPHIDGET_OK)
			break;
		ret = bridgeSendToDevice(phid, BP_SETCURRENTLIMIT, NULL, NULL, "%g", ch->currentLimit);
		if (ret != EPHIDGET_OK)
			break;
		break;
	case PHIDCHUID_STC1002_STEPPER_100:
		ret = bridgeSendToDevice(phid, BP_SETDATAINTERVAL, NULL, NULL, "%u",
		  (uint32_t)round(ch->dataInterval));
		if (ret != EPHIDGET_OK)
			break;
		ret = bridgeSendToDevice(phid, BP_SETCONTROLMODE, NULL, NULL, "%d", ch->controlMode);
		if (ret != EPHIDGET_OK)
			break;
		ret = bridgeSendToDevice(phid, BP_SETVELOCITYLIMIT, NULL, NULL, "%g", ch->velocityLimit);
		if (ret != EPHIDGET_OK)
			break;
		ret = bridgeSendToDevice(phid, BP_SETACCELERATION, NULL, NULL, "%g", ch->acceleration);
		if (ret != EPHIDGET_OK)
			break;
		ret = bridgeSendToDevice(phid, BP_SETCURRENTLIMIT, NULL, NULL, "%g", ch->currentLimit);
		if (ret != EPHIDGET_OK)
			break;
		break;
	case PHIDCHUID_STC1002_STEPPER_110:
		ret = bridgeSendToDevice(phid, BP_SETDATAINTERVAL, NULL, NULL, "%u",
		  (uint32_t)round(ch->dataInterval));
		if (ret != EPHIDGET_OK)
			break;
		ret = bridgeSendToDevice(phid, BP_SETCONTROLMODE, NULL, NULL, "%d", ch->controlMode);
		if (ret != EPHIDGET_OK)
			break;
		ret = bridgeSendToDevice(phid, BP_SETVELOCITYLIMIT, NULL, NULL, "%g", ch->velocityLimit);
		if (ret != EPHIDGET_OK)
			break;
		ret = bridgeSendToDevice(phid, BP_SETACCELERATION, NULL, NULL, "%g", ch->acceleration);
		if (ret != EPHIDGET_OK)
			break;
		ret = bridgeSendToDevice(phid, BP_SETCURRENTLIMIT, NULL, NULL, "%g", ch->currentLimit);
		if (ret != EPHIDGET_OK)
			break;
		break;
	case PHIDCHUID_STC1003_STEPPER_100:
		ret = bridgeSendToDevice(phid, BP_SETDATAINTERVAL, NULL, NULL, "%u",
		  (uint32_t)round(ch->dataInterval));
		if (ret != EPHIDGET_OK)
			break;
		ret = bridgeSendToDevice(phid, BP_SETCONTROLMODE, NULL, NULL, "%d", ch->controlMode);
		if (ret != EPHIDGET_OK)
			break;
		ret = bridgeSendToDevice(phid, BP_SETVELOCITYLIMIT, NULL, NULL, "%g", ch->velocityLimit);
		if (ret != EPHIDGET_OK)
			break;
		ret = bridgeSendToDevice(phid, BP_SETACCELERATION, NULL, NULL, "%g", ch->acceleration);
		if (ret != EPHIDGET_OK)
			break;
		ret = bridgeSendToDevice(phid, BP_SETCURRENTLIMIT, NULL, NULL, "%g", ch->currentLimit);
		if (ret != EPHIDGET_OK)
			break;
		break;
	case PHIDCHUID_STC1003_STEPPER_110:
		ret = bridgeSendToDevice(phid, BP_SETDATAINTERVAL, NULL, NULL, "%u",
		  (uint32_t)round(ch->dataInterval));
		if (ret != EPHIDGET_OK)
			break;
		ret = bridgeSendToDevice(phid, BP_SETCONTROLMODE, NULL, NULL, "%d", ch->controlMode);
		if (ret != EPHIDGET_OK)
			break;
		ret = bridgeSendToDevice(phid, BP_SETVELOCITYLIMIT, NULL, NULL, "%g", ch->velocityLimit);
		if (ret != EPHIDGET_OK)
			break;
		ret = bridgeSendToDevice(phid, BP_SETACCELERATION, NULL, NULL, "%g", ch->acceleration);
		if (ret != EPHIDGET_OK)
			break;
		ret = bridgeSendToDevice(phid, BP_SETCURRENTLIMIT, NULL, NULL, "%g", ch->currentLimit);
		if (ret != EPHIDGET_OK)
			break;
		break;
	case PHIDCHUID_STC1003_STEPPER_200:
		ret = bridgeSendToDevice(phid, BP_SETDATAINTERVAL, NULL, NULL, "%u",
		  (uint32_t)round(ch->dataInterval));
		if (ret != EPHIDGET_OK)
			break;
		ret = bridgeSendToDevice(phid, BP_SETCONTROLMODE, NULL, NULL, "%d", ch->controlMode);
		if (ret != EPHIDGET_OK)
			break;
		ret = bridgeSendToDevice(phid, BP_SETVELOCITYLIMIT, NULL, NULL, "%g", ch->velocityLimit);
		if (ret != EPHIDGET_OK)
			break;
		ret = bridgeSendToDevice(phid, BP_SETACCELERATION, NULL, NULL, "%g", ch->acceleration);
		if (ret != EPHIDGET_OK)
			break;
		ret = bridgeSendToDevice(phid, BP_SETCURRENTLIMIT, NULL, NULL, "%g", ch->currentLimit);
		if (ret != EPHIDGET_OK)
			break;
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

	mos_free(*ch, sizeof (struct _PhidgetStepper));
}

static PhidgetReturnCode CCONV
_create(PhidgetStepperHandle *phidp) {

	CHANNELCREATE_BODY(Stepper, PHIDCHCLASS_STEPPER);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetStepper_delete(PhidgetStepperHandle *phidp) {

	return (Phidget_delete((PhidgetHandle *)phidp));
}

API_PRETURN
PhidgetStepper_enableFailsafe(PhidgetStepperHandle ch, uint32_t failsafeTime) {

	TESTPTR_PR(ch);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_STEPPER);
	TESTATTACHED_PR(ch);

	return bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETFAILSAFETIME, NULL, NULL, "%u",
	  failsafeTime);
}

API_PRETURN
PhidgetStepper_resetFailsafe(PhidgetStepperHandle ch) {

	TESTPTR_PR(ch);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_STEPPER);
	TESTATTACHED_PR(ch);

	return bridgeSendToDevice((PhidgetChannelHandle)ch, BP_FAILSAFERESET, NULL, NULL, NULL);
}

API_PRETURN
PhidgetStepper_setControlMode(PhidgetStepperHandle ch, PhidgetStepper_ControlMode controlMode) {

	TESTPTR_PR(ch);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_STEPPER);
	TESTATTACHED_PR(ch);

	return (bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETCONTROLMODE, NULL, NULL, "%d",
	  controlMode));
}

API_PRETURN
PhidgetStepper_getControlMode(PhidgetStepperHandle ch, PhidgetStepper_ControlMode *controlMode) {

	TESTPTR_PR(ch);
	TESTPTR_PR(controlMode);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_STEPPER);
	TESTATTACHED_PR(ch);

	*controlMode = ch->controlMode;
	if (ch->controlMode == (PhidgetStepper_ControlMode)PUNK_ENUM)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetStepper_setCurrentLimit(PhidgetStepperHandle ch, double currentLimit) {

	TESTPTR_PR(ch);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_STEPPER);
	TESTATTACHED_PR(ch);

	return (bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETCURRENTLIMIT, NULL, NULL, "%g",
	  currentLimit));
}

API_PRETURN
PhidgetStepper_getCurrentLimit(PhidgetStepperHandle ch, double *currentLimit) {

	TESTPTR_PR(ch);
	TESTPTR_PR(currentLimit);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_STEPPER);
	TESTATTACHED_PR(ch);

	switch (ch->phid.UCD->uid) {
	case PHIDCHUID_1062_STEPPER_100:
		return (PHID_RETURN(EPHIDGET_UNSUPPORTED));
	default:
		break;
	}

	*currentLimit = ch->currentLimit;
	if (ch->currentLimit == (double)PUNK_DBL)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetStepper_getMinCurrentLimit(PhidgetStepperHandle ch, double *minCurrentLimit) {

	TESTPTR_PR(ch);
	TESTPTR_PR(minCurrentLimit);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_STEPPER);
	TESTATTACHED_PR(ch);

	switch (ch->phid.UCD->uid) {
	case PHIDCHUID_1062_STEPPER_100:
		return (PHID_RETURN(EPHIDGET_UNSUPPORTED));
	default:
		break;
	}

	*minCurrentLimit = ch->minCurrentLimit;
	if (ch->minCurrentLimit == (double)PUNK_DBL)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetStepper_getMaxCurrentLimit(PhidgetStepperHandle ch, double *maxCurrentLimit) {

	TESTPTR_PR(ch);
	TESTPTR_PR(maxCurrentLimit);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_STEPPER);
	TESTATTACHED_PR(ch);

	switch (ch->phid.UCD->uid) {
	case PHIDCHUID_1062_STEPPER_100:
		return (PHID_RETURN(EPHIDGET_UNSUPPORTED));
	default:
		break;
	}

	*maxCurrentLimit = ch->maxCurrentLimit;
	if (ch->maxCurrentLimit == (double)PUNK_DBL)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetStepper_setDataInterval(PhidgetStepperHandle ch, uint32_t dataInterval) {

	TESTPTR_PR(ch);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_STEPPER);
	TESTATTACHED_PR(ch);

	return (bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETDATAINTERVAL, NULL, NULL, "%u",
	  dataInterval));
}

API_PRETURN
PhidgetStepper_getDataInterval(PhidgetStepperHandle ch, uint32_t *dataInterval) {

	TESTPTR_PR(ch);
	TESTPTR_PR(dataInterval);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_STEPPER);
	TESTATTACHED_PR(ch);

	*dataInterval = (uint32_t)round(ch->dataInterval);
	if (ch->dataInterval == (double)PUNK_DBL)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetStepper_getMinDataInterval(PhidgetStepperHandle ch, uint32_t *minDataInterval) {

	TESTPTR_PR(ch);
	TESTPTR_PR(minDataInterval);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_STEPPER);
	TESTATTACHED_PR(ch);

	*minDataInterval = ch->minDataInterval;
	if (ch->minDataInterval == (uint32_t)PUNK_UINT32)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetStepper_getMaxDataInterval(PhidgetStepperHandle ch, uint32_t *maxDataInterval) {

	TESTPTR_PR(ch);
	TESTPTR_PR(maxDataInterval);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_STEPPER);
	TESTATTACHED_PR(ch);

	*maxDataInterval = ch->maxDataInterval;
	if (ch->maxDataInterval == (uint32_t)PUNK_UINT32)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetStepper_setDataRate(PhidgetStepperHandle ch, double dataRate) {

	TESTPTR_PR(ch);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_STEPPER);
	TESTATTACHED_PR(ch);

	return bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETDATAINTERVAL, NULL, NULL, "%u%g",
	  (uint32_t)round(1000.0 / dataRate), (double)(1000.0 / dataRate));
}

API_PRETURN
PhidgetStepper_getDataRate(PhidgetStepperHandle ch, double *dataRate) {

	TESTPTR_PR(ch);
	TESTPTR_PR(dataRate);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_STEPPER);
	TESTATTACHED_PR(ch);

	*dataRate = round_double((1000.0 / ch->dataInterval), 4);
	if (ch->dataInterval == (double)PUNK_DBL)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetStepper_getMinDataRate(PhidgetStepperHandle ch, double *minDataRate) {

	TESTPTR_PR(ch);
	TESTPTR_PR(minDataRate);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_STEPPER);
	TESTATTACHED_PR(ch);

	*minDataRate = ch->minDataRate;
	if (ch->minDataRate == (double)PUNK_DBL)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetStepper_getMaxDataRate(PhidgetStepperHandle ch, double *maxDataRate) {

	TESTPTR_PR(ch);
	TESTPTR_PR(maxDataRate);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_STEPPER);
	TESTATTACHED_PR(ch);

	*maxDataRate = ch->maxDataRate;
	if (ch->maxDataRate == (double)PUNK_DBL)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetStepper_setEngaged(PhidgetStepperHandle ch, int engaged) {

	TESTPTR_PR(ch);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_STEPPER);
	TESTATTACHED_PR(ch);

	return (bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETENGAGED, NULL, NULL, "%d", engaged));
}

API_PRETURN
PhidgetStepper_getEngaged(PhidgetStepperHandle ch, int *engaged) {

	TESTPTR_PR(ch);
	TESTPTR_PR(engaged);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_STEPPER);
	TESTATTACHED_PR(ch);

	*engaged = ch->engaged;
	if (ch->engaged == (int)PUNK_BOOL)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetStepper_getMinFailsafeTime(PhidgetStepperHandle ch, uint32_t *minFailsafeTime) {

	TESTPTR_PR(ch);
	TESTPTR_PR(minFailsafeTime);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_STEPPER);
	TESTATTACHED_PR(ch);

	switch (ch->phid.UCD->uid) {
	case PHIDCHUID_1062_STEPPER_100:
	case PHIDCHUID_1063_STEPPER_100:
	case PHIDCHUID_1067_STEPPER_200:
	case PHIDCHUID_1067_STEPPER_300:
	case PHIDCHUID_STC1000_STEPPER_100:
	case PHIDCHUID_STC1001_STEPPER_100:
	case PHIDCHUID_STC1002_STEPPER_100:
	case PHIDCHUID_STC1003_STEPPER_100:
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
PhidgetStepper_getMaxFailsafeTime(PhidgetStepperHandle ch, uint32_t *maxFailsafeTime) {

	TESTPTR_PR(ch);
	TESTPTR_PR(maxFailsafeTime);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_STEPPER);
	TESTATTACHED_PR(ch);

	switch (ch->phid.UCD->uid) {
	case PHIDCHUID_1062_STEPPER_100:
	case PHIDCHUID_1063_STEPPER_100:
	case PHIDCHUID_1067_STEPPER_200:
	case PHIDCHUID_1067_STEPPER_300:
	case PHIDCHUID_STC1000_STEPPER_100:
	case PHIDCHUID_STC1001_STEPPER_100:
	case PHIDCHUID_STC1002_STEPPER_100:
	case PHIDCHUID_STC1003_STEPPER_100:
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
PhidgetStepper_setHoldingCurrentLimit(PhidgetStepperHandle ch, double holdingCurrentLimit) {

	TESTPTR_PR(ch);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_STEPPER);
	TESTATTACHED_PR(ch);

	return (bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETHOLDINGCURRENTLIMIT, NULL, NULL, "%g",
	  holdingCurrentLimit));
}

API_PRETURN
PhidgetStepper_getHoldingCurrentLimit(PhidgetStepperHandle ch, double *holdingCurrentLimit) {

	TESTPTR_PR(ch);
	TESTPTR_PR(holdingCurrentLimit);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_STEPPER);
	TESTATTACHED_PR(ch);

	switch (ch->phid.UCD->uid) {
	case PHIDCHUID_1062_STEPPER_100:
	case PHIDCHUID_1063_STEPPER_100:
	case PHIDCHUID_1067_STEPPER_200:
	case PHIDCHUID_1067_STEPPER_300:
		return (PHID_RETURN(EPHIDGET_UNSUPPORTED));
	default:
		break;
	}

	*holdingCurrentLimit = ch->holdingCurrentLimit;
	if (ch->holdingCurrentLimit == (double)PUNK_DBL)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetStepper_getIsMoving(PhidgetStepperHandle ch, int *isMoving) {

	TESTPTR_PR(ch);
	TESTPTR_PR(isMoving);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_STEPPER);
	TESTATTACHED_PR(ch);

	*isMoving = ch->isMoving;
	if (ch->isMoving == (int)PUNK_BOOL)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetStepper_getRescaleFactor(PhidgetStepperHandle ch, double *rescaleFactor) {

	TESTPTR_PR(ch);
	TESTPTR_PR(rescaleFactor);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_STEPPER);
	TESTATTACHED_PR(ch);

	*rescaleFactor = ch->rescaleFactor;
	if (ch->rescaleFactor == (double)PUNK_DBL)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetStepper_setOnPositionChangeHandler(PhidgetStepperHandle ch,
  PhidgetStepper_OnPositionChangeCallback fptr, void *ctx) {

	TESTPTR_PR(ch);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_STEPPER);

	ch->PositionChange = fptr;
	ch->PositionChangeCtx = ctx;

	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetStepper_setOnStoppedHandler(PhidgetStepperHandle ch, PhidgetStepper_OnStoppedCallback fptr,
  void *ctx) {

	TESTPTR_PR(ch);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_STEPPER);

	ch->Stopped = fptr;
	ch->StoppedCtx = ctx;

	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetStepper_setOnVelocityChangeHandler(PhidgetStepperHandle ch,
  PhidgetStepper_OnVelocityChangeCallback fptr, void *ctx) {

	TESTPTR_PR(ch);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_STEPPER);

	ch->VelocityChange = fptr;
	ch->VelocityChangeCtx = ctx;

	return (EPHIDGET_OK);
}
