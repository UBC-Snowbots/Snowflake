/* Generated: Mon Feb 08 2016 10:44:04 GMT-0700 (Mountain Standard Time) */
/* Will not be regenerated. */

#include "phidgetbase.h"
#include "class/rcservo.gen.h"
#include "class/rcservo.gen.c"

#define POSN_RANGE (ch->maxPosition - ch->minPosition)
#define POSN_RANGE_ABS fabs(POSN_RANGE)
#define PWM_RANGE (ch->maxPulseWidth - ch->minPulseWidth)

#define POSITION_USER(position_us)																				\
(position_us > ch->minPulseWidth ? (																			\
	(ch->minPosition + ((position_us - ch->minPulseWidth) / PWM_RANGE) * (ch->maxPosition - ch->minPosition))	\
) : (																											\
	ch->minPosition)																							\
)
#define POSITION_US(position_user)														\
(ch->maxPosition > ch->minPosition ? (													\
	(ch->minPulseWidth + (PWM_RANGE * (position_user - ch->minPosition)) / POSN_RANGE)	\
) : (																					\
	(ch->maxPulseWidth) + (PWM_RANGE * (position_user - ch->maxPosition)) / POSN_RANGE)	\
)
#define VELOCITY_USER(velocity_us) ((POSN_RANGE_ABS * velocity_us) / PWM_RANGE)
#define VELOCITY_US(velocity_user) ((PWM_RANGE * velocity_user) / POSN_RANGE_ABS)
#define ACCEL_USER(accel_us) ((POSN_RANGE_ABS * accel_us) / PWM_RANGE)
#define ACCEL_US(accel_user) ((PWM_RANGE * accel_user) / POSN_RANGE_ABS)

static void CCONV
PhidgetRCServo_errorHandler(PhidgetChannelHandle phid, Phidget_ErrorEventCode code) {}

static void CCONV
PhidgetRCServo_free(PhidgetChannelHandle *ch) {
	_free(ch);
}

API_PRETURN
PhidgetRCServo_create(PhidgetRCServoHandle *phidp) {
	return (_create(phidp));
}

static PhidgetReturnCode CCONV
PhidgetRCServo_setStatus(PhidgetChannelHandle phid, BridgePacket *bp) {
	return (_setStatus(phid, bp));
}

static PhidgetReturnCode CCONV
PhidgetRCServo_getStatus(PhidgetChannelHandle phid, BridgePacket **bp) {
	return (_getStatus(phid, bp));
}

static PhidgetReturnCode CCONV
PhidgetRCServo_initAfterOpen(PhidgetChannelHandle phid) {
	return (_initAfterOpen(phid));
}

static PhidgetReturnCode CCONV
PhidgetRCServo_setDefaults(PhidgetChannelHandle phid) {
	return (_setDefaults(phid));
}

static PhidgetReturnCode
PhidgetRCServo_bridgeInput(PhidgetChannelHandle phid, BridgePacket *bp) {
	PhidgetRCServoHandle ch;
	PhidgetReturnCode res;

	ch = (PhidgetRCServoHandle)phid;

	switch (bp->vpkt) {
	case BP_POSITIONCHANGE:
		ch->position = getBridgePacketDouble(bp, 0);
		FIRECH(ch, PositionChange, POSITION_USER(ch->position));
		res = EPHIDGET_OK;
		break;
	case BP_VELOCITYCHANGE:
		ch->velocity = getBridgePacketDouble(bp, 0);
		FIRECH(ch, VelocityChange, VELOCITY_USER(ch->velocity));
		res = EPHIDGET_OK;
		break;
	case BP_TARGETPOSITIONREACHED:
		ch->isMoving = PFALSE;
		ch->position = getBridgePacketDouble(bp, 0);
		FIRECH(ch, TargetPositionReached, POSITION_USER(ch->position));
		res = EPHIDGET_OK;
		break;
	case BP_SETENGAGED:
	case BP_SETVELOCITYLIMIT:
	case BP_SETTARGETPOSITION:
		if (bp->vpkt == BP_SETTARGETPOSITION)
			TESTRANGE_IOP(bp->iop, "%lf", getBridgePacketDouble(bp, 0), ch->minPulseWidth, ch->maxPulseWidth);

		res = _bridgeInput(phid, bp);
		if (res == EPHIDGET_OK) {
			if (ch->engaged == PTRUE && ch->velocityLimit != 0 && ch->position != ch->targetPosition)
				ch->isMoving = PTRUE;
		}
		break;
	case BP_SETMINPULSEWIDTH:
		TESTRANGE_IOP(bp->iop, "%lf", getBridgePacketDouble(bp, 0), ch->minPulseWidthLimit, ch->maxPulseWidth);
		res = _bridgeInput(phid, bp);
		break;
	case BP_SETMAXPULSEWIDTH:
		TESTRANGE_IOP(bp->iop, "%lf", getBridgePacketDouble(bp, 0), ch->minPulseWidth, ch->maxPulseWidthLimit);
		res = _bridgeInput(phid, bp);
		break;
	case BP_SETFAILSAFETIME:
		TESTRANGE_IOP(bp->iop, "%u", getBridgePacketUInt32(bp, 0), ch->minFailsafeTime, ch->maxFailsafeTime);
		res = _bridgeInput(phid, bp);
		break;
	default:
		res = _bridgeInput(phid, bp);
		break;
	}

	return (res);
}

static void
PhidgetRCServo_fireInitialEvents(PhidgetChannelHandle phid) {
	_fireInitialEvents(phid);
}

static int
PhidgetRCServo_hasInitialState(PhidgetChannelHandle phid) {
	return (_hasInitialState(phid));
}

API_PRETURN
PhidgetRCServo_setMaxPosition(PhidgetRCServoHandle ch, double maxPosition) {

	TESTPTR_PR(ch);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_RCSERVO);
	TESTATTACHED_PR(ch);

	ch->maxPosition = maxPosition;
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetRCServo_setMinPosition(PhidgetRCServoHandle ch, double minPosition) {

	TESTPTR_PR(ch);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_RCSERVO);
	TESTATTACHED_PR(ch);

	ch->minPosition = minPosition;
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetRCServo_getMaxVelocityLimit(PhidgetRCServoHandle ch, double *maxVelocityLimit) {

	TESTPTR_PR(ch);
	TESTPTR_PR(maxVelocityLimit);
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

	if (ch->maxVelocityLimit == PUNK_DBL) {
		*maxVelocityLimit = PUNK_DBL;
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	}
	*maxVelocityLimit = VELOCITY_USER(ch->maxVelocityLimit);
	return (EPHIDGET_OK);
}


API_PRETURN
PhidgetRCServo_getMaxAcceleration(PhidgetRCServoHandle ch, double *maxAcceleration) {

	TESTPTR_PR(ch);
	TESTPTR_PR(maxAcceleration);
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

	if (ch->maxAcceleration == PUNK_DBL) {
		*maxAcceleration = PUNK_DBL;
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	}
	*maxAcceleration = ACCEL_USER(ch->maxAcceleration);
	return (EPHIDGET_OK);
}


API_PRETURN
PhidgetRCServo_getMinAcceleration(PhidgetRCServoHandle ch, double *minAcceleration) {

	TESTPTR_PR(ch);
	TESTPTR_PR(minAcceleration);
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

	if (ch->minAcceleration == PUNK_DBL) {
		*minAcceleration = PUNK_DBL;
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	}
	*minAcceleration = ACCEL_USER(ch->minAcceleration);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetRCServo_getPosition(PhidgetRCServoHandle ch, double *position) {

	TESTPTR_PR(ch);
	TESTPTR_PR(position);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_RCSERVO);
	TESTATTACHED_PR(ch);

	if (ch->position == PUNK_DBL) {
		*position = PUNK_DBL;
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	}
	*position = POSITION_USER(ch->position);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetRCServo_setTargetPosition(PhidgetRCServoHandle ch, double targetPosition) {

	TESTPTR_PR(ch);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_RCSERVO);
	TESTATTACHED_PR(ch);

	return (bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETTARGETPOSITION, NULL, NULL, "%g",
		POSITION_US(targetPosition)));
}

API_VRETURN
PhidgetRCServo_setTargetPosition_async(PhidgetRCServoHandle ch, double targetPosition, Phidget_AsyncCallback fptr,
	void *ctx) {
	PhidgetReturnCode res;

	if (ch == NULL) {
		if (fptr) fptr((PhidgetHandle)ch, ctx, EPHIDGET_INVALIDARG);
		return;
	}
	if (ch->phid.class != PHIDCHCLASS_RCSERVO) {
		if (fptr) fptr((PhidgetHandle)ch, ctx, EPHIDGET_WRONGDEVICE);
		return;
	}
	if (!ISATTACHED(ch)) {
		if (fptr) fptr((PhidgetHandle)ch, ctx, EPHIDGET_NOTATTACHED);
		return;
	}

	res = bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETTARGETPOSITION, fptr, ctx, "%g", POSITION_US(targetPosition));
	if (res != EPHIDGET_OK && fptr != NULL)
		fptr((PhidgetHandle)ch, ctx, res);
}

API_PRETURN
PhidgetRCServo_getTargetPosition(PhidgetRCServoHandle ch, double *targetPosition) {

	TESTPTR_PR(ch);
	TESTPTR_PR(targetPosition);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_RCSERVO);
	TESTATTACHED_PR(ch);

	if (ch->targetPosition == PUNK_DBL) {
		*targetPosition = PUNK_DBL;
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	}
	*targetPosition = POSITION_USER(ch->targetPosition);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetRCServo_setVelocityLimit(PhidgetRCServoHandle ch, double velocityLimit) {

	TESTPTR_PR(ch);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_RCSERVO);
	TESTATTACHED_PR(ch);

	return (bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETVELOCITYLIMIT, NULL, NULL, "%g",
		VELOCITY_US(velocityLimit)));
}

API_PRETURN
PhidgetRCServo_getVelocityLimit(PhidgetRCServoHandle ch, double *velocityLimit) {

	TESTPTR_PR(ch);
	TESTPTR_PR(velocityLimit);
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

	if (ch->velocityLimit == PUNK_DBL) {
		*velocityLimit = PUNK_DBL;
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	}
	*velocityLimit = VELOCITY_USER(ch->velocityLimit);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetRCServo_getVelocity(PhidgetRCServoHandle ch, double *velocity) {

	TESTPTR_PR(ch);
	TESTPTR_PR(velocity);
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
		return (PHID_RETURN(EPHIDGET_UNSUPPORTED));
	default:
		break;
	}

	if (ch->velocity == PUNK_DBL) {
		*velocity = PUNK_DBL;
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	}
	*velocity = VELOCITY_USER(ch->velocity);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetRCServo_setAcceleration(PhidgetRCServoHandle ch, double acceleration) {

	TESTPTR_PR(ch);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_RCSERVO);
	TESTATTACHED_PR(ch);

	return (bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETACCELERATION, NULL, NULL, "%g",
		ACCEL_US(acceleration)));
}

API_PRETURN
PhidgetRCServo_getAcceleration(PhidgetRCServoHandle ch, double *acceleration) {

	TESTPTR_PR(ch);
	TESTPTR_PR(acceleration);
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

	if (ch->acceleration == PUNK_DBL) {
		*acceleration = PUNK_DBL;
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	}
	*acceleration = ACCEL_USER(ch->acceleration);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetRCServo_getMinVelocityLimit(PhidgetRCServoHandle ch, double *minVelocityLimit) {

	TESTPTR_PR(ch);
	TESTPTR_PR(minVelocityLimit);
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

	if (ch->minVelocityLimit == PUNK_DBL) {
		*minVelocityLimit = PUNK_DBL;
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	}
	*minVelocityLimit = VELOCITY_USER(ch->minVelocityLimit);
	return (EPHIDGET_OK);
}
