
#ifdef COMPILE_PHIDGETS_LABVIEW
void CCONV Phidget_OnAttach_LaviewHandler(PhidgetHandle ch,void *ctx) {
	LVUserEventRef ev = *(LVUserEventRef *)ctx;
	MgErr ret = 0;
	lvPhidgetAttachArgs *lvdata;
	lvdata = (lvPhidgetAttachArgs *)DSNewPtr(sizeof(lvPhidgetAttachArgs));
	lvdata->ch = (intptr_t)ch;
	ret = PostLVUserEvent(ev, lvdata);
	DSDisposePtr(lvdata);
}

API_PRETURN_HDR Phidget_setOnAttachLabviewHandler(PhidgetHandle ch, LVUserEventRef *lvEventRef) {
	TESTPTR(ch);
	if(lvEventRef && *lvEventRef)
		Phidget_setOnAttachHandler(ch, Phidget_OnAttach_LaviewHandler, lvEventRef);
	else
		Phidget_setOnAttachHandler(ch, NULL, NULL);

	return (EPHIDGET_OK);
}
#else
API_PRETURN_HDR Phidget_setOnAttachLabviewHandler(PhidgetHandle ch, LVUserEventRef *lvEventRef) {

	return (EPHIDGET_UNSUPPORTED);
}
#endif

#ifdef COMPILE_PHIDGETS_LABVIEW
void CCONV Phidget_OnDetach_LaviewHandler(PhidgetHandle ch,void *ctx) {
	LVUserEventRef ev = *(LVUserEventRef *)ctx;
	MgErr ret = 0;
	lvPhidgetDetachArgs *lvdata;
	lvdata = (lvPhidgetDetachArgs *)DSNewPtr(sizeof(lvPhidgetDetachArgs));
	lvdata->ch = (intptr_t)ch;
	ret = PostLVUserEvent(ev, lvdata);
	DSDisposePtr(lvdata);
}

API_PRETURN_HDR Phidget_setOnDetachLabviewHandler(PhidgetHandle ch, LVUserEventRef *lvEventRef) {
	TESTPTR(ch);
	if(lvEventRef && *lvEventRef)
		Phidget_setOnDetachHandler(ch, Phidget_OnDetach_LaviewHandler, lvEventRef);
	else
		Phidget_setOnDetachHandler(ch, NULL, NULL);

	return (EPHIDGET_OK);
}
#else
API_PRETURN_HDR Phidget_setOnDetachLabviewHandler(PhidgetHandle ch, LVUserEventRef *lvEventRef) {

	return (EPHIDGET_UNSUPPORTED);
}
#endif

#ifdef COMPILE_PHIDGETS_LABVIEW
void CCONV Phidget_OnError_LaviewHandler(PhidgetHandle ch,void *ctx, Phidget_ErrorEventCode code,
  const char *description) {
	LVUserEventRef ev = *(LVUserEventRef *)ctx;
	MgErr ret = 0;
	lvPhidgetErrorArgs *lvdata;
	lvdata = (lvPhidgetErrorArgs *)DSNewPtr(sizeof(lvPhidgetErrorArgs));
	lvdata->ch = (intptr_t)ch;
	lvdata->code = code;
	lvdata->description = (LStrHandle)DSNewHandle(sizeof(int32_t) + sizeof(char) * (mos_strlen(description) + 1));
	mos_strlcpy((char *)LStrBuf(*lvdata->description), description, mos_strlen(description) + 1);
	LStrLen(*lvdata->description) = (int32_t)mos_strlen(description);
	ret = PostLVUserEvent(ev, lvdata);
	DSDisposePtr(lvdata);
}

API_PRETURN_HDR Phidget_setOnErrorLabviewHandler(PhidgetHandle ch, LVUserEventRef *lvEventRef) {
	TESTPTR(ch);
	if(lvEventRef && *lvEventRef)
		Phidget_setOnErrorHandler(ch, Phidget_OnError_LaviewHandler, lvEventRef);
	else
		Phidget_setOnErrorHandler(ch, NULL, NULL);

	return (EPHIDGET_OK);
}
#else
API_PRETURN_HDR Phidget_setOnErrorLabviewHandler(PhidgetHandle ch, LVUserEventRef *lvEventRef) {

	return (EPHIDGET_UNSUPPORTED);
}
#endif

#ifdef COMPILE_PHIDGETS_LABVIEW
void CCONV Phidget_OnPropertyChange_LaviewHandler(PhidgetHandle ch,void *ctx,
  const char *propertyName) {
	LVUserEventRef ev = *(LVUserEventRef *)ctx;
	MgErr ret = 0;
	lvPhidgetPropertyChangeArgs *lvdata;
	lvdata = (lvPhidgetPropertyChangeArgs *)DSNewPtr(sizeof(lvPhidgetPropertyChangeArgs));
	lvdata->ch = (intptr_t)ch;
	lvdata->propertyName = (LStrHandle)DSNewHandle(sizeof(int32_t) + sizeof(char) * (mos_strlen(propertyName) + 1));
	mos_strlcpy((char *)LStrBuf(*lvdata->propertyName), propertyName, mos_strlen(propertyName) + 1);
	LStrLen(*lvdata->propertyName) = (int32_t)mos_strlen(propertyName);
	ret = PostLVUserEvent(ev, lvdata);
	DSDisposePtr(lvdata);
}

API_PRETURN_HDR Phidget_setOnPropertyChangeLabviewHandler(PhidgetHandle ch,
  LVUserEventRef *lvEventRef) {
	TESTPTR(ch);
	if(lvEventRef && *lvEventRef)
		Phidget_setOnPropertyChangeHandler(ch, Phidget_OnPropertyChange_LaviewHandler, lvEventRef);
	else
		Phidget_setOnPropertyChangeHandler(ch, NULL, NULL);

	return (EPHIDGET_OK);
}
#else
API_PRETURN_HDR Phidget_setOnPropertyChangeLabviewHandler(PhidgetHandle ch,
  LVUserEventRef *lvEventRef) {

	return (EPHIDGET_UNSUPPORTED);
}
#endif

#ifdef COMPILE_PHIDGETS_LABVIEW
void CCONV PhidgetManager_OnAttach_LaviewHandler(PhidgetManagerHandle man,void *ctx,
  PhidgetHandle channel) {
	LVUserEventRef ev = *(LVUserEventRef *)ctx;
	MgErr ret = 0;
	lvPhidgetManagerAttachArgs *lvdata;
	lvdata = (lvPhidgetManagerAttachArgs *)DSNewPtr(sizeof(lvPhidgetManagerAttachArgs));
	lvdata->man = (intptr_t)man;
	lvdata->channel = (intptr_t)channel;
	Phidget_retain(channel);
	ret = PostLVUserEvent(ev, lvdata);
	DSDisposePtr(lvdata);
}

API_PRETURN_HDR PhidgetManager_setOnAttachLabviewHandler(PhidgetManagerHandle man,
  LVUserEventRef *lvEventRef) {
	TESTPTR(man);
	if(lvEventRef && *lvEventRef)
		PhidgetManager_setOnAttachHandler(man, PhidgetManager_OnAttach_LaviewHandler, lvEventRef);
	else
		PhidgetManager_setOnAttachHandler(man, NULL, NULL);

	return (EPHIDGET_OK);
}
#else
API_PRETURN_HDR PhidgetManager_setOnAttachLabviewHandler(PhidgetManagerHandle man,
  LVUserEventRef *lvEventRef) {

	return (EPHIDGET_UNSUPPORTED);
}
#endif

#ifdef COMPILE_PHIDGETS_LABVIEW
void CCONV PhidgetManager_OnDetach_LaviewHandler(PhidgetManagerHandle man,void *ctx,
  PhidgetHandle channel) {
	LVUserEventRef ev = *(LVUserEventRef *)ctx;
	MgErr ret = 0;
	lvPhidgetManagerDetachArgs *lvdata;
	lvdata = (lvPhidgetManagerDetachArgs *)DSNewPtr(sizeof(lvPhidgetManagerDetachArgs));
	lvdata->man = (intptr_t)man;
	lvdata->channel = (intptr_t)channel;
	Phidget_retain(channel);
	ret = PostLVUserEvent(ev, lvdata);
	DSDisposePtr(lvdata);
}

API_PRETURN_HDR PhidgetManager_setOnDetachLabviewHandler(PhidgetManagerHandle man,
  LVUserEventRef *lvEventRef) {
	TESTPTR(man);
	if(lvEventRef && *lvEventRef)
		PhidgetManager_setOnDetachHandler(man, PhidgetManager_OnDetach_LaviewHandler, lvEventRef);
	else
		PhidgetManager_setOnDetachHandler(man, NULL, NULL);

	return (EPHIDGET_OK);
}
#else
API_PRETURN_HDR PhidgetManager_setOnDetachLabviewHandler(PhidgetManagerHandle man,
  LVUserEventRef *lvEventRef) {

	return (EPHIDGET_UNSUPPORTED);
}
#endif

#ifdef COMPILE_PHIDGETS_LABVIEW
void CCONV PhidgetNet_OnServerAdded_LaviewHandler(void *ctx, PhidgetServer *server, void *kv) {
	LVUserEventRef ev = *(LVUserEventRef *)ctx;
	MgErr ret = 0;
	lvPhidgetNetServerAddedArgs *lvdata;
	lvdata = (lvPhidgetNetServerAddedArgs *)DSNewPtr(sizeof(lvPhidgetNetServerAddedArgs));
	lvdata->server = *server;
	lvdata->kv = kv;
	ret = PostLVUserEvent(ev, lvdata);
	DSDisposePtr(lvdata);
}

API_PRETURN_HDR PhidgetNet_setOnServerAddedLabviewHandler(LVUserEventRef *lvEventRef) {
	if(lvEventRef && *lvEventRef)
		PhidgetNet_setOnServerAddedHandler(PhidgetNet_OnServerAdded_LaviewHandler, lvEventRef);
	else
		PhidgetNet_setOnServerAddedHandler(NULL, NULL);

	return (EPHIDGET_OK);
}
#else
API_PRETURN_HDR PhidgetNet_setOnServerAddedLabviewHandler(LVUserEventRef *lvEventRef) {

	return (EPHIDGET_UNSUPPORTED);
}
#endif

#ifdef COMPILE_PHIDGETS_LABVIEW
void CCONV PhidgetNet_OnServerRemoved_LaviewHandler(void *ctx, PhidgetServer *server) {
	LVUserEventRef ev = *(LVUserEventRef *)ctx;
	MgErr ret = 0;
	lvPhidgetNetServerRemovedArgs *lvdata;
	lvdata = (lvPhidgetNetServerRemovedArgs *)DSNewPtr(sizeof(lvPhidgetNetServerRemovedArgs));
	lvdata->server = *server;
	ret = PostLVUserEvent(ev, lvdata);
	DSDisposePtr(lvdata);
}

API_PRETURN_HDR PhidgetNet_setOnServerRemovedLabviewHandler(LVUserEventRef *lvEventRef) {
	if(lvEventRef && *lvEventRef)
		PhidgetNet_setOnServerRemovedHandler(PhidgetNet_OnServerRemoved_LaviewHandler, lvEventRef);
	else
		PhidgetNet_setOnServerRemovedHandler(NULL, NULL);

	return (EPHIDGET_OK);
}
#else
API_PRETURN_HDR PhidgetNet_setOnServerRemovedLabviewHandler(LVUserEventRef *lvEventRef) {

	return (EPHIDGET_UNSUPPORTED);
}
#endif

#ifdef COMPILE_PHIDGETS_LABVIEW
void CCONV PhidgetVoltageRatioInput_OnSensorChange_LaviewHandler(PhidgetVoltageRatioInputHandle ch,
 void *ctx, double sensorValue, Phidget_UnitInfo *sensorUnit) {
	LVUserEventRef ev = *(LVUserEventRef *)ctx;
	MgErr ret = 0;
	lvPhidgetVoltageRatioInputSensorChangeArgs *lvdata;
	lvdata = (lvPhidgetVoltageRatioInputSensorChangeArgs *)DSNewPtr(sizeof(lvPhidgetVoltageRatioInputSensorChangeArgs));
	lvdata->ch = (intptr_t)ch;
	lvdata->sensorValue = sensorValue;
	lvdata->sensorUnit = *sensorUnit;
	ret = PostLVUserEvent(ev, lvdata);
	DSDisposePtr(lvdata);
}

API_PRETURN_HDR PhidgetVoltageRatioInput_setOnSensorChangeLabviewHandler(PhidgetVoltageRatioInputHandle ch, LVUserEventRef *lvEventRef) {
	TESTPTR(ch);
	if(lvEventRef && *lvEventRef)
		PhidgetVoltageRatioInput_setOnSensorChangeHandler(ch,
		  PhidgetVoltageRatioInput_OnSensorChange_LaviewHandler, lvEventRef);
	else
		PhidgetVoltageRatioInput_setOnSensorChangeHandler(ch, NULL, NULL);

	return (EPHIDGET_OK);
}
#else
API_PRETURN_HDR PhidgetVoltageRatioInput_setOnSensorChangeLabviewHandler(PhidgetVoltageRatioInputHandle ch, LVUserEventRef *lvEventRef) {

	return (EPHIDGET_UNSUPPORTED);
}
#endif

#ifdef COMPILE_PHIDGETS_LABVIEW
void CCONV PhidgetVoltageRatioInput_OnVoltageRatioChange_LaviewHandler(PhidgetVoltageRatioInputHandle ch,
 void *ctx, double voltageRatio) {
	LVUserEventRef ev = *(LVUserEventRef *)ctx;
	MgErr ret = 0;
	lvPhidgetVoltageRatioInputVoltageRatioChangeArgs *lvdata;
	lvdata = (lvPhidgetVoltageRatioInputVoltageRatioChangeArgs *)DSNewPtr(sizeof(lvPhidgetVoltageRatioInputVoltageRatioChangeArgs));
	lvdata->ch = (intptr_t)ch;
	lvdata->voltageRatio = voltageRatio;
	ret = PostLVUserEvent(ev, lvdata);
	DSDisposePtr(lvdata);
}

API_PRETURN_HDR PhidgetVoltageRatioInput_setOnVoltageRatioChangeLabviewHandler(PhidgetVoltageRatioInputHandle ch, LVUserEventRef *lvEventRef) {
	TESTPTR(ch);
	if(lvEventRef && *lvEventRef)
		PhidgetVoltageRatioInput_setOnVoltageRatioChangeHandler(ch,
		  PhidgetVoltageRatioInput_OnVoltageRatioChange_LaviewHandler, lvEventRef);
	else
		PhidgetVoltageRatioInput_setOnVoltageRatioChangeHandler(ch, NULL, NULL);

	return (EPHIDGET_OK);
}
#else
API_PRETURN_HDR PhidgetVoltageRatioInput_setOnVoltageRatioChangeLabviewHandler(PhidgetVoltageRatioInputHandle ch, LVUserEventRef *lvEventRef) {

	return (EPHIDGET_UNSUPPORTED);
}
#endif

#ifdef COMPILE_PHIDGETS_LABVIEW
void CCONV PhidgetDigitalInput_OnStateChange_LaviewHandler(PhidgetDigitalInputHandle ch,void *ctx,
  int state) {
	LVUserEventRef ev = *(LVUserEventRef *)ctx;
	MgErr ret = 0;
	lvPhidgetDigitalInputStateChangeArgs *lvdata;
	lvdata = (lvPhidgetDigitalInputStateChangeArgs *)DSNewPtr(sizeof(lvPhidgetDigitalInputStateChangeArgs));
	lvdata->ch = (intptr_t)ch;
	lvdata->state = state;
	ret = PostLVUserEvent(ev, lvdata);
	DSDisposePtr(lvdata);
}

API_PRETURN_HDR PhidgetDigitalInput_setOnStateChangeLabviewHandler(PhidgetDigitalInputHandle ch,
  LVUserEventRef *lvEventRef) {
	TESTPTR(ch);
	if(lvEventRef && *lvEventRef)
		PhidgetDigitalInput_setOnStateChangeHandler(ch, PhidgetDigitalInput_OnStateChange_LaviewHandler,
		  lvEventRef);
	else
		PhidgetDigitalInput_setOnStateChangeHandler(ch, NULL, NULL);

	return (EPHIDGET_OK);
}
#else
API_PRETURN_HDR PhidgetDigitalInput_setOnStateChangeLabviewHandler(PhidgetDigitalInputHandle ch,
  LVUserEventRef *lvEventRef) {

	return (EPHIDGET_UNSUPPORTED);
}
#endif

#ifdef COMPILE_PHIDGETS_LABVIEW
void CCONV PhidgetRCServo_OnPositionChange_LaviewHandler(PhidgetRCServoHandle ch,void *ctx,
  double position) {
	LVUserEventRef ev = *(LVUserEventRef *)ctx;
	MgErr ret = 0;
	lvPhidgetRCServoPositionChangeArgs *lvdata;
	lvdata = (lvPhidgetRCServoPositionChangeArgs *)DSNewPtr(sizeof(lvPhidgetRCServoPositionChangeArgs));
	lvdata->ch = (intptr_t)ch;
	lvdata->position = position;
	ret = PostLVUserEvent(ev, lvdata);
	DSDisposePtr(lvdata);
}

API_PRETURN_HDR PhidgetRCServo_setOnPositionChangeLabviewHandler(PhidgetRCServoHandle ch,
  LVUserEventRef *lvEventRef) {
	TESTPTR(ch);
	if(lvEventRef && *lvEventRef)
		PhidgetRCServo_setOnPositionChangeHandler(ch, PhidgetRCServo_OnPositionChange_LaviewHandler,
		  lvEventRef);
	else
		PhidgetRCServo_setOnPositionChangeHandler(ch, NULL, NULL);

	return (EPHIDGET_OK);
}
#else
API_PRETURN_HDR PhidgetRCServo_setOnPositionChangeLabviewHandler(PhidgetRCServoHandle ch,
  LVUserEventRef *lvEventRef) {

	return (EPHIDGET_UNSUPPORTED);
}
#endif

#ifdef COMPILE_PHIDGETS_LABVIEW
void CCONV PhidgetRCServo_OnTargetPositionReached_LaviewHandler(PhidgetRCServoHandle ch,void *ctx,
  double position) {
	LVUserEventRef ev = *(LVUserEventRef *)ctx;
	MgErr ret = 0;
	lvPhidgetRCServoTargetPositionReachedArgs *lvdata;
	lvdata = (lvPhidgetRCServoTargetPositionReachedArgs *)DSNewPtr(sizeof(lvPhidgetRCServoTargetPositionReachedArgs));
	lvdata->ch = (intptr_t)ch;
	lvdata->position = position;
	ret = PostLVUserEvent(ev, lvdata);
	DSDisposePtr(lvdata);
}

API_PRETURN_HDR PhidgetRCServo_setOnTargetPositionReachedLabviewHandler(PhidgetRCServoHandle ch,
  LVUserEventRef *lvEventRef) {
	TESTPTR(ch);
	if(lvEventRef && *lvEventRef)
		PhidgetRCServo_setOnTargetPositionReachedHandler(ch,
		  PhidgetRCServo_OnTargetPositionReached_LaviewHandler, lvEventRef);
	else
		PhidgetRCServo_setOnTargetPositionReachedHandler(ch, NULL, NULL);

	return (EPHIDGET_OK);
}
#else
API_PRETURN_HDR PhidgetRCServo_setOnTargetPositionReachedLabviewHandler(PhidgetRCServoHandle ch,
  LVUserEventRef *lvEventRef) {

	return (EPHIDGET_UNSUPPORTED);
}
#endif

#ifdef COMPILE_PHIDGETS_LABVIEW
void CCONV PhidgetRCServo_OnVelocityChange_LaviewHandler(PhidgetRCServoHandle ch,void *ctx,
  double velocity) {
	LVUserEventRef ev = *(LVUserEventRef *)ctx;
	MgErr ret = 0;
	lvPhidgetRCServoVelocityChangeArgs *lvdata;
	lvdata = (lvPhidgetRCServoVelocityChangeArgs *)DSNewPtr(sizeof(lvPhidgetRCServoVelocityChangeArgs));
	lvdata->ch = (intptr_t)ch;
	lvdata->velocity = velocity;
	ret = PostLVUserEvent(ev, lvdata);
	DSDisposePtr(lvdata);
}

API_PRETURN_HDR PhidgetRCServo_setOnVelocityChangeLabviewHandler(PhidgetRCServoHandle ch,
  LVUserEventRef *lvEventRef) {
	TESTPTR(ch);
	if(lvEventRef && *lvEventRef)
		PhidgetRCServo_setOnVelocityChangeHandler(ch, PhidgetRCServo_OnVelocityChange_LaviewHandler,
		  lvEventRef);
	else
		PhidgetRCServo_setOnVelocityChangeHandler(ch, NULL, NULL);

	return (EPHIDGET_OK);
}
#else
API_PRETURN_HDR PhidgetRCServo_setOnVelocityChangeLabviewHandler(PhidgetRCServoHandle ch,
  LVUserEventRef *lvEventRef) {

	return (EPHIDGET_UNSUPPORTED);
}
#endif

#ifdef COMPILE_PHIDGETS_LABVIEW
void CCONV PhidgetAccelerometer_OnAccelerationChange_LaviewHandler(PhidgetAccelerometerHandle ch,
 void *ctx, const double acceleration[3], double timestamp) {
	LVUserEventRef ev = *(LVUserEventRef *)ctx;
	MgErr ret = 0;
	lvPhidgetAccelerometerAccelerationChangeArgs *lvdata;
	lvdata = (lvPhidgetAccelerometerAccelerationChangeArgs *)DSNewPtr(sizeof(lvPhidgetAccelerometerAccelerationChangeArgs));
	lvdata->ch = (intptr_t)ch;
	lvdata->acceleration = (lvArrHandle *)DSNewHandle(sizeof(size_t) + sizeof(double) * 3);
	memcpy(&((*(lvdata->acceleration))->data), acceleration, sizeof(double) * 3);
	(*(lvdata->acceleration))->length = (int32_t)3;
	lvdata->timestamp = timestamp;
	ret = PostLVUserEvent(ev, lvdata);
	DSDisposePtr(lvdata);
}

API_PRETURN_HDR PhidgetAccelerometer_setOnAccelerationChangeLabviewHandler(PhidgetAccelerometerHandle ch,
  LVUserEventRef *lvEventRef) {
	TESTPTR(ch);
	if(lvEventRef && *lvEventRef)
		PhidgetAccelerometer_setOnAccelerationChangeHandler(ch,
		  PhidgetAccelerometer_OnAccelerationChange_LaviewHandler, lvEventRef);
	else
		PhidgetAccelerometer_setOnAccelerationChangeHandler(ch, NULL, NULL);

	return (EPHIDGET_OK);
}
#else
API_PRETURN_HDR PhidgetAccelerometer_setOnAccelerationChangeLabviewHandler(PhidgetAccelerometerHandle ch,
  LVUserEventRef *lvEventRef) {

	return (EPHIDGET_UNSUPPORTED);
}
#endif

#ifdef COMPILE_PHIDGETS_LABVIEW
void CCONV PhidgetVoltageInput_OnSensorChange_LaviewHandler(PhidgetVoltageInputHandle ch,void *ctx,
  double sensorValue, Phidget_UnitInfo *sensorUnit) {
	LVUserEventRef ev = *(LVUserEventRef *)ctx;
	MgErr ret = 0;
	lvPhidgetVoltageInputSensorChangeArgs *lvdata;
	lvdata = (lvPhidgetVoltageInputSensorChangeArgs *)DSNewPtr(sizeof(lvPhidgetVoltageInputSensorChangeArgs));
	lvdata->ch = (intptr_t)ch;
	lvdata->sensorValue = sensorValue;
	lvdata->sensorUnit = *sensorUnit;
	ret = PostLVUserEvent(ev, lvdata);
	DSDisposePtr(lvdata);
}

API_PRETURN_HDR PhidgetVoltageInput_setOnSensorChangeLabviewHandler(PhidgetVoltageInputHandle ch,
  LVUserEventRef *lvEventRef) {
	TESTPTR(ch);
	if(lvEventRef && *lvEventRef)
		PhidgetVoltageInput_setOnSensorChangeHandler(ch,
		  PhidgetVoltageInput_OnSensorChange_LaviewHandler, lvEventRef);
	else
		PhidgetVoltageInput_setOnSensorChangeHandler(ch, NULL, NULL);

	return (EPHIDGET_OK);
}
#else
API_PRETURN_HDR PhidgetVoltageInput_setOnSensorChangeLabviewHandler(PhidgetVoltageInputHandle ch,
  LVUserEventRef *lvEventRef) {

	return (EPHIDGET_UNSUPPORTED);
}
#endif

#ifdef COMPILE_PHIDGETS_LABVIEW
void CCONV PhidgetVoltageInput_OnVoltageChange_LaviewHandler(PhidgetVoltageInputHandle ch,void *ctx,
  double voltage) {
	LVUserEventRef ev = *(LVUserEventRef *)ctx;
	MgErr ret = 0;
	lvPhidgetVoltageInputVoltageChangeArgs *lvdata;
	lvdata = (lvPhidgetVoltageInputVoltageChangeArgs *)DSNewPtr(sizeof(lvPhidgetVoltageInputVoltageChangeArgs));
	lvdata->ch = (intptr_t)ch;
	lvdata->voltage = voltage;
	ret = PostLVUserEvent(ev, lvdata);
	DSDisposePtr(lvdata);
}

API_PRETURN_HDR PhidgetVoltageInput_setOnVoltageChangeLabviewHandler(PhidgetVoltageInputHandle ch,
  LVUserEventRef *lvEventRef) {
	TESTPTR(ch);
	if(lvEventRef && *lvEventRef)
		PhidgetVoltageInput_setOnVoltageChangeHandler(ch,
		  PhidgetVoltageInput_OnVoltageChange_LaviewHandler, lvEventRef);
	else
		PhidgetVoltageInput_setOnVoltageChangeHandler(ch, NULL, NULL);

	return (EPHIDGET_OK);
}
#else
API_PRETURN_HDR PhidgetVoltageInput_setOnVoltageChangeLabviewHandler(PhidgetVoltageInputHandle ch,
  LVUserEventRef *lvEventRef) {

	return (EPHIDGET_UNSUPPORTED);
}
#endif

#ifdef COMPILE_PHIDGETS_LABVIEW
void CCONV PhidgetCapacitiveTouch_OnTouch_LaviewHandler(PhidgetCapacitiveTouchHandle ch,void *ctx,
  double touchValue) {
	LVUserEventRef ev = *(LVUserEventRef *)ctx;
	MgErr ret = 0;
	lvPhidgetCapacitiveTouchTouchArgs *lvdata;
	lvdata = (lvPhidgetCapacitiveTouchTouchArgs *)DSNewPtr(sizeof(lvPhidgetCapacitiveTouchTouchArgs));
	lvdata->ch = (intptr_t)ch;
	lvdata->touchValue = touchValue;
	ret = PostLVUserEvent(ev, lvdata);
	DSDisposePtr(lvdata);
}

API_PRETURN_HDR PhidgetCapacitiveTouch_setOnTouchLabviewHandler(PhidgetCapacitiveTouchHandle ch,
  LVUserEventRef *lvEventRef) {
	TESTPTR(ch);
	if(lvEventRef && *lvEventRef)
		PhidgetCapacitiveTouch_setOnTouchHandler(ch, PhidgetCapacitiveTouch_OnTouch_LaviewHandler,
		  lvEventRef);
	else
		PhidgetCapacitiveTouch_setOnTouchHandler(ch, NULL, NULL);

	return (EPHIDGET_OK);
}
#else
API_PRETURN_HDR PhidgetCapacitiveTouch_setOnTouchLabviewHandler(PhidgetCapacitiveTouchHandle ch,
  LVUserEventRef *lvEventRef) {

	return (EPHIDGET_UNSUPPORTED);
}
#endif

#ifdef COMPILE_PHIDGETS_LABVIEW
void CCONV PhidgetCapacitiveTouch_OnTouchEnd_LaviewHandler(PhidgetCapacitiveTouchHandle ch,void *ctx) {
	LVUserEventRef ev = *(LVUserEventRef *)ctx;
	MgErr ret = 0;
	lvPhidgetCapacitiveTouchTouchEndArgs *lvdata;
	lvdata = (lvPhidgetCapacitiveTouchTouchEndArgs *)DSNewPtr(sizeof(lvPhidgetCapacitiveTouchTouchEndArgs));
	lvdata->ch = (intptr_t)ch;
	ret = PostLVUserEvent(ev, lvdata);
	DSDisposePtr(lvdata);
}

API_PRETURN_HDR PhidgetCapacitiveTouch_setOnTouchEndLabviewHandler(PhidgetCapacitiveTouchHandle ch,
  LVUserEventRef *lvEventRef) {
	TESTPTR(ch);
	if(lvEventRef && *lvEventRef)
		PhidgetCapacitiveTouch_setOnTouchEndHandler(ch, PhidgetCapacitiveTouch_OnTouchEnd_LaviewHandler,
		  lvEventRef);
	else
		PhidgetCapacitiveTouch_setOnTouchEndHandler(ch, NULL, NULL);

	return (EPHIDGET_OK);
}
#else
API_PRETURN_HDR PhidgetCapacitiveTouch_setOnTouchEndLabviewHandler(PhidgetCapacitiveTouchHandle ch,
  LVUserEventRef *lvEventRef) {

	return (EPHIDGET_UNSUPPORTED);
}
#endif

#ifdef COMPILE_PHIDGETS_LABVIEW
void CCONV PhidgetRFID_OnTag_LaviewHandler(PhidgetRFIDHandle ch,void *ctx, const char *tag,
  PhidgetRFID_Protocol protocol) {
	LVUserEventRef ev = *(LVUserEventRef *)ctx;
	MgErr ret = 0;
	lvPhidgetRFIDTagArgs *lvdata;
	lvdata = (lvPhidgetRFIDTagArgs *)DSNewPtr(sizeof(lvPhidgetRFIDTagArgs));
	lvdata->ch = (intptr_t)ch;
	lvdata->tag = (LStrHandle)DSNewHandle(sizeof(int32_t) + sizeof(char) * (mos_strlen(tag) + 1));
	mos_strlcpy((char *)LStrBuf(*lvdata->tag), tag, mos_strlen(tag) + 1);
	LStrLen(*lvdata->tag) = (int32_t)mos_strlen(tag);
	lvdata->protocol = protocol;
	ret = PostLVUserEvent(ev, lvdata);
	DSDisposePtr(lvdata);
}

API_PRETURN_HDR PhidgetRFID_setOnTagLabviewHandler(PhidgetRFIDHandle ch, LVUserEventRef *lvEventRef) {
	TESTPTR(ch);
	if(lvEventRef && *lvEventRef)
		PhidgetRFID_setOnTagHandler(ch, PhidgetRFID_OnTag_LaviewHandler, lvEventRef);
	else
		PhidgetRFID_setOnTagHandler(ch, NULL, NULL);

	return (EPHIDGET_OK);
}
#else
API_PRETURN_HDR PhidgetRFID_setOnTagLabviewHandler(PhidgetRFIDHandle ch, LVUserEventRef *lvEventRef) {

	return (EPHIDGET_UNSUPPORTED);
}
#endif

#ifdef COMPILE_PHIDGETS_LABVIEW
void CCONV PhidgetRFID_OnTagLost_LaviewHandler(PhidgetRFIDHandle ch,void *ctx, const char *tag,
  PhidgetRFID_Protocol protocol) {
	LVUserEventRef ev = *(LVUserEventRef *)ctx;
	MgErr ret = 0;
	lvPhidgetRFIDTagLostArgs *lvdata;
	lvdata = (lvPhidgetRFIDTagLostArgs *)DSNewPtr(sizeof(lvPhidgetRFIDTagLostArgs));
	lvdata->ch = (intptr_t)ch;
	lvdata->tag = (LStrHandle)DSNewHandle(sizeof(int32_t) + sizeof(char) * (mos_strlen(tag) + 1));
	mos_strlcpy((char *)LStrBuf(*lvdata->tag), tag, mos_strlen(tag) + 1);
	LStrLen(*lvdata->tag) = (int32_t)mos_strlen(tag);
	lvdata->protocol = protocol;
	ret = PostLVUserEvent(ev, lvdata);
	DSDisposePtr(lvdata);
}

API_PRETURN_HDR PhidgetRFID_setOnTagLostLabviewHandler(PhidgetRFIDHandle ch,
  LVUserEventRef *lvEventRef) {
	TESTPTR(ch);
	if(lvEventRef && *lvEventRef)
		PhidgetRFID_setOnTagLostHandler(ch, PhidgetRFID_OnTagLost_LaviewHandler, lvEventRef);
	else
		PhidgetRFID_setOnTagLostHandler(ch, NULL, NULL);

	return (EPHIDGET_OK);
}
#else
API_PRETURN_HDR PhidgetRFID_setOnTagLostLabviewHandler(PhidgetRFIDHandle ch,
  LVUserEventRef *lvEventRef) {

	return (EPHIDGET_UNSUPPORTED);
}
#endif

#ifdef COMPILE_PHIDGETS_LABVIEW
void CCONV PhidgetGPS_OnHeadingChange_LaviewHandler(PhidgetGPSHandle ch,void *ctx, double heading,
  double velocity) {
	LVUserEventRef ev = *(LVUserEventRef *)ctx;
	MgErr ret = 0;
	lvPhidgetGPSHeadingChangeArgs *lvdata;
	lvdata = (lvPhidgetGPSHeadingChangeArgs *)DSNewPtr(sizeof(lvPhidgetGPSHeadingChangeArgs));
	lvdata->ch = (intptr_t)ch;
	lvdata->heading = heading;
	lvdata->velocity = velocity;
	ret = PostLVUserEvent(ev, lvdata);
	DSDisposePtr(lvdata);
}

API_PRETURN_HDR PhidgetGPS_setOnHeadingChangeLabviewHandler(PhidgetGPSHandle ch,
  LVUserEventRef *lvEventRef) {
	TESTPTR(ch);
	if(lvEventRef && *lvEventRef)
		PhidgetGPS_setOnHeadingChangeHandler(ch, PhidgetGPS_OnHeadingChange_LaviewHandler,
		  lvEventRef);
	else
		PhidgetGPS_setOnHeadingChangeHandler(ch, NULL, NULL);

	return (EPHIDGET_OK);
}
#else
API_PRETURN_HDR PhidgetGPS_setOnHeadingChangeLabviewHandler(PhidgetGPSHandle ch,
  LVUserEventRef *lvEventRef) {

	return (EPHIDGET_UNSUPPORTED);
}
#endif

#ifdef COMPILE_PHIDGETS_LABVIEW
void CCONV PhidgetGPS_OnPositionChange_LaviewHandler(PhidgetGPSHandle ch,void *ctx, double latitude,
  double longitude, double altitude) {
	LVUserEventRef ev = *(LVUserEventRef *)ctx;
	MgErr ret = 0;
	lvPhidgetGPSPositionChangeArgs *lvdata;
	lvdata = (lvPhidgetGPSPositionChangeArgs *)DSNewPtr(sizeof(lvPhidgetGPSPositionChangeArgs));
	lvdata->ch = (intptr_t)ch;
	lvdata->latitude = latitude;
	lvdata->longitude = longitude;
	lvdata->altitude = altitude;
	ret = PostLVUserEvent(ev, lvdata);
	DSDisposePtr(lvdata);
}

API_PRETURN_HDR PhidgetGPS_setOnPositionChangeLabviewHandler(PhidgetGPSHandle ch,
  LVUserEventRef *lvEventRef) {
	TESTPTR(ch);
	if(lvEventRef && *lvEventRef)
		PhidgetGPS_setOnPositionChangeHandler(ch, PhidgetGPS_OnPositionChange_LaviewHandler,
		  lvEventRef);
	else
		PhidgetGPS_setOnPositionChangeHandler(ch, NULL, NULL);

	return (EPHIDGET_OK);
}
#else
API_PRETURN_HDR PhidgetGPS_setOnPositionChangeLabviewHandler(PhidgetGPSHandle ch,
  LVUserEventRef *lvEventRef) {

	return (EPHIDGET_UNSUPPORTED);
}
#endif

#ifdef COMPILE_PHIDGETS_LABVIEW
void CCONV PhidgetGPS_OnPositionFixStateChange_LaviewHandler(PhidgetGPSHandle ch,void *ctx,
  int positionFixState) {
	LVUserEventRef ev = *(LVUserEventRef *)ctx;
	MgErr ret = 0;
	lvPhidgetGPSPositionFixStateChangeArgs *lvdata;
	lvdata = (lvPhidgetGPSPositionFixStateChangeArgs *)DSNewPtr(sizeof(lvPhidgetGPSPositionFixStateChangeArgs));
	lvdata->ch = (intptr_t)ch;
	lvdata->positionFixState = positionFixState;
	ret = PostLVUserEvent(ev, lvdata);
	DSDisposePtr(lvdata);
}

API_PRETURN_HDR PhidgetGPS_setOnPositionFixStateChangeLabviewHandler(PhidgetGPSHandle ch,
  LVUserEventRef *lvEventRef) {
	TESTPTR(ch);
	if(lvEventRef && *lvEventRef)
		PhidgetGPS_setOnPositionFixStateChangeHandler(ch,
		  PhidgetGPS_OnPositionFixStateChange_LaviewHandler, lvEventRef);
	else
		PhidgetGPS_setOnPositionFixStateChangeHandler(ch, NULL, NULL);

	return (EPHIDGET_OK);
}
#else
API_PRETURN_HDR PhidgetGPS_setOnPositionFixStateChangeLabviewHandler(PhidgetGPSHandle ch,
  LVUserEventRef *lvEventRef) {

	return (EPHIDGET_UNSUPPORTED);
}
#endif

#ifdef COMPILE_PHIDGETS_LABVIEW
void CCONV PhidgetGyroscope_OnAngularRateUpdate_LaviewHandler(PhidgetGyroscopeHandle ch,void *ctx,
  const double angularRate[3], double timestamp) {
	LVUserEventRef ev = *(LVUserEventRef *)ctx;
	MgErr ret = 0;
	lvPhidgetGyroscopeAngularRateUpdateArgs *lvdata;
	lvdata = (lvPhidgetGyroscopeAngularRateUpdateArgs *)DSNewPtr(sizeof(lvPhidgetGyroscopeAngularRateUpdateArgs));
	lvdata->ch = (intptr_t)ch;
	lvdata->angularRate = (lvArrHandle *)DSNewHandle(sizeof(size_t) + sizeof(double) * 3);
	memcpy(&((*(lvdata->angularRate))->data), angularRate, sizeof(double) * 3);
	(*(lvdata->angularRate))->length = (int32_t)3;
	lvdata->timestamp = timestamp;
	ret = PostLVUserEvent(ev, lvdata);
	DSDisposePtr(lvdata);
}

API_PRETURN_HDR PhidgetGyroscope_setOnAngularRateUpdateLabviewHandler(PhidgetGyroscopeHandle ch,
  LVUserEventRef *lvEventRef) {
	TESTPTR(ch);
	if(lvEventRef && *lvEventRef)
		PhidgetGyroscope_setOnAngularRateUpdateHandler(ch,
		  PhidgetGyroscope_OnAngularRateUpdate_LaviewHandler, lvEventRef);
	else
		PhidgetGyroscope_setOnAngularRateUpdateHandler(ch, NULL, NULL);

	return (EPHIDGET_OK);
}
#else
API_PRETURN_HDR PhidgetGyroscope_setOnAngularRateUpdateLabviewHandler(PhidgetGyroscopeHandle ch,
  LVUserEventRef *lvEventRef) {

	return (EPHIDGET_UNSUPPORTED);
}
#endif

#ifdef COMPILE_PHIDGETS_LABVIEW
void CCONV PhidgetMagnetometer_OnMagneticFieldChange_LaviewHandler(PhidgetMagnetometerHandle ch,void *ctx,
  const double magneticField[3], double timestamp) {
	LVUserEventRef ev = *(LVUserEventRef *)ctx;
	MgErr ret = 0;
	lvPhidgetMagnetometerMagneticFieldChangeArgs *lvdata;
	lvdata = (lvPhidgetMagnetometerMagneticFieldChangeArgs *)DSNewPtr(sizeof(lvPhidgetMagnetometerMagneticFieldChangeArgs));
	lvdata->ch = (intptr_t)ch;
	lvdata->magneticField = (lvArrHandle *)DSNewHandle(sizeof(size_t) + sizeof(double) * 3);
	memcpy(&((*(lvdata->magneticField))->data), magneticField, sizeof(double) * 3);
	(*(lvdata->magneticField))->length = (int32_t)3;
	lvdata->timestamp = timestamp;
	ret = PostLVUserEvent(ev, lvdata);
	DSDisposePtr(lvdata);
}

API_PRETURN_HDR PhidgetMagnetometer_setOnMagneticFieldChangeLabviewHandler(PhidgetMagnetometerHandle ch,
  LVUserEventRef *lvEventRef) {
	TESTPTR(ch);
	if(lvEventRef && *lvEventRef)
		PhidgetMagnetometer_setOnMagneticFieldChangeHandler(ch,
		  PhidgetMagnetometer_OnMagneticFieldChange_LaviewHandler, lvEventRef);
	else
		PhidgetMagnetometer_setOnMagneticFieldChangeHandler(ch, NULL, NULL);

	return (EPHIDGET_OK);
}
#else
API_PRETURN_HDR PhidgetMagnetometer_setOnMagneticFieldChangeLabviewHandler(PhidgetMagnetometerHandle ch,
  LVUserEventRef *lvEventRef) {

	return (EPHIDGET_UNSUPPORTED);
}
#endif

#ifdef COMPILE_PHIDGETS_LABVIEW
void CCONV PhidgetSpatial_OnAlgorithmData_LaviewHandler(PhidgetSpatialHandle ch,void *ctx,
  const double quaternion[4], double timestamp) {
	LVUserEventRef ev = *(LVUserEventRef *)ctx;
	MgErr ret = 0;
	lvPhidgetSpatialAlgorithmDataArgs *lvdata;
	lvdata = (lvPhidgetSpatialAlgorithmDataArgs *)DSNewPtr(sizeof(lvPhidgetSpatialAlgorithmDataArgs));
	lvdata->ch = (intptr_t)ch;
	lvdata->quaternion = (lvArrHandle *)DSNewHandle(sizeof(size_t) + sizeof(double) * 4);
	memcpy(&((*(lvdata->quaternion))->data), quaternion, sizeof(double) * 4);
	(*(lvdata->quaternion))->length = (int32_t)4;
	lvdata->timestamp = timestamp;
	ret = PostLVUserEvent(ev, lvdata);
	DSDisposePtr(lvdata);
}

API_PRETURN_HDR PhidgetSpatial_setOnAlgorithmDataLabviewHandler(PhidgetSpatialHandle ch,
  LVUserEventRef *lvEventRef) {
	TESTPTR(ch);
	if(lvEventRef && *lvEventRef)
		PhidgetSpatial_setOnAlgorithmDataHandler(ch, PhidgetSpatial_OnAlgorithmData_LaviewHandler,
		  lvEventRef);
	else
		PhidgetSpatial_setOnAlgorithmDataHandler(ch, NULL, NULL);

	return (EPHIDGET_OK);
}
#else
API_PRETURN_HDR PhidgetSpatial_setOnAlgorithmDataLabviewHandler(PhidgetSpatialHandle ch,
  LVUserEventRef *lvEventRef) {

	return (EPHIDGET_UNSUPPORTED);
}
#endif

#ifdef COMPILE_PHIDGETS_LABVIEW
void CCONV PhidgetSpatial_OnSpatialData_LaviewHandler(PhidgetSpatialHandle ch,void *ctx,
  const double acceleration[3], const double angularRate[3], const double magneticField[3], double timestamp) {
	LVUserEventRef ev = *(LVUserEventRef *)ctx;
	MgErr ret = 0;
	lvPhidgetSpatialSpatialDataArgs *lvdata;
	lvdata = (lvPhidgetSpatialSpatialDataArgs *)DSNewPtr(sizeof(lvPhidgetSpatialSpatialDataArgs));
	lvdata->ch = (intptr_t)ch;
	lvdata->acceleration = (lvArrHandle *)DSNewHandle(sizeof(size_t) + sizeof(double) * 3);
	memcpy(&((*(lvdata->acceleration))->data), acceleration, sizeof(double) * 3);
	(*(lvdata->acceleration))->length = (int32_t)3;
	lvdata->angularRate = (lvArrHandle *)DSNewHandle(sizeof(size_t) + sizeof(double) * 3);
	memcpy(&((*(lvdata->angularRate))->data), angularRate, sizeof(double) * 3);
	(*(lvdata->angularRate))->length = (int32_t)3;
	lvdata->magneticField = (lvArrHandle *)DSNewHandle(sizeof(size_t) + sizeof(double) * 3);
	memcpy(&((*(lvdata->magneticField))->data), magneticField, sizeof(double) * 3);
	(*(lvdata->magneticField))->length = (int32_t)3;
	lvdata->timestamp = timestamp;
	ret = PostLVUserEvent(ev, lvdata);
	DSDisposePtr(lvdata);
}

API_PRETURN_HDR PhidgetSpatial_setOnSpatialDataLabviewHandler(PhidgetSpatialHandle ch,
  LVUserEventRef *lvEventRef) {
	TESTPTR(ch);
	if(lvEventRef && *lvEventRef)
		PhidgetSpatial_setOnSpatialDataHandler(ch, PhidgetSpatial_OnSpatialData_LaviewHandler,
		  lvEventRef);
	else
		PhidgetSpatial_setOnSpatialDataHandler(ch, NULL, NULL);

	return (EPHIDGET_OK);
}
#else
API_PRETURN_HDR PhidgetSpatial_setOnSpatialDataLabviewHandler(PhidgetSpatialHandle ch,
  LVUserEventRef *lvEventRef) {

	return (EPHIDGET_UNSUPPORTED);
}
#endif

#ifdef COMPILE_PHIDGETS_LABVIEW
void CCONV PhidgetTemperatureSensor_OnTemperatureChange_LaviewHandler(PhidgetTemperatureSensorHandle ch,
 void *ctx, double temperature) {
	LVUserEventRef ev = *(LVUserEventRef *)ctx;
	MgErr ret = 0;
	lvPhidgetTemperatureSensorTemperatureChangeArgs *lvdata;
	lvdata = (lvPhidgetTemperatureSensorTemperatureChangeArgs *)DSNewPtr(sizeof(lvPhidgetTemperatureSensorTemperatureChangeArgs));
	lvdata->ch = (intptr_t)ch;
	lvdata->temperature = temperature;
	ret = PostLVUserEvent(ev, lvdata);
	DSDisposePtr(lvdata);
}

API_PRETURN_HDR PhidgetTemperatureSensor_setOnTemperatureChangeLabviewHandler(PhidgetTemperatureSensorHandle ch, LVUserEventRef *lvEventRef) {
	TESTPTR(ch);
	if(lvEventRef && *lvEventRef)
		PhidgetTemperatureSensor_setOnTemperatureChangeHandler(ch,
		  PhidgetTemperatureSensor_OnTemperatureChange_LaviewHandler, lvEventRef);
	else
		PhidgetTemperatureSensor_setOnTemperatureChangeHandler(ch, NULL, NULL);

	return (EPHIDGET_OK);
}
#else
API_PRETURN_HDR PhidgetTemperatureSensor_setOnTemperatureChangeLabviewHandler(PhidgetTemperatureSensorHandle ch, LVUserEventRef *lvEventRef) {

	return (EPHIDGET_UNSUPPORTED);
}
#endif

#ifdef COMPILE_PHIDGETS_LABVIEW
void CCONV PhidgetEncoder_OnPositionChange_LaviewHandler(PhidgetEncoderHandle ch,void *ctx,
  int positionChange, double timeChange, int indexTriggered) {
	LVUserEventRef ev = *(LVUserEventRef *)ctx;
	MgErr ret = 0;
	lvPhidgetEncoderPositionChangeArgs *lvdata;
	lvdata = (lvPhidgetEncoderPositionChangeArgs *)DSNewPtr(sizeof(lvPhidgetEncoderPositionChangeArgs));
	lvdata->ch = (intptr_t)ch;
	lvdata->positionChange = positionChange;
	lvdata->timeChange = timeChange;
	lvdata->indexTriggered = indexTriggered;
	ret = PostLVUserEvent(ev, lvdata);
	DSDisposePtr(lvdata);
}

API_PRETURN_HDR PhidgetEncoder_setOnPositionChangeLabviewHandler(PhidgetEncoderHandle ch,
  LVUserEventRef *lvEventRef) {
	TESTPTR(ch);
	if(lvEventRef && *lvEventRef)
		PhidgetEncoder_setOnPositionChangeHandler(ch, PhidgetEncoder_OnPositionChange_LaviewHandler,
		  lvEventRef);
	else
		PhidgetEncoder_setOnPositionChangeHandler(ch, NULL, NULL);

	return (EPHIDGET_OK);
}
#else
API_PRETURN_HDR PhidgetEncoder_setOnPositionChangeLabviewHandler(PhidgetEncoderHandle ch,
  LVUserEventRef *lvEventRef) {

	return (EPHIDGET_UNSUPPORTED);
}
#endif

#ifdef COMPILE_PHIDGETS_LABVIEW
void CCONV PhidgetFrequencyCounter_OnCountChange_LaviewHandler(PhidgetFrequencyCounterHandle ch,void *ctx,
  uint64_t counts, double timeChange) {
	LVUserEventRef ev = *(LVUserEventRef *)ctx;
	MgErr ret = 0;
	lvPhidgetFrequencyCounterCountChangeArgs *lvdata;
	lvdata = (lvPhidgetFrequencyCounterCountChangeArgs *)DSNewPtr(sizeof(lvPhidgetFrequencyCounterCountChangeArgs));
	lvdata->ch = (intptr_t)ch;
	lvdata->counts = counts;
	lvdata->timeChange = timeChange;
	ret = PostLVUserEvent(ev, lvdata);
	DSDisposePtr(lvdata);
}

API_PRETURN_HDR PhidgetFrequencyCounter_setOnCountChangeLabviewHandler(PhidgetFrequencyCounterHandle ch,
  LVUserEventRef *lvEventRef) {
	TESTPTR(ch);
	if(lvEventRef && *lvEventRef)
		PhidgetFrequencyCounter_setOnCountChangeHandler(ch,
		  PhidgetFrequencyCounter_OnCountChange_LaviewHandler, lvEventRef);
	else
		PhidgetFrequencyCounter_setOnCountChangeHandler(ch, NULL, NULL);

	return (EPHIDGET_OK);
}
#else
API_PRETURN_HDR PhidgetFrequencyCounter_setOnCountChangeLabviewHandler(PhidgetFrequencyCounterHandle ch,
  LVUserEventRef *lvEventRef) {

	return (EPHIDGET_UNSUPPORTED);
}
#endif

#ifdef COMPILE_PHIDGETS_LABVIEW
void CCONV PhidgetFrequencyCounter_OnFrequencyChange_LaviewHandler(PhidgetFrequencyCounterHandle ch,
 void *ctx, double frequency) {
	LVUserEventRef ev = *(LVUserEventRef *)ctx;
	MgErr ret = 0;
	lvPhidgetFrequencyCounterFrequencyChangeArgs *lvdata;
	lvdata = (lvPhidgetFrequencyCounterFrequencyChangeArgs *)DSNewPtr(sizeof(lvPhidgetFrequencyCounterFrequencyChangeArgs));
	lvdata->ch = (intptr_t)ch;
	lvdata->frequency = frequency;
	ret = PostLVUserEvent(ev, lvdata);
	DSDisposePtr(lvdata);
}

API_PRETURN_HDR PhidgetFrequencyCounter_setOnFrequencyChangeLabviewHandler(PhidgetFrequencyCounterHandle ch, LVUserEventRef *lvEventRef) {
	TESTPTR(ch);
	if(lvEventRef && *lvEventRef)
		PhidgetFrequencyCounter_setOnFrequencyChangeHandler(ch,
		  PhidgetFrequencyCounter_OnFrequencyChange_LaviewHandler, lvEventRef);
	else
		PhidgetFrequencyCounter_setOnFrequencyChangeHandler(ch, NULL, NULL);

	return (EPHIDGET_OK);
}
#else
API_PRETURN_HDR PhidgetFrequencyCounter_setOnFrequencyChangeLabviewHandler(PhidgetFrequencyCounterHandle ch, LVUserEventRef *lvEventRef) {

	return (EPHIDGET_UNSUPPORTED);
}
#endif

#ifdef COMPILE_PHIDGETS_LABVIEW
void CCONV PhidgetIR_OnCode_LaviewHandler(PhidgetIRHandle ch,void *ctx, const char *code,
  uint32_t bitCount, int isRepeat) {
	LVUserEventRef ev = *(LVUserEventRef *)ctx;
	MgErr ret = 0;
	lvPhidgetIRCodeArgs *lvdata;
	lvdata = (lvPhidgetIRCodeArgs *)DSNewPtr(sizeof(lvPhidgetIRCodeArgs));
	lvdata->ch = (intptr_t)ch;
	lvdata->code = (LStrHandle)DSNewHandle(sizeof(int32_t) + sizeof(char) * (mos_strlen(code) + 1));
	mos_strlcpy((char *)LStrBuf(*lvdata->code), code, mos_strlen(code) + 1);
	LStrLen(*lvdata->code) = (int32_t)mos_strlen(code);
	lvdata->bitCount = bitCount;
	lvdata->isRepeat = isRepeat;
	ret = PostLVUserEvent(ev, lvdata);
	DSDisposePtr(lvdata);
}

API_PRETURN_HDR PhidgetIR_setOnCodeLabviewHandler(PhidgetIRHandle ch, LVUserEventRef *lvEventRef) {
	TESTPTR(ch);
	if(lvEventRef && *lvEventRef)
		PhidgetIR_setOnCodeHandler(ch, PhidgetIR_OnCode_LaviewHandler, lvEventRef);
	else
		PhidgetIR_setOnCodeHandler(ch, NULL, NULL);

	return (EPHIDGET_OK);
}
#else
API_PRETURN_HDR PhidgetIR_setOnCodeLabviewHandler(PhidgetIRHandle ch, LVUserEventRef *lvEventRef) {

	return (EPHIDGET_UNSUPPORTED);
}
#endif

#ifdef COMPILE_PHIDGETS_LABVIEW
void CCONV PhidgetIR_OnLearn_LaviewHandler(PhidgetIRHandle ch,void *ctx, const char *code,
  PhidgetIR_CodeInfo *codeInfo) {
	LVUserEventRef ev = *(LVUserEventRef *)ctx;
	MgErr ret = 0;
	lvPhidgetIRLearnArgs *lvdata;
	lvdata = (lvPhidgetIRLearnArgs *)DSNewPtr(sizeof(lvPhidgetIRLearnArgs));
	lvdata->ch = (intptr_t)ch;
	lvdata->code = (LStrHandle)DSNewHandle(sizeof(int32_t) + sizeof(char) * (mos_strlen(code) + 1));
	mos_strlcpy((char *)LStrBuf(*lvdata->code), code, mos_strlen(code) + 1);
	LStrLen(*lvdata->code) = (int32_t)mos_strlen(code);
	lvdata->codeInfo = *codeInfo;
	ret = PostLVUserEvent(ev, lvdata);
	DSDisposePtr(lvdata);
}

API_PRETURN_HDR PhidgetIR_setOnLearnLabviewHandler(PhidgetIRHandle ch, LVUserEventRef *lvEventRef) {
	TESTPTR(ch);
	if(lvEventRef && *lvEventRef)
		PhidgetIR_setOnLearnHandler(ch, PhidgetIR_OnLearn_LaviewHandler, lvEventRef);
	else
		PhidgetIR_setOnLearnHandler(ch, NULL, NULL);

	return (EPHIDGET_OK);
}
#else
API_PRETURN_HDR PhidgetIR_setOnLearnLabviewHandler(PhidgetIRHandle ch, LVUserEventRef *lvEventRef) {

	return (EPHIDGET_UNSUPPORTED);
}
#endif

#ifdef COMPILE_PHIDGETS_LABVIEW
void CCONV PhidgetIR_OnRawData_LaviewHandler(PhidgetIRHandle ch,void *ctx, const uint32_t *data,
  size_t dataLen) {
	LVUserEventRef ev = *(LVUserEventRef *)ctx;
	MgErr ret = 0;
	lvPhidgetIRRawDataArgs *lvdata;
	lvdata = (lvPhidgetIRRawDataArgs *)DSNewPtr(sizeof(lvPhidgetIRRawDataArgs));
	lvdata->ch = (intptr_t)ch;
	lvdata->data = (lvArrHandle *)DSNewHandle(sizeof(size_t) + sizeof(uint32_t) * dataLen);
	memcpy(&((*(lvdata->data))->data), data, sizeof(uint32_t) * dataLen);
	(*(lvdata->data))->length = (int32_t)dataLen;
	ret = PostLVUserEvent(ev, lvdata);
	DSDisposePtr(lvdata);
}

API_PRETURN_HDR PhidgetIR_setOnRawDataLabviewHandler(PhidgetIRHandle ch, LVUserEventRef *lvEventRef) {
	TESTPTR(ch);
	if(lvEventRef && *lvEventRef)
		PhidgetIR_setOnRawDataHandler(ch, PhidgetIR_OnRawData_LaviewHandler, lvEventRef);
	else
		PhidgetIR_setOnRawDataHandler(ch, NULL, NULL);

	return (EPHIDGET_OK);
}
#else
API_PRETURN_HDR PhidgetIR_setOnRawDataLabviewHandler(PhidgetIRHandle ch, LVUserEventRef *lvEventRef) {

	return (EPHIDGET_UNSUPPORTED);
}
#endif

#ifdef COMPILE_PHIDGETS_LABVIEW
void CCONV PhidgetPHSensor_OnPHChange_LaviewHandler(PhidgetPHSensorHandle ch,void *ctx, double PH) {
	LVUserEventRef ev = *(LVUserEventRef *)ctx;
	MgErr ret = 0;
	lvPhidgetPHSensorPHChangeArgs *lvdata;
	lvdata = (lvPhidgetPHSensorPHChangeArgs *)DSNewPtr(sizeof(lvPhidgetPHSensorPHChangeArgs));
	lvdata->ch = (intptr_t)ch;
	lvdata->PH = PH;
	ret = PostLVUserEvent(ev, lvdata);
	DSDisposePtr(lvdata);
}

API_PRETURN_HDR PhidgetPHSensor_setOnPHChangeLabviewHandler(PhidgetPHSensorHandle ch,
  LVUserEventRef *lvEventRef) {
	TESTPTR(ch);
	if(lvEventRef && *lvEventRef)
		PhidgetPHSensor_setOnPHChangeHandler(ch, PhidgetPHSensor_OnPHChange_LaviewHandler,
		  lvEventRef);
	else
		PhidgetPHSensor_setOnPHChangeHandler(ch, NULL, NULL);

	return (EPHIDGET_OK);
}
#else
API_PRETURN_HDR PhidgetPHSensor_setOnPHChangeLabviewHandler(PhidgetPHSensorHandle ch,
  LVUserEventRef *lvEventRef) {

	return (EPHIDGET_UNSUPPORTED);
}
#endif

#ifdef COMPILE_PHIDGETS_LABVIEW
void CCONV PhidgetDCMotor_OnBackEMFChange_LaviewHandler(PhidgetDCMotorHandle ch,void *ctx,
  double backEMF) {
	LVUserEventRef ev = *(LVUserEventRef *)ctx;
	MgErr ret = 0;
	lvPhidgetDCMotorBackEMFChangeArgs *lvdata;
	lvdata = (lvPhidgetDCMotorBackEMFChangeArgs *)DSNewPtr(sizeof(lvPhidgetDCMotorBackEMFChangeArgs));
	lvdata->ch = (intptr_t)ch;
	lvdata->backEMF = backEMF;
	ret = PostLVUserEvent(ev, lvdata);
	DSDisposePtr(lvdata);
}

API_PRETURN_HDR PhidgetDCMotor_setOnBackEMFChangeLabviewHandler(PhidgetDCMotorHandle ch,
  LVUserEventRef *lvEventRef) {
	TESTPTR(ch);
	if(lvEventRef && *lvEventRef)
		PhidgetDCMotor_setOnBackEMFChangeHandler(ch, PhidgetDCMotor_OnBackEMFChange_LaviewHandler,
		  lvEventRef);
	else
		PhidgetDCMotor_setOnBackEMFChangeHandler(ch, NULL, NULL);

	return (EPHIDGET_OK);
}
#else
API_PRETURN_HDR PhidgetDCMotor_setOnBackEMFChangeLabviewHandler(PhidgetDCMotorHandle ch,
  LVUserEventRef *lvEventRef) {

	return (EPHIDGET_UNSUPPORTED);
}
#endif

#ifdef COMPILE_PHIDGETS_LABVIEW
void CCONV PhidgetDCMotor_OnBrakingStrengthChange_LaviewHandler(PhidgetDCMotorHandle ch,void *ctx,
  double brakingStrength) {
	LVUserEventRef ev = *(LVUserEventRef *)ctx;
	MgErr ret = 0;
	lvPhidgetDCMotorBrakingStrengthChangeArgs *lvdata;
	lvdata = (lvPhidgetDCMotorBrakingStrengthChangeArgs *)DSNewPtr(sizeof(lvPhidgetDCMotorBrakingStrengthChangeArgs));
	lvdata->ch = (intptr_t)ch;
	lvdata->brakingStrength = brakingStrength;
	ret = PostLVUserEvent(ev, lvdata);
	DSDisposePtr(lvdata);
}

API_PRETURN_HDR PhidgetDCMotor_setOnBrakingStrengthChangeLabviewHandler(PhidgetDCMotorHandle ch,
  LVUserEventRef *lvEventRef) {
	TESTPTR(ch);
	if(lvEventRef && *lvEventRef)
		PhidgetDCMotor_setOnBrakingStrengthChangeHandler(ch,
		  PhidgetDCMotor_OnBrakingStrengthChange_LaviewHandler, lvEventRef);
	else
		PhidgetDCMotor_setOnBrakingStrengthChangeHandler(ch, NULL, NULL);

	return (EPHIDGET_OK);
}
#else
API_PRETURN_HDR PhidgetDCMotor_setOnBrakingStrengthChangeLabviewHandler(PhidgetDCMotorHandle ch,
  LVUserEventRef *lvEventRef) {

	return (EPHIDGET_UNSUPPORTED);
}
#endif

#ifdef COMPILE_PHIDGETS_LABVIEW
void CCONV PhidgetDCMotor_OnVelocityUpdate_LaviewHandler(PhidgetDCMotorHandle ch,void *ctx,
  double velocity) {
	LVUserEventRef ev = *(LVUserEventRef *)ctx;
	MgErr ret = 0;
	lvPhidgetDCMotorVelocityUpdateArgs *lvdata;
	lvdata = (lvPhidgetDCMotorVelocityUpdateArgs *)DSNewPtr(sizeof(lvPhidgetDCMotorVelocityUpdateArgs));
	lvdata->ch = (intptr_t)ch;
	lvdata->velocity = velocity;
	ret = PostLVUserEvent(ev, lvdata);
	DSDisposePtr(lvdata);
}

API_PRETURN_HDR PhidgetDCMotor_setOnVelocityUpdateLabviewHandler(PhidgetDCMotorHandle ch,
  LVUserEventRef *lvEventRef) {
	TESTPTR(ch);
	if(lvEventRef && *lvEventRef)
		PhidgetDCMotor_setOnVelocityUpdateHandler(ch, PhidgetDCMotor_OnVelocityUpdate_LaviewHandler,
		  lvEventRef);
	else
		PhidgetDCMotor_setOnVelocityUpdateHandler(ch, NULL, NULL);

	return (EPHIDGET_OK);
}
#else
API_PRETURN_HDR PhidgetDCMotor_setOnVelocityUpdateLabviewHandler(PhidgetDCMotorHandle ch,
  LVUserEventRef *lvEventRef) {

	return (EPHIDGET_UNSUPPORTED);
}
#endif

#ifdef COMPILE_PHIDGETS_LABVIEW
void CCONV PhidgetCurrentInput_OnCurrentChange_LaviewHandler(PhidgetCurrentInputHandle ch,void *ctx,
  double current) {
	LVUserEventRef ev = *(LVUserEventRef *)ctx;
	MgErr ret = 0;
	lvPhidgetCurrentInputCurrentChangeArgs *lvdata;
	lvdata = (lvPhidgetCurrentInputCurrentChangeArgs *)DSNewPtr(sizeof(lvPhidgetCurrentInputCurrentChangeArgs));
	lvdata->ch = (intptr_t)ch;
	lvdata->current = current;
	ret = PostLVUserEvent(ev, lvdata);
	DSDisposePtr(lvdata);
}

API_PRETURN_HDR PhidgetCurrentInput_setOnCurrentChangeLabviewHandler(PhidgetCurrentInputHandle ch,
  LVUserEventRef *lvEventRef) {
	TESTPTR(ch);
	if(lvEventRef && *lvEventRef)
		PhidgetCurrentInput_setOnCurrentChangeHandler(ch,
		  PhidgetCurrentInput_OnCurrentChange_LaviewHandler, lvEventRef);
	else
		PhidgetCurrentInput_setOnCurrentChangeHandler(ch, NULL, NULL);

	return (EPHIDGET_OK);
}
#else
API_PRETURN_HDR PhidgetCurrentInput_setOnCurrentChangeLabviewHandler(PhidgetCurrentInputHandle ch,
  LVUserEventRef *lvEventRef) {

	return (EPHIDGET_UNSUPPORTED);
}
#endif

#ifdef COMPILE_PHIDGETS_LABVIEW
void CCONV PhidgetStepper_OnPositionChange_LaviewHandler(PhidgetStepperHandle ch,void *ctx,
  double position) {
	LVUserEventRef ev = *(LVUserEventRef *)ctx;
	MgErr ret = 0;
	lvPhidgetStepperPositionChangeArgs *lvdata;
	lvdata = (lvPhidgetStepperPositionChangeArgs *)DSNewPtr(sizeof(lvPhidgetStepperPositionChangeArgs));
	lvdata->ch = (intptr_t)ch;
	lvdata->position = position;
	ret = PostLVUserEvent(ev, lvdata);
	DSDisposePtr(lvdata);
}

API_PRETURN_HDR PhidgetStepper_setOnPositionChangeLabviewHandler(PhidgetStepperHandle ch,
  LVUserEventRef *lvEventRef) {
	TESTPTR(ch);
	if(lvEventRef && *lvEventRef)
		PhidgetStepper_setOnPositionChangeHandler(ch, PhidgetStepper_OnPositionChange_LaviewHandler,
		  lvEventRef);
	else
		PhidgetStepper_setOnPositionChangeHandler(ch, NULL, NULL);

	return (EPHIDGET_OK);
}
#else
API_PRETURN_HDR PhidgetStepper_setOnPositionChangeLabviewHandler(PhidgetStepperHandle ch,
  LVUserEventRef *lvEventRef) {

	return (EPHIDGET_UNSUPPORTED);
}
#endif

#ifdef COMPILE_PHIDGETS_LABVIEW
void CCONV PhidgetStepper_OnStopped_LaviewHandler(PhidgetStepperHandle ch,void *ctx) {
	LVUserEventRef ev = *(LVUserEventRef *)ctx;
	MgErr ret = 0;
	lvPhidgetStepperStoppedArgs *lvdata;
	lvdata = (lvPhidgetStepperStoppedArgs *)DSNewPtr(sizeof(lvPhidgetStepperStoppedArgs));
	lvdata->ch = (intptr_t)ch;
	ret = PostLVUserEvent(ev, lvdata);
	DSDisposePtr(lvdata);
}

API_PRETURN_HDR PhidgetStepper_setOnStoppedLabviewHandler(PhidgetStepperHandle ch,
  LVUserEventRef *lvEventRef) {
	TESTPTR(ch);
	if(lvEventRef && *lvEventRef)
		PhidgetStepper_setOnStoppedHandler(ch, PhidgetStepper_OnStopped_LaviewHandler, lvEventRef);
	else
		PhidgetStepper_setOnStoppedHandler(ch, NULL, NULL);

	return (EPHIDGET_OK);
}
#else
API_PRETURN_HDR PhidgetStepper_setOnStoppedLabviewHandler(PhidgetStepperHandle ch,
  LVUserEventRef *lvEventRef) {

	return (EPHIDGET_UNSUPPORTED);
}
#endif

#ifdef COMPILE_PHIDGETS_LABVIEW
void CCONV PhidgetStepper_OnVelocityChange_LaviewHandler(PhidgetStepperHandle ch,void *ctx,
  double velocity) {
	LVUserEventRef ev = *(LVUserEventRef *)ctx;
	MgErr ret = 0;
	lvPhidgetStepperVelocityChangeArgs *lvdata;
	lvdata = (lvPhidgetStepperVelocityChangeArgs *)DSNewPtr(sizeof(lvPhidgetStepperVelocityChangeArgs));
	lvdata->ch = (intptr_t)ch;
	lvdata->velocity = velocity;
	ret = PostLVUserEvent(ev, lvdata);
	DSDisposePtr(lvdata);
}

API_PRETURN_HDR PhidgetStepper_setOnVelocityChangeLabviewHandler(PhidgetStepperHandle ch,
  LVUserEventRef *lvEventRef) {
	TESTPTR(ch);
	if(lvEventRef && *lvEventRef)
		PhidgetStepper_setOnVelocityChangeHandler(ch, PhidgetStepper_OnVelocityChange_LaviewHandler,
		  lvEventRef);
	else
		PhidgetStepper_setOnVelocityChangeHandler(ch, NULL, NULL);

	return (EPHIDGET_OK);
}
#else
API_PRETURN_HDR PhidgetStepper_setOnVelocityChangeLabviewHandler(PhidgetStepperHandle ch,
  LVUserEventRef *lvEventRef) {

	return (EPHIDGET_UNSUPPORTED);
}
#endif

#ifdef COMPILE_PHIDGETS_LABVIEW
void CCONV PhidgetDataAdapter_OnPacket_LaviewHandler(PhidgetDataAdapterHandle ch,void *ctx,
  const uint8_t *data, size_t dataLen, PhidgetDataAdapter_PacketErrorCode error) {
	LVUserEventRef ev = *(LVUserEventRef *)ctx;
	MgErr ret = 0;
	lvPhidgetDataAdapterPacketArgs *lvdata;
	lvdata = (lvPhidgetDataAdapterPacketArgs *)DSNewPtr(sizeof(lvPhidgetDataAdapterPacketArgs));
	lvdata->ch = (intptr_t)ch;
	lvdata->data = (lvArrHandle *)DSNewHandle(sizeof(size_t) + sizeof(uint8_t) * dataLen);
	memcpy(&((*(lvdata->data))->data), data, sizeof(uint8_t) * dataLen);
	(*(lvdata->data))->length = (int32_t)dataLen;
	lvdata->error = error;
	ret = PostLVUserEvent(ev, lvdata);
	DSDisposePtr(lvdata);
}

API_PRETURN_HDR PhidgetDataAdapter_setOnPacketLabviewHandler(PhidgetDataAdapterHandle ch,
  LVUserEventRef *lvEventRef) {
	TESTPTR(ch);
	if(lvEventRef && *lvEventRef)
		PhidgetDataAdapter_setOnPacketHandler(ch, PhidgetDataAdapter_OnPacket_LaviewHandler,
		  lvEventRef);
	else
		PhidgetDataAdapter_setOnPacketHandler(ch, NULL, NULL);

	return (EPHIDGET_OK);
}
#else
API_PRETURN_HDR PhidgetDataAdapter_setOnPacketLabviewHandler(PhidgetDataAdapterHandle ch,
  LVUserEventRef *lvEventRef) {

	return (EPHIDGET_UNSUPPORTED);
}
#endif

#ifdef COMPILE_PHIDGETS_LABVIEW
void CCONV PhidgetMotorPositionController_OnDutyCycleUpdate_LaviewHandler(PhidgetMotorPositionControllerHandle ch,void *ctx, double dutyCycle) {
	LVUserEventRef ev = *(LVUserEventRef *)ctx;
	MgErr ret = 0;
	lvPhidgetMotorPositionControllerDutyCycleUpdateArgs *lvdata;
	lvdata = (lvPhidgetMotorPositionControllerDutyCycleUpdateArgs *)DSNewPtr(sizeof(lvPhidgetMotorPositionControllerDutyCycleUpdateArgs));
	lvdata->ch = (intptr_t)ch;
	lvdata->dutyCycle = dutyCycle;
	ret = PostLVUserEvent(ev, lvdata);
	DSDisposePtr(lvdata);
}

API_PRETURN_HDR PhidgetMotorPositionController_setOnDutyCycleUpdateLabviewHandler(PhidgetMotorPositionControllerHandle ch, LVUserEventRef *lvEventRef) {
	TESTPTR(ch);
	if(lvEventRef && *lvEventRef)
		PhidgetMotorPositionController_setOnDutyCycleUpdateHandler(ch,
		  PhidgetMotorPositionController_OnDutyCycleUpdate_LaviewHandler, lvEventRef);
	else
		PhidgetMotorPositionController_setOnDutyCycleUpdateHandler(ch, NULL, NULL);

	return (EPHIDGET_OK);
}
#else
API_PRETURN_HDR PhidgetMotorPositionController_setOnDutyCycleUpdateLabviewHandler(PhidgetMotorPositionControllerHandle ch, LVUserEventRef *lvEventRef) {

	return (EPHIDGET_UNSUPPORTED);
}
#endif

#ifdef COMPILE_PHIDGETS_LABVIEW
void CCONV PhidgetMotorPositionController_OnPositionChange_LaviewHandler(PhidgetMotorPositionControllerHandle ch,void *ctx, double position) {
	LVUserEventRef ev = *(LVUserEventRef *)ctx;
	MgErr ret = 0;
	lvPhidgetMotorPositionControllerPositionChangeArgs *lvdata;
	lvdata = (lvPhidgetMotorPositionControllerPositionChangeArgs *)DSNewPtr(sizeof(lvPhidgetMotorPositionControllerPositionChangeArgs));
	lvdata->ch = (intptr_t)ch;
	lvdata->position = position;
	ret = PostLVUserEvent(ev, lvdata);
	DSDisposePtr(lvdata);
}

API_PRETURN_HDR PhidgetMotorPositionController_setOnPositionChangeLabviewHandler(PhidgetMotorPositionControllerHandle ch, LVUserEventRef *lvEventRef) {
	TESTPTR(ch);
	if(lvEventRef && *lvEventRef)
		PhidgetMotorPositionController_setOnPositionChangeHandler(ch,
		  PhidgetMotorPositionController_OnPositionChange_LaviewHandler, lvEventRef);
	else
		PhidgetMotorPositionController_setOnPositionChangeHandler(ch, NULL, NULL);

	return (EPHIDGET_OK);
}
#else
API_PRETURN_HDR PhidgetMotorPositionController_setOnPositionChangeLabviewHandler(PhidgetMotorPositionControllerHandle ch, LVUserEventRef *lvEventRef) {

	return (EPHIDGET_UNSUPPORTED);
}
#endif

#ifdef COMPILE_PHIDGETS_LABVIEW
void CCONV PhidgetBLDCMotor_OnBrakingStrengthChange_LaviewHandler(PhidgetBLDCMotorHandle ch,void *ctx,
  double brakingStrength) {
	LVUserEventRef ev = *(LVUserEventRef *)ctx;
	MgErr ret = 0;
	lvPhidgetBLDCMotorBrakingStrengthChangeArgs *lvdata;
	lvdata = (lvPhidgetBLDCMotorBrakingStrengthChangeArgs *)DSNewPtr(sizeof(lvPhidgetBLDCMotorBrakingStrengthChangeArgs));
	lvdata->ch = (intptr_t)ch;
	lvdata->brakingStrength = brakingStrength;
	ret = PostLVUserEvent(ev, lvdata);
	DSDisposePtr(lvdata);
}

API_PRETURN_HDR PhidgetBLDCMotor_setOnBrakingStrengthChangeLabviewHandler(PhidgetBLDCMotorHandle ch,
  LVUserEventRef *lvEventRef) {
	TESTPTR(ch);
	if(lvEventRef && *lvEventRef)
		PhidgetBLDCMotor_setOnBrakingStrengthChangeHandler(ch,
		  PhidgetBLDCMotor_OnBrakingStrengthChange_LaviewHandler, lvEventRef);
	else
		PhidgetBLDCMotor_setOnBrakingStrengthChangeHandler(ch, NULL, NULL);

	return (EPHIDGET_OK);
}
#else
API_PRETURN_HDR PhidgetBLDCMotor_setOnBrakingStrengthChangeLabviewHandler(PhidgetBLDCMotorHandle ch,
  LVUserEventRef *lvEventRef) {

	return (EPHIDGET_UNSUPPORTED);
}
#endif

#ifdef COMPILE_PHIDGETS_LABVIEW
void CCONV PhidgetBLDCMotor_OnPositionChange_LaviewHandler(PhidgetBLDCMotorHandle ch,void *ctx,
  double position) {
	LVUserEventRef ev = *(LVUserEventRef *)ctx;
	MgErr ret = 0;
	lvPhidgetBLDCMotorPositionChangeArgs *lvdata;
	lvdata = (lvPhidgetBLDCMotorPositionChangeArgs *)DSNewPtr(sizeof(lvPhidgetBLDCMotorPositionChangeArgs));
	lvdata->ch = (intptr_t)ch;
	lvdata->position = position;
	ret = PostLVUserEvent(ev, lvdata);
	DSDisposePtr(lvdata);
}

API_PRETURN_HDR PhidgetBLDCMotor_setOnPositionChangeLabviewHandler(PhidgetBLDCMotorHandle ch,
  LVUserEventRef *lvEventRef) {
	TESTPTR(ch);
	if(lvEventRef && *lvEventRef)
		PhidgetBLDCMotor_setOnPositionChangeHandler(ch, PhidgetBLDCMotor_OnPositionChange_LaviewHandler,
		  lvEventRef);
	else
		PhidgetBLDCMotor_setOnPositionChangeHandler(ch, NULL, NULL);

	return (EPHIDGET_OK);
}
#else
API_PRETURN_HDR PhidgetBLDCMotor_setOnPositionChangeLabviewHandler(PhidgetBLDCMotorHandle ch,
  LVUserEventRef *lvEventRef) {

	return (EPHIDGET_UNSUPPORTED);
}
#endif

#ifdef COMPILE_PHIDGETS_LABVIEW
void CCONV PhidgetBLDCMotor_OnVelocityUpdate_LaviewHandler(PhidgetBLDCMotorHandle ch,void *ctx,
  double velocity) {
	LVUserEventRef ev = *(LVUserEventRef *)ctx;
	MgErr ret = 0;
	lvPhidgetBLDCMotorVelocityUpdateArgs *lvdata;
	lvdata = (lvPhidgetBLDCMotorVelocityUpdateArgs *)DSNewPtr(sizeof(lvPhidgetBLDCMotorVelocityUpdateArgs));
	lvdata->ch = (intptr_t)ch;
	lvdata->velocity = velocity;
	ret = PostLVUserEvent(ev, lvdata);
	DSDisposePtr(lvdata);
}

API_PRETURN_HDR PhidgetBLDCMotor_setOnVelocityUpdateLabviewHandler(PhidgetBLDCMotorHandle ch,
  LVUserEventRef *lvEventRef) {
	TESTPTR(ch);
	if(lvEventRef && *lvEventRef)
		PhidgetBLDCMotor_setOnVelocityUpdateHandler(ch, PhidgetBLDCMotor_OnVelocityUpdate_LaviewHandler,
		  lvEventRef);
	else
		PhidgetBLDCMotor_setOnVelocityUpdateHandler(ch, NULL, NULL);

	return (EPHIDGET_OK);
}
#else
API_PRETURN_HDR PhidgetBLDCMotor_setOnVelocityUpdateLabviewHandler(PhidgetBLDCMotorHandle ch,
  LVUserEventRef *lvEventRef) {

	return (EPHIDGET_UNSUPPORTED);
}
#endif

#ifdef COMPILE_PHIDGETS_LABVIEW
void CCONV PhidgetDistanceSensor_OnDistanceChange_LaviewHandler(PhidgetDistanceSensorHandle ch,void *ctx,
  uint32_t distance) {
	LVUserEventRef ev = *(LVUserEventRef *)ctx;
	MgErr ret = 0;
	lvPhidgetDistanceSensorDistanceChangeArgs *lvdata;
	lvdata = (lvPhidgetDistanceSensorDistanceChangeArgs *)DSNewPtr(sizeof(lvPhidgetDistanceSensorDistanceChangeArgs));
	lvdata->ch = (intptr_t)ch;
	lvdata->distance = distance;
	ret = PostLVUserEvent(ev, lvdata);
	DSDisposePtr(lvdata);
}

API_PRETURN_HDR PhidgetDistanceSensor_setOnDistanceChangeLabviewHandler(PhidgetDistanceSensorHandle ch,
  LVUserEventRef *lvEventRef) {
	TESTPTR(ch);
	if(lvEventRef && *lvEventRef)
		PhidgetDistanceSensor_setOnDistanceChangeHandler(ch,
		  PhidgetDistanceSensor_OnDistanceChange_LaviewHandler, lvEventRef);
	else
		PhidgetDistanceSensor_setOnDistanceChangeHandler(ch, NULL, NULL);

	return (EPHIDGET_OK);
}
#else
API_PRETURN_HDR PhidgetDistanceSensor_setOnDistanceChangeLabviewHandler(PhidgetDistanceSensorHandle ch,
  LVUserEventRef *lvEventRef) {

	return (EPHIDGET_UNSUPPORTED);
}
#endif

#ifdef COMPILE_PHIDGETS_LABVIEW
void CCONV PhidgetDistanceSensor_OnSonarReflectionsUpdate_LaviewHandler(PhidgetDistanceSensorHandle ch,
 void *ctx, const uint32_t distances[8], const uint32_t amplitudes[8], uint32_t count) {
	LVUserEventRef ev = *(LVUserEventRef *)ctx;
	MgErr ret = 0;
	lvPhidgetDistanceSensorSonarReflectionsUpdateArgs *lvdata;
	lvdata = (lvPhidgetDistanceSensorSonarReflectionsUpdateArgs *)DSNewPtr(sizeof(lvPhidgetDistanceSensorSonarReflectionsUpdateArgs));
	lvdata->ch = (intptr_t)ch;
	lvdata->distances = (lvArrHandle *)DSNewHandle(sizeof(size_t) + sizeof(uint32_t) * 8);
	memcpy(&((*(lvdata->distances))->data), distances, sizeof(uint32_t) * 8);
	(*(lvdata->distances))->length = (int32_t)8;
	lvdata->amplitudes = (lvArrHandle *)DSNewHandle(sizeof(size_t) + sizeof(uint32_t) * 8);
	memcpy(&((*(lvdata->amplitudes))->data), amplitudes, sizeof(uint32_t) * 8);
	(*(lvdata->amplitudes))->length = (int32_t)8;
	lvdata->count = count;
	ret = PostLVUserEvent(ev, lvdata);
	DSDisposePtr(lvdata);
}

API_PRETURN_HDR PhidgetDistanceSensor_setOnSonarReflectionsUpdateLabviewHandler(PhidgetDistanceSensorHandle ch, LVUserEventRef *lvEventRef) {
	TESTPTR(ch);
	if(lvEventRef && *lvEventRef)
		PhidgetDistanceSensor_setOnSonarReflectionsUpdateHandler(ch,
		  PhidgetDistanceSensor_OnSonarReflectionsUpdate_LaviewHandler, lvEventRef);
	else
		PhidgetDistanceSensor_setOnSonarReflectionsUpdateHandler(ch, NULL, NULL);

	return (EPHIDGET_OK);
}
#else
API_PRETURN_HDR PhidgetDistanceSensor_setOnSonarReflectionsUpdateLabviewHandler(PhidgetDistanceSensorHandle ch, LVUserEventRef *lvEventRef) {

	return (EPHIDGET_UNSUPPORTED);
}
#endif

#ifdef COMPILE_PHIDGETS_LABVIEW
void CCONV PhidgetHumiditySensor_OnHumidityChange_LaviewHandler(PhidgetHumiditySensorHandle ch,void *ctx,
  double humidity) {
	LVUserEventRef ev = *(LVUserEventRef *)ctx;
	MgErr ret = 0;
	lvPhidgetHumiditySensorHumidityChangeArgs *lvdata;
	lvdata = (lvPhidgetHumiditySensorHumidityChangeArgs *)DSNewPtr(sizeof(lvPhidgetHumiditySensorHumidityChangeArgs));
	lvdata->ch = (intptr_t)ch;
	lvdata->humidity = humidity;
	ret = PostLVUserEvent(ev, lvdata);
	DSDisposePtr(lvdata);
}

API_PRETURN_HDR PhidgetHumiditySensor_setOnHumidityChangeLabviewHandler(PhidgetHumiditySensorHandle ch,
  LVUserEventRef *lvEventRef) {
	TESTPTR(ch);
	if(lvEventRef && *lvEventRef)
		PhidgetHumiditySensor_setOnHumidityChangeHandler(ch,
		  PhidgetHumiditySensor_OnHumidityChange_LaviewHandler, lvEventRef);
	else
		PhidgetHumiditySensor_setOnHumidityChangeHandler(ch, NULL, NULL);

	return (EPHIDGET_OK);
}
#else
API_PRETURN_HDR PhidgetHumiditySensor_setOnHumidityChangeLabviewHandler(PhidgetHumiditySensorHandle ch,
  LVUserEventRef *lvEventRef) {

	return (EPHIDGET_UNSUPPORTED);
}
#endif

#ifdef COMPILE_PHIDGETS_LABVIEW
void CCONV PhidgetLightSensor_OnIlluminanceChange_LaviewHandler(PhidgetLightSensorHandle ch,void *ctx,
  double illuminance) {
	LVUserEventRef ev = *(LVUserEventRef *)ctx;
	MgErr ret = 0;
	lvPhidgetLightSensorIlluminanceChangeArgs *lvdata;
	lvdata = (lvPhidgetLightSensorIlluminanceChangeArgs *)DSNewPtr(sizeof(lvPhidgetLightSensorIlluminanceChangeArgs));
	lvdata->ch = (intptr_t)ch;
	lvdata->illuminance = illuminance;
	ret = PostLVUserEvent(ev, lvdata);
	DSDisposePtr(lvdata);
}

API_PRETURN_HDR PhidgetLightSensor_setOnIlluminanceChangeLabviewHandler(PhidgetLightSensorHandle ch,
  LVUserEventRef *lvEventRef) {
	TESTPTR(ch);
	if(lvEventRef && *lvEventRef)
		PhidgetLightSensor_setOnIlluminanceChangeHandler(ch,
		  PhidgetLightSensor_OnIlluminanceChange_LaviewHandler, lvEventRef);
	else
		PhidgetLightSensor_setOnIlluminanceChangeHandler(ch, NULL, NULL);

	return (EPHIDGET_OK);
}
#else
API_PRETURN_HDR PhidgetLightSensor_setOnIlluminanceChangeLabviewHandler(PhidgetLightSensorHandle ch,
  LVUserEventRef *lvEventRef) {

	return (EPHIDGET_UNSUPPORTED);
}
#endif

#ifdef COMPILE_PHIDGETS_LABVIEW
void CCONV PhidgetPressureSensor_OnPressureChange_LaviewHandler(PhidgetPressureSensorHandle ch,void *ctx,
  double pressure) {
	LVUserEventRef ev = *(LVUserEventRef *)ctx;
	MgErr ret = 0;
	lvPhidgetPressureSensorPressureChangeArgs *lvdata;
	lvdata = (lvPhidgetPressureSensorPressureChangeArgs *)DSNewPtr(sizeof(lvPhidgetPressureSensorPressureChangeArgs));
	lvdata->ch = (intptr_t)ch;
	lvdata->pressure = pressure;
	ret = PostLVUserEvent(ev, lvdata);
	DSDisposePtr(lvdata);
}

API_PRETURN_HDR PhidgetPressureSensor_setOnPressureChangeLabviewHandler(PhidgetPressureSensorHandle ch,
  LVUserEventRef *lvEventRef) {
	TESTPTR(ch);
	if(lvEventRef && *lvEventRef)
		PhidgetPressureSensor_setOnPressureChangeHandler(ch,
		  PhidgetPressureSensor_OnPressureChange_LaviewHandler, lvEventRef);
	else
		PhidgetPressureSensor_setOnPressureChangeHandler(ch, NULL, NULL);

	return (EPHIDGET_OK);
}
#else
API_PRETURN_HDR PhidgetPressureSensor_setOnPressureChangeLabviewHandler(PhidgetPressureSensorHandle ch,
  LVUserEventRef *lvEventRef) {

	return (EPHIDGET_UNSUPPORTED);
}
#endif

#ifdef COMPILE_PHIDGETS_LABVIEW
void CCONV PhidgetResistanceInput_OnResistanceChange_LaviewHandler(PhidgetResistanceInputHandle ch,
 void *ctx, double resistance) {
	LVUserEventRef ev = *(LVUserEventRef *)ctx;
	MgErr ret = 0;
	lvPhidgetResistanceInputResistanceChangeArgs *lvdata;
	lvdata = (lvPhidgetResistanceInputResistanceChangeArgs *)DSNewPtr(sizeof(lvPhidgetResistanceInputResistanceChangeArgs));
	lvdata->ch = (intptr_t)ch;
	lvdata->resistance = resistance;
	ret = PostLVUserEvent(ev, lvdata);
	DSDisposePtr(lvdata);
}

API_PRETURN_HDR PhidgetResistanceInput_setOnResistanceChangeLabviewHandler(PhidgetResistanceInputHandle ch, LVUserEventRef *lvEventRef) {
	TESTPTR(ch);
	if(lvEventRef && *lvEventRef)
		PhidgetResistanceInput_setOnResistanceChangeHandler(ch,
		  PhidgetResistanceInput_OnResistanceChange_LaviewHandler, lvEventRef);
	else
		PhidgetResistanceInput_setOnResistanceChangeHandler(ch, NULL, NULL);

	return (EPHIDGET_OK);
}
#else
API_PRETURN_HDR PhidgetResistanceInput_setOnResistanceChangeLabviewHandler(PhidgetResistanceInputHandle ch, LVUserEventRef *lvEventRef) {

	return (EPHIDGET_UNSUPPORTED);
}
#endif

#ifdef COMPILE_PHIDGETS_LABVIEW
void CCONV PhidgetSoundSensor_OnSPLChange_LaviewHandler(PhidgetSoundSensorHandle ch,void *ctx, double dB,
  double dBA, double dBC, const double octaves[10]) {
	LVUserEventRef ev = *(LVUserEventRef *)ctx;
	MgErr ret = 0;
	lvPhidgetSoundSensorSPLChangeArgs *lvdata;
	lvdata = (lvPhidgetSoundSensorSPLChangeArgs *)DSNewPtr(sizeof(lvPhidgetSoundSensorSPLChangeArgs));
	lvdata->ch = (intptr_t)ch;
	lvdata->dB = dB;
	lvdata->dBA = dBA;
	lvdata->dBC = dBC;
	lvdata->octaves = (lvArrHandle *)DSNewHandle(sizeof(size_t) + sizeof(double) * 10);
	memcpy(&((*(lvdata->octaves))->data), octaves, sizeof(double) * 10);
	(*(lvdata->octaves))->length = (int32_t)10;
	ret = PostLVUserEvent(ev, lvdata);
	DSDisposePtr(lvdata);
}

API_PRETURN_HDR PhidgetSoundSensor_setOnSPLChangeLabviewHandler(PhidgetSoundSensorHandle ch,
  LVUserEventRef *lvEventRef) {
	TESTPTR(ch);
	if(lvEventRef && *lvEventRef)
		PhidgetSoundSensor_setOnSPLChangeHandler(ch, PhidgetSoundSensor_OnSPLChange_LaviewHandler,
		  lvEventRef);
	else
		PhidgetSoundSensor_setOnSPLChangeHandler(ch, NULL, NULL);

	return (EPHIDGET_OK);
}
#else
API_PRETURN_HDR PhidgetSoundSensor_setOnSPLChangeLabviewHandler(PhidgetSoundSensorHandle ch,
  LVUserEventRef *lvEventRef) {

	return (EPHIDGET_UNSUPPORTED);
}
#endif

#ifdef COMPILE_PHIDGETS_LABVIEW
void CCONV PhidgetFirmwareUpgrade_OnProgressChange_LaviewHandler(PhidgetFirmwareUpgradeHandle ch,
 void *ctx, double progress) {
	LVUserEventRef ev = *(LVUserEventRef *)ctx;
	MgErr ret = 0;
	lvPhidgetFirmwareUpgradeProgressChangeArgs *lvdata;
	lvdata = (lvPhidgetFirmwareUpgradeProgressChangeArgs *)DSNewPtr(sizeof(lvPhidgetFirmwareUpgradeProgressChangeArgs));
	lvdata->ch = (intptr_t)ch;
	lvdata->progress = progress;
	ret = PostLVUserEvent(ev, lvdata);
	DSDisposePtr(lvdata);
}

API_PRETURN_HDR PhidgetFirmwareUpgrade_setOnProgressChangeLabviewHandler(PhidgetFirmwareUpgradeHandle ch,
  LVUserEventRef *lvEventRef) {
	TESTPTR(ch);
	if(lvEventRef && *lvEventRef)
		PhidgetFirmwareUpgrade_setOnProgressChangeHandler(ch,
		  PhidgetFirmwareUpgrade_OnProgressChange_LaviewHandler, lvEventRef);
	else
		PhidgetFirmwareUpgrade_setOnProgressChangeHandler(ch, NULL, NULL);

	return (EPHIDGET_OK);
}
#else
API_PRETURN_HDR PhidgetFirmwareUpgrade_setOnProgressChangeLabviewHandler(PhidgetFirmwareUpgradeHandle ch,
  LVUserEventRef *lvEventRef) {

	return (EPHIDGET_UNSUPPORTED);
}
#endif

#ifdef COMPILE_PHIDGETS_LABVIEW
void CCONV PhidgetDictionary_OnAdd_LaviewHandler(PhidgetDictionaryHandle ch,void *ctx, const char *key,
  const char *value) {
	LVUserEventRef ev = *(LVUserEventRef *)ctx;
	MgErr ret = 0;
	lvPhidgetDictionaryAddArgs *lvdata;
	lvdata = (lvPhidgetDictionaryAddArgs *)DSNewPtr(sizeof(lvPhidgetDictionaryAddArgs));
	lvdata->ch = (intptr_t)ch;
	lvdata->key = (LStrHandle)DSNewHandle(sizeof(int32_t) + sizeof(char) * (mos_strlen(key) + 1));
	mos_strlcpy((char *)LStrBuf(*lvdata->key), key, mos_strlen(key) + 1);
	LStrLen(*lvdata->key) = (int32_t)mos_strlen(key);
	lvdata->value = (LStrHandle)DSNewHandle(sizeof(int32_t) + sizeof(char) * (mos_strlen(value) + 1));
	mos_strlcpy((char *)LStrBuf(*lvdata->value), value, mos_strlen(value) + 1);
	LStrLen(*lvdata->value) = (int32_t)mos_strlen(value);
	ret = PostLVUserEvent(ev, lvdata);
	DSDisposePtr(lvdata);
}

API_PRETURN_HDR PhidgetDictionary_setOnAddLabviewHandler(PhidgetDictionaryHandle ch,
  LVUserEventRef *lvEventRef) {
	TESTPTR(ch);
	if(lvEventRef && *lvEventRef)
		PhidgetDictionary_setOnAddHandler(ch, PhidgetDictionary_OnAdd_LaviewHandler, lvEventRef);
	else
		PhidgetDictionary_setOnAddHandler(ch, NULL, NULL);

	return (EPHIDGET_OK);
}
#else
API_PRETURN_HDR PhidgetDictionary_setOnAddLabviewHandler(PhidgetDictionaryHandle ch,
  LVUserEventRef *lvEventRef) {

	return (EPHIDGET_UNSUPPORTED);
}
#endif

#ifdef COMPILE_PHIDGETS_LABVIEW
void CCONV PhidgetDictionary_OnRemove_LaviewHandler(PhidgetDictionaryHandle ch,void *ctx,
  const char *key) {
	LVUserEventRef ev = *(LVUserEventRef *)ctx;
	MgErr ret = 0;
	lvPhidgetDictionaryRemoveArgs *lvdata;
	lvdata = (lvPhidgetDictionaryRemoveArgs *)DSNewPtr(sizeof(lvPhidgetDictionaryRemoveArgs));
	lvdata->ch = (intptr_t)ch;
	lvdata->key = (LStrHandle)DSNewHandle(sizeof(int32_t) + sizeof(char) * (mos_strlen(key) + 1));
	mos_strlcpy((char *)LStrBuf(*lvdata->key), key, mos_strlen(key) + 1);
	LStrLen(*lvdata->key) = (int32_t)mos_strlen(key);
	ret = PostLVUserEvent(ev, lvdata);
	DSDisposePtr(lvdata);
}

API_PRETURN_HDR PhidgetDictionary_setOnRemoveLabviewHandler(PhidgetDictionaryHandle ch,
  LVUserEventRef *lvEventRef) {
	TESTPTR(ch);
	if(lvEventRef && *lvEventRef)
		PhidgetDictionary_setOnRemoveHandler(ch, PhidgetDictionary_OnRemove_LaviewHandler,
		  lvEventRef);
	else
		PhidgetDictionary_setOnRemoveHandler(ch, NULL, NULL);

	return (EPHIDGET_OK);
}
#else
API_PRETURN_HDR PhidgetDictionary_setOnRemoveLabviewHandler(PhidgetDictionaryHandle ch,
  LVUserEventRef *lvEventRef) {

	return (EPHIDGET_UNSUPPORTED);
}
#endif

#ifdef COMPILE_PHIDGETS_LABVIEW
void CCONV PhidgetDictionary_OnUpdate_LaviewHandler(PhidgetDictionaryHandle ch,void *ctx, const char *key,
  const char *value) {
	LVUserEventRef ev = *(LVUserEventRef *)ctx;
	MgErr ret = 0;
	lvPhidgetDictionaryUpdateArgs *lvdata;
	lvdata = (lvPhidgetDictionaryUpdateArgs *)DSNewPtr(sizeof(lvPhidgetDictionaryUpdateArgs));
	lvdata->ch = (intptr_t)ch;
	lvdata->key = (LStrHandle)DSNewHandle(sizeof(int32_t) + sizeof(char) * (mos_strlen(key) + 1));
	mos_strlcpy((char *)LStrBuf(*lvdata->key), key, mos_strlen(key) + 1);
	LStrLen(*lvdata->key) = (int32_t)mos_strlen(key);
	lvdata->value = (LStrHandle)DSNewHandle(sizeof(int32_t) + sizeof(char) * (mos_strlen(value) + 1));
	mos_strlcpy((char *)LStrBuf(*lvdata->value), value, mos_strlen(value) + 1);
	LStrLen(*lvdata->value) = (int32_t)mos_strlen(value);
	ret = PostLVUserEvent(ev, lvdata);
	DSDisposePtr(lvdata);
}

API_PRETURN_HDR PhidgetDictionary_setOnUpdateLabviewHandler(PhidgetDictionaryHandle ch,
  LVUserEventRef *lvEventRef) {
	TESTPTR(ch);
	if(lvEventRef && *lvEventRef)
		PhidgetDictionary_setOnUpdateHandler(ch, PhidgetDictionary_OnUpdate_LaviewHandler,
		  lvEventRef);
	else
		PhidgetDictionary_setOnUpdateHandler(ch, NULL, NULL);

	return (EPHIDGET_OK);
}
#else
API_PRETURN_HDR PhidgetDictionary_setOnUpdateLabviewHandler(PhidgetDictionaryHandle ch,
  LVUserEventRef *lvEventRef) {

	return (EPHIDGET_UNSUPPORTED);
}
#endif
