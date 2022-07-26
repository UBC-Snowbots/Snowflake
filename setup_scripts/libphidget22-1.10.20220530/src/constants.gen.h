#ifndef _PHIDGETCONSTANTS_GEN_H_
#define _PHIDGETCONSTANTS_GEN_H_

#define PHIDGET_SERIALNUMBER_ANY -1	// Pass to ${PROPERTY:DeviceSerialNumber:set} to open any serial number.
#define PHIDGET_HUBPORT_ANY -1	// Pass to ${PROPERTY:HubPort:set} to open any hub port.
#define PHIDGET_CHANNEL_ANY -1	// Pass to ${PROPERTY:Channel:set} to open any channel.
#define PHIDGET_LABEL_ANY NULL	// Pass to ${PROPERTY:DeviceLabel:set} to open any label.
#define PHIDGET_TIMEOUT_INFINITE 0x0	// Pass to ${METHOD:openWaitForAttachment} for an infinite timeout.
#define PHIDGET_TIMEOUT_DEFAULT 0x3e8	// Pass to ${METHOD:openWaitForAttachment} for the default timeout.
#define PHIDGETSERVER_AUTHREQUIRED 0x1	// PhidgetServer flag indicating that the server requires a password to authenticate
#define IR_RAWDATA_LONGSPACE 0xffffffff	// The value for a long space in raw data
#define IR_MAX_CODE_BIT_COUNT 0x80	// Maximum bit count for sent / received data
#define IR_MAX_CODE_STR_LENGTH 0x21	// Maximum bit count for sent / received data

#endif /* _PHIDGETCONSTANTS_GEN_H_ */
