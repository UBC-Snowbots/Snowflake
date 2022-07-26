#include "phidgetbase.h"

int
supportedBridgePacket(PhidgetChannelHandle ch, bridgepacket_t pkt) {

	switch (ch->UCD->uid) {
	case PHIDCHUID_ifkit488_VOLTAGERATIOINPUT_000:
		switch (pkt) {
		case BP_DATAINTERVALCHANGE:
		case BP_MINDATAINTERVALCHANGE:
		case BP_SETDATAINTERVAL:
		case BP_SETSENSORTYPE:
		case BP_SETCHANGETRIGGER:
		case BP_SETSENSORVALUECHANGETRIGGER:
		case BP_VOLTAGERATIOCHANGE:
		case BP_SENSORCHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_ifkit488_DIGITALINPUT_000:
		switch (pkt) {
		case BP_STATECHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_ifkit488_DIGITALOUTPUT_000:
		switch (pkt) {
		case BP_SETSTATE:
		case BP_SETDUTYCYCLE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1000_RCSERVO_OLD1_200:
		switch (pkt) {
		case BP_SETMINPULSEWIDTH:
		case BP_SETMAXPULSEWIDTH:
		case BP_SETTARGETPOSITION:
		case BP_SETENGAGED:
		case BP_POSITIONCHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1000_RCSERVO_OLD2_200:
		switch (pkt) {
		case BP_SETMINPULSEWIDTH:
		case BP_SETMAXPULSEWIDTH:
		case BP_SETTARGETPOSITION:
		case BP_SETENGAGED:
		case BP_POSITIONCHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1000_RCSERVO_300:
		switch (pkt) {
		case BP_SETMINPULSEWIDTH:
		case BP_SETMAXPULSEWIDTH:
		case BP_SETTARGETPOSITION:
		case BP_SETENGAGED:
		case BP_POSITIONCHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1000_RCSERVO_313:
		switch (pkt) {
		case BP_SETMINPULSEWIDTH:
		case BP_SETMAXPULSEWIDTH:
		case BP_SETTARGETPOSITION:
		case BP_SETENGAGED:
		case BP_POSITIONCHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1001_RCSERVO_OLD1_200:
		switch (pkt) {
		case BP_SETMINPULSEWIDTH:
		case BP_SETMAXPULSEWIDTH:
		case BP_SETTARGETPOSITION:
		case BP_SETENGAGED:
		case BP_POSITIONCHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1001_RCSERVO_OLD2_200:
		switch (pkt) {
		case BP_SETMINPULSEWIDTH:
		case BP_SETMAXPULSEWIDTH:
		case BP_SETTARGETPOSITION:
		case BP_SETENGAGED:
		case BP_POSITIONCHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1001_RCSERVO_313:
		switch (pkt) {
		case BP_SETMINPULSEWIDTH:
		case BP_SETMAXPULSEWIDTH:
		case BP_SETTARGETPOSITION:
		case BP_SETENGAGED:
		case BP_POSITIONCHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1001_RCSERVO_400:
		switch (pkt) {
		case BP_SETMINPULSEWIDTH:
		case BP_SETMAXPULSEWIDTH:
		case BP_SETTARGETPOSITION:
		case BP_SETENGAGED:
		case BP_POSITIONCHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1002_VOLTAGEOUTPUT_100:
		switch (pkt) {
		case BP_SETENABLED:
		case BP_SETVOLTAGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1008_ACCELEROMETER_000:
		switch (pkt) {
		case BP_SETCHANGETRIGGER:
		case BP_SETDATAINTERVAL:
		case BP_ACCELERATIONCHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1011_VOLTAGEINPUT_000:
		switch (pkt) {
		case BP_DATAINTERVALCHANGE:
		case BP_MINDATAINTERVALCHANGE:
		case BP_SETDATAINTERVAL:
		case BP_SETSENSORTYPE:
		case BP_SETCHANGETRIGGER:
		case BP_SETSENSORVALUECHANGETRIGGER:
		case BP_VOLTAGECHANGE:
		case BP_SENSORCHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1011_VOLTAGERATIOINPUT_000:
		switch (pkt) {
		case BP_DATAINTERVALCHANGE:
		case BP_MINDATAINTERVALCHANGE:
		case BP_SETDATAINTERVAL:
		case BP_SETSENSORTYPE:
		case BP_SETCHANGETRIGGER:
		case BP_SETSENSORVALUECHANGETRIGGER:
		case BP_VOLTAGERATIOCHANGE:
		case BP_SENSORCHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1011_DIGITALINPUT_000:
		switch (pkt) {
		case BP_STATECHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1011_DIGITALOUTPUT_000:
		switch (pkt) {
		case BP_SETSTATE:
		case BP_SETDUTYCYCLE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1012_DIGITALINPUT_000:
		switch (pkt) {
		case BP_STATECHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1012_DIGITALOUTPUT_000:
		switch (pkt) {
		case BP_SETSTATE:
		case BP_SETDUTYCYCLE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1012_DIGITALINPUT_601:
		switch (pkt) {
		case BP_STATECHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1012_DIGITALOUTPUT_601:
		switch (pkt) {
		case BP_SETSTATE:
		case BP_SETDUTYCYCLE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1012_DIGITALINPUT_602:
		switch (pkt) {
		case BP_STATECHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1012_DIGITALOUTPUT_602:
		switch (pkt) {
		case BP_SETSTATE:
		case BP_SETDUTYCYCLE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1013_VOLTAGEINPUT_000:
		switch (pkt) {
		case BP_DATAINTERVALCHANGE:
		case BP_MINDATAINTERVALCHANGE:
		case BP_SETDATAINTERVAL:
		case BP_SETSENSORTYPE:
		case BP_SETCHANGETRIGGER:
		case BP_SETSENSORVALUECHANGETRIGGER:
		case BP_VOLTAGECHANGE:
		case BP_SENSORCHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1013_VOLTAGERATIOINPUT_000:
		switch (pkt) {
		case BP_DATAINTERVALCHANGE:
		case BP_MINDATAINTERVALCHANGE:
		case BP_SETDATAINTERVAL:
		case BP_SETSENSORTYPE:
		case BP_SETCHANGETRIGGER:
		case BP_SETSENSORVALUECHANGETRIGGER:
		case BP_VOLTAGERATIOCHANGE:
		case BP_SENSORCHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1013_DIGITALINPUT_000:
		switch (pkt) {
		case BP_STATECHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1013_DIGITALOUTPUT_000:
		switch (pkt) {
		case BP_SETDUTYCYCLE:
		case BP_SETSTATE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1018_VOLTAGEINPUT_821:
		switch (pkt) {
		case BP_DATAINTERVALCHANGE:
		case BP_MINDATAINTERVALCHANGE:
		case BP_SETDATAINTERVAL:
		case BP_SETSENSORTYPE:
		case BP_SETCHANGETRIGGER:
		case BP_SETSENSORVALUECHANGETRIGGER:
		case BP_VOLTAGECHANGE:
		case BP_SENSORCHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1018_VOLTAGERATIOINPUT_821:
		switch (pkt) {
		case BP_DATAINTERVALCHANGE:
		case BP_MINDATAINTERVALCHANGE:
		case BP_SETDATAINTERVAL:
		case BP_SETSENSORTYPE:
		case BP_SETCHANGETRIGGER:
		case BP_SETSENSORVALUECHANGETRIGGER:
		case BP_VOLTAGERATIOCHANGE:
		case BP_SENSORCHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1018_DIGITALINPUT_821:
		switch (pkt) {
		case BP_STATECHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1018_DIGITALOUTPUT_821:
		switch (pkt) {
		case BP_SETSTATE:
		case BP_SETDUTYCYCLE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1014_DIGITALOUTPUT_000:
		switch (pkt) {
		case BP_SETSTATE:
		case BP_SETDUTYCYCLE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1014_DIGITALOUTPUT_704:
		switch (pkt) {
		case BP_SETSTATE:
		case BP_SETDUTYCYCLE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1015_CAPACITIVETOUCH_000:
		switch (pkt) {
		case BP_SETCHANGETRIGGER:
		case BP_SETDATAINTERVAL:
		case BP_TOUCHINPUTVALUECHANGE:
		case BP_TOUCHINPUTEND:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1016_CAPACITIVETOUCH_000:
		switch (pkt) {
		case BP_SETCHANGETRIGGER:
		case BP_SETDATAINTERVAL:
		case BP_TOUCHINPUTVALUECHANGE:
		case BP_TOUCHINPUTEND:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1017_DIGITALOUTPUT_000:
		switch (pkt) {
		case BP_SETSTATE:
		case BP_SETDUTYCYCLE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1018_VOLTAGEINPUT_900:
		switch (pkt) {
		case BP_DATAINTERVALCHANGE:
		case BP_MINDATAINTERVALCHANGE:
		case BP_SETDATAINTERVAL:
		case BP_SETSENSORTYPE:
		case BP_SETCHANGETRIGGER:
		case BP_SETSENSORVALUECHANGETRIGGER:
		case BP_VOLTAGECHANGE:
		case BP_SENSORCHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1018_VOLTAGERATIOINPUT_900:
		switch (pkt) {
		case BP_DATAINTERVALCHANGE:
		case BP_MINDATAINTERVALCHANGE:
		case BP_SETDATAINTERVAL:
		case BP_SETSENSORTYPE:
		case BP_SETCHANGETRIGGER:
		case BP_SETSENSORVALUECHANGETRIGGER:
		case BP_VOLTAGERATIOCHANGE:
		case BP_SENSORCHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1018_DIGITALINPUT_900:
		switch (pkt) {
		case BP_STATECHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1018_DIGITALOUTPUT_900:
		switch (pkt) {
		case BP_SETSTATE:
		case BP_SETDUTYCYCLE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1018_VOLTAGEINPUT_1000:
		switch (pkt) {
		case BP_DATAINTERVALCHANGE:
		case BP_MINDATAINTERVALCHANGE:
		case BP_SETDATAINTERVAL:
		case BP_SETSENSORTYPE:
		case BP_SETCHANGETRIGGER:
		case BP_SETSENSORVALUECHANGETRIGGER:
		case BP_VOLTAGECHANGE:
		case BP_SENSORCHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1018_VOLTAGERATIOINPUT_1000:
		switch (pkt) {
		case BP_DATAINTERVALCHANGE:
		case BP_MINDATAINTERVALCHANGE:
		case BP_SETDATAINTERVAL:
		case BP_SETSENSORTYPE:
		case BP_SETCHANGETRIGGER:
		case BP_SETSENSORVALUECHANGETRIGGER:
		case BP_VOLTAGERATIOCHANGE:
		case BP_SENSORCHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1018_DIGITALINPUT_1000:
		switch (pkt) {
		case BP_STATECHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1018_DIGITALOUTPUT_1000:
		switch (pkt) {
		case BP_SETSTATE:
		case BP_SETDUTYCYCLE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1023_RFID_000:
		switch (pkt) {
		case BP_TAG:
		case BP_TAGLOST:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1023_RFID_104:
		switch (pkt) {
		case BP_TAG:
		case BP_TAGLOST:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1023_RFID_200:
		switch (pkt) {
		case BP_SETANTENNAON:
		case BP_TAG:
		case BP_TAGLOST:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1023_DIGITALOUTPUT_5V_200:
		switch (pkt) {
		case BP_SETSTATE:
		case BP_SETDUTYCYCLE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1023_DIGITALOUTPUT_LED_200:
		switch (pkt) {
		case BP_SETSTATE:
		case BP_SETDUTYCYCLE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1023_DIGITALOUTPUT_ONBOARD_LED_200:
		switch (pkt) {
		case BP_SETSTATE:
		case BP_SETDUTYCYCLE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1023_RFID_201:
		switch (pkt) {
		case BP_SETANTENNAON:
		case BP_TAG:
		case BP_TAGLOST:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1023_DIGITALOUTPUT_5V_201:
		switch (pkt) {
		case BP_SETSTATE:
		case BP_SETDUTYCYCLE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1023_DIGITALOUTPUT_LED_201:
		switch (pkt) {
		case BP_SETSTATE:
		case BP_SETDUTYCYCLE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1023_DIGITALOUTPUT_ONBOARD_LED_201:
		switch (pkt) {
		case BP_SETSTATE:
		case BP_SETDUTYCYCLE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1024_RFID_100:
		switch (pkt) {
		case BP_SETANTENNAON:
		case BP_WRITE:
		case BP_TAG:
		case BP_TAGLOST:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1024_DIGITALOUTPUT_5V_100:
		switch (pkt) {
		case BP_SETSTATE:
		case BP_SETDUTYCYCLE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1024_DIGITALOUTPUT_LED_100:
		switch (pkt) {
		case BP_SETSTATE:
		case BP_SETDUTYCYCLE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1024_DIGITALOUTPUT_ONBOARD_LED_100:
		switch (pkt) {
		case BP_SETSTATE:
		case BP_SETDUTYCYCLE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1030_DIGITALOUTPUT_100:
		switch (pkt) {
		case BP_SETDUTYCYCLE:
		case BP_SETSTATE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1031_DIGITALOUTPUT_100:
		switch (pkt) {
		case BP_SETDUTYCYCLE:
		case BP_SETLEDCURRENTLIMIT:
		case BP_SETLEDFORWARDVOLTAGE:
		case BP_SETSTATE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1032_DIGITALOUTPUT_200:
		switch (pkt) {
		case BP_SETDUTYCYCLE:
		case BP_SETLEDCURRENTLIMIT:
		case BP_SETLEDFORWARDVOLTAGE:
		case BP_SETSTATE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1040_GPS_000:
		switch (pkt) {
		case BP_DATA:
		case BP_DATE:
		case BP_TIME:
		case BP_POSITIONCHANGE:
		case BP_HEADINGCHANGE:
		case BP_POSITIONFIXSTATUSCHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1041_ACCELEROMETER_200:
		switch (pkt) {
		case BP_SETDATAINTERVAL:
		case BP_SETCHANGETRIGGER:
		case BP_ACCELERATIONCHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1042_ACCELEROMETER_300:
		switch (pkt) {
		case BP_DATAINTERVALCHANGE:
		case BP_SETDATAINTERVAL:
		case BP_SETCHANGETRIGGER:
		case BP_ACCELERATIONCHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1042_GYROSCOPE_300:
		switch (pkt) {
		case BP_DATAINTERVALCHANGE:
		case BP_SETDATAINTERVAL:
		case BP_ZERO:
		case BP_ANGULARRATEUPDATE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1042_MAGNETOMETER_300:
		switch (pkt) {
		case BP_DATAINTERVALCHANGE:
		case BP_SETDATAINTERVAL:
		case BP_SETCHANGETRIGGER:
		case BP_SETCORRECTIONPARAMETERS:
		case BP_SAVECORRECTIONPARAMETERS:
		case BP_RESETCORRECTIONPARAMETERS:
		case BP_FIELDSTRENGTHCHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1042_SPATIAL_300:
		switch (pkt) {
		case BP_DATAINTERVALCHANGE:
		case BP_SETDATAINTERVAL:
		case BP_ZERO:
		case BP_SETCORRECTIONPARAMETERS:
		case BP_SAVECORRECTIONPARAMETERS:
		case BP_RESETCORRECTIONPARAMETERS:
		case BP_SPATIALDATA:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1043_ACCELEROMETER_300:
		switch (pkt) {
		case BP_SETDATAINTERVAL:
		case BP_SETCHANGETRIGGER:
		case BP_SETSPATIALPRECISION:
		case BP_ACCELERATIONCHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1044_ACCELEROMETER_400:
		switch (pkt) {
		case BP_DATAINTERVALCHANGE:
		case BP_SETDATAINTERVAL:
		case BP_SETCHANGETRIGGER:
		case BP_SETSPATIALPRECISION:
		case BP_ACCELERATIONCHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1044_GYROSCOPE_400:
		switch (pkt) {
		case BP_DATAINTERVALCHANGE:
		case BP_SETDATAINTERVAL:
		case BP_SETSPATIALPRECISION:
		case BP_ZERO:
		case BP_ANGULARRATEUPDATE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1044_MAGNETOMETER_400:
		switch (pkt) {
		case BP_DATAINTERVALCHANGE:
		case BP_SETDATAINTERVAL:
		case BP_SETCHANGETRIGGER:
		case BP_SETCORRECTIONPARAMETERS:
		case BP_SAVECORRECTIONPARAMETERS:
		case BP_RESETCORRECTIONPARAMETERS:
		case BP_FIELDSTRENGTHCHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1044_SPATIAL_400:
		switch (pkt) {
		case BP_DATAINTERVALCHANGE:
		case BP_SETDATAINTERVAL:
		case BP_SETSPATIALPRECISION:
		case BP_ZERO:
		case BP_SETCORRECTIONPARAMETERS:
		case BP_SAVECORRECTIONPARAMETERS:
		case BP_RESETCORRECTIONPARAMETERS:
		case BP_SPATIALDATA:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1044_ACCELEROMETER_500:
		switch (pkt) {
		case BP_DATAINTERVALCHANGE:
		case BP_SETDATAINTERVAL:
		case BP_SETCHANGETRIGGER:
		case BP_SETSPATIALPRECISION:
		case BP_ACCELERATIONCHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1044_GYROSCOPE_500:
		switch (pkt) {
		case BP_DATAINTERVALCHANGE:
		case BP_SETDATAINTERVAL:
		case BP_SETSPATIALPRECISION:
		case BP_ZERO:
		case BP_ANGULARRATEUPDATE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1044_MAGNETOMETER_500:
		switch (pkt) {
		case BP_DATAINTERVALCHANGE:
		case BP_SETDATAINTERVAL:
		case BP_SETCHANGETRIGGER:
		case BP_SETCORRECTIONPARAMETERS:
		case BP_SAVECORRECTIONPARAMETERS:
		case BP_RESETCORRECTIONPARAMETERS:
		case BP_FIELDSTRENGTHCHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1044_SPATIAL_500:
		switch (pkt) {
		case BP_DATAINTERVALCHANGE:
		case BP_SETDATAINTERVAL:
		case BP_SETSPATIALPRECISION:
		case BP_SETSPATIALALGORITHM:
		case BP_SETSPATIALALGORITHMMAGGAIN:
		case BP_ZERO:
		case BP_SETCORRECTIONPARAMETERS:
		case BP_SAVECORRECTIONPARAMETERS:
		case BP_RESETCORRECTIONPARAMETERS:
		case BP_ZEROSPATIALALGORITHM:
		case BP_SPATIALDATA:
		case BP_SPATIALALGDATA:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1044_ACCELEROMETER_510:
		switch (pkt) {
		case BP_DATAINTERVALCHANGE:
		case BP_SETDATAINTERVAL:
		case BP_SETCHANGETRIGGER:
		case BP_SETSPATIALPRECISION:
		case BP_ACCELERATIONCHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1044_GYROSCOPE_510:
		switch (pkt) {
		case BP_DATAINTERVALCHANGE:
		case BP_SETDATAINTERVAL:
		case BP_SETSPATIALPRECISION:
		case BP_ZERO:
		case BP_ANGULARRATEUPDATE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1044_MAGNETOMETER_510:
		switch (pkt) {
		case BP_DATAINTERVALCHANGE:
		case BP_SETDATAINTERVAL:
		case BP_SETCHANGETRIGGER:
		case BP_SETCORRECTIONPARAMETERS:
		case BP_SAVECORRECTIONPARAMETERS:
		case BP_RESETCORRECTIONPARAMETERS:
		case BP_FIELDSTRENGTHCHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1044_SPATIAL_510:
		switch (pkt) {
		case BP_DATAINTERVALCHANGE:
		case BP_SETDATAINTERVAL:
		case BP_SETSPATIALPRECISION:
		case BP_SETSPATIALALGORITHM:
		case BP_SETSPATIALALGORITHMMAGGAIN:
		case BP_ZERO:
		case BP_SETCORRECTIONPARAMETERS:
		case BP_SAVECORRECTIONPARAMETERS:
		case BP_RESETCORRECTIONPARAMETERS:
		case BP_SETAHRSPARAMETERS:
		case BP_ZEROSPATIALALGORITHM:
		case BP_SPATIALDATA:
		case BP_SPATIALALGDATA:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1045_TEMPERATURESENSOR_IR_100:
		switch (pkt) {
		case BP_SETCHANGETRIGGER:
		case BP_SETDATAINTERVAL:
		case BP_TEMPERATURECHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1045_TEMPERATURESENSOR_IC_100:
		switch (pkt) {
		case BP_SETCHANGETRIGGER:
		case BP_SETDATAINTERVAL:
		case BP_TEMPERATURECHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1046_VOLTAGERATIOINPUT_100:
		switch (pkt) {
		case BP_DATAINTERVALCHANGE:
		case BP_SETENABLED:
		case BP_SETBRIDGEGAIN:
		case BP_SETDATAINTERVAL:
		case BP_SETCHANGETRIGGER:
		case BP_VOLTAGERATIOCHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1046_VOLTAGERATIOINPUT_102:
		switch (pkt) {
		case BP_DATAINTERVALCHANGE:
		case BP_SETENABLED:
		case BP_SETBRIDGEGAIN:
		case BP_SETDATAINTERVAL:
		case BP_SETCHANGETRIGGER:
		case BP_VOLTAGERATIOCHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1047_ENCODER_100:
		switch (pkt) {
		case BP_SETCHANGETRIGGER:
		case BP_SETENABLED:
		case BP_SETDATAINTERVAL:
		case BP_POSITIONCHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1047_DIGITALINPUT_100:
		switch (pkt) {
		case BP_STATECHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1047_ENCODER_200:
		switch (pkt) {
		case BP_SETCHANGETRIGGER:
		case BP_SETENABLED:
		case BP_SETDATAINTERVAL:
		case BP_POSITIONCHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1047_DIGITALINPUT_200:
		switch (pkt) {
		case BP_STATECHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1048_TEMPERATURESENSOR_THERMOCOUPLE_100:
		switch (pkt) {
		case BP_SETCHANGETRIGGER:
		case BP_SETTHERMOCOUPLETYPE:
		case BP_SETDATAINTERVAL:
		case BP_TEMPERATURECHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1048_TEMPERATURESENSOR_IC_100:
		switch (pkt) {
		case BP_SETCHANGETRIGGER:
		case BP_SETDATAINTERVAL:
		case BP_TEMPERATURECHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1048_VOLTAGEINPUT_100:
		switch (pkt) {
		case BP_SETCHANGETRIGGER:
		case BP_SETDATAINTERVAL:
		case BP_VOLTAGECHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1048_TEMPERATURESENSOR_THERMOCOUPLE_200:
		switch (pkt) {
		case BP_SETCHANGETRIGGER:
		case BP_SETTHERMOCOUPLETYPE:
		case BP_SETDATAINTERVAL:
		case BP_TEMPERATURECHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1048_TEMPERATURESENSOR_IC_200:
		switch (pkt) {
		case BP_SETCHANGETRIGGER:
		case BP_SETDATAINTERVAL:
		case BP_TEMPERATURECHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1048_VOLTAGEINPUT_200:
		switch (pkt) {
		case BP_SETCHANGETRIGGER:
		case BP_SETDATAINTERVAL:
		case BP_VOLTAGECHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1049_ACCELEROMETER_000:
		switch (pkt) {
		case BP_SETDATAINTERVAL:
		case BP_SETCHANGETRIGGER:
		case BP_ACCELERATIONCHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1051_TEMPERATURESENSOR_THERMOCOUPLE_000:
		switch (pkt) {
		case BP_SETCHANGETRIGGER:
		case BP_SETDATAINTERVAL:
		case BP_TEMPERATURECHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1051_TEMPERATURESENSOR_IC_000:
		switch (pkt) {
		case BP_SETCHANGETRIGGER:
		case BP_SETDATAINTERVAL:
		case BP_TEMPERATURECHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1051_TEMPERATURESENSOR_THERMOCOUPLE_200:
		switch (pkt) {
		case BP_SETCHANGETRIGGER:
		case BP_SETTHERMOCOUPLETYPE:
		case BP_SETDATAINTERVAL:
		case BP_TEMPERATURECHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1051_TEMPERATURESENSOR_IC_200:
		switch (pkt) {
		case BP_SETCHANGETRIGGER:
		case BP_SETDATAINTERVAL:
		case BP_TEMPERATURECHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1051_VOLTAGEINPUT_200:
		switch (pkt) {
		case BP_SETCHANGETRIGGER:
		case BP_SETDATAINTERVAL:
		case BP_VOLTAGECHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1051_TEMPERATURESENSOR_THERMOCOUPLE_300:
		switch (pkt) {
		case BP_SETCHANGETRIGGER:
		case BP_SETTHERMOCOUPLETYPE:
		case BP_SETDATAINTERVAL:
		case BP_TEMPERATURECHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1051_TEMPERATURESENSOR_IC_300:
		switch (pkt) {
		case BP_SETCHANGETRIGGER:
		case BP_SETDATAINTERVAL:
		case BP_TEMPERATURECHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1051_VOLTAGEINPUT_300:
		switch (pkt) {
		case BP_SETCHANGETRIGGER:
		case BP_SETDATAINTERVAL:
		case BP_VOLTAGECHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1051_TEMPERATURESENSOR_THERMOCOUPLE_400:
		switch (pkt) {
		case BP_SETCHANGETRIGGER:
		case BP_SETTHERMOCOUPLETYPE:
		case BP_SETDATAINTERVAL:
		case BP_TEMPERATURECHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1051_TEMPERATURESENSOR_IC_400:
		switch (pkt) {
		case BP_SETCHANGETRIGGER:
		case BP_SETDATAINTERVAL:
		case BP_TEMPERATURECHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1051_VOLTAGEINPUT_400:
		switch (pkt) {
		case BP_SETCHANGETRIGGER:
		case BP_SETDATAINTERVAL:
		case BP_VOLTAGECHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1052_ENCODER_000:
		switch (pkt) {
		case BP_SETCHANGETRIGGER:
		case BP_SETDATAINTERVAL:
		case BP_POSITIONCHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1052_DIGITALINPUT_000:
		switch (pkt) {
		case BP_STATECHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1052_ENCODER_101:
		switch (pkt) {
		case BP_SETCHANGETRIGGER:
		case BP_SETDATAINTERVAL:
		case BP_POSITIONCHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1052_DIGITALINPUT_101:
		switch (pkt) {
		case BP_STATECHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1052_ENCODER_110:
		switch (pkt) {
		case BP_SETCHANGETRIGGER:
		case BP_SETDATAINTERVAL:
		case BP_POSITIONCHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1052_DIGITALINPUT_110:
		switch (pkt) {
		case BP_STATECHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1053_ACCELEROMETER_300:
		switch (pkt) {
		case BP_SETCHANGETRIGGER:
		case BP_SETDATAINTERVAL:
		case BP_ACCELERATIONCHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1054_FREQUENCYCOUNTER_000:
		switch (pkt) {
		case BP_FREQUENCYDATA:
		case BP_SETENABLED:
		case BP_SETFILTERTYPE:
		case BP_SETDATAINTERVAL:
		case BP_FREQUENCYCHANGE:
		case BP_COUNTCHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1055_IR_100:
		switch (pkt) {
		case BP_REPEAT:
		case BP_TRANSMIT:
		case BP_TRANSMITREPEAT:
		case BP_TRANSMITRAW:
		case BP_CODE:
		case BP_LEARN:
		case BP_RAWDATA:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1055_IR_200_USB:
		switch (pkt) {
		case BP_REPEAT:
		case BP_TRANSMIT:
		case BP_TRANSMITREPEAT:
		case BP_TRANSMITRAW:
		case BP_CODE:
		case BP_LEARN:
		case BP_RAWDATA:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1055_IR_200_VINT:
		switch (pkt) {
		case BP_REPEAT:
		case BP_TRANSMIT:
		case BP_TRANSMITREPEAT:
		case BP_TRANSMITRAW:
		case BP_CODE:
		case BP_LEARN:
		case BP_RAWDATA:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1056_ACCELEROMETER_000:
		switch (pkt) {
		case BP_DATAINTERVALCHANGE:
		case BP_SETDATAINTERVAL:
		case BP_SETCHANGETRIGGER:
		case BP_ACCELERATIONCHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1056_GYROSCOPE_000:
		switch (pkt) {
		case BP_DATAINTERVALCHANGE:
		case BP_SETDATAINTERVAL:
		case BP_ZERO:
		case BP_ANGULARRATEUPDATE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1056_MAGNETOMETER_000:
		switch (pkt) {
		case BP_DATAINTERVALCHANGE:
		case BP_SETDATAINTERVAL:
		case BP_SETCHANGETRIGGER:
		case BP_SETCORRECTIONPARAMETERS:
		case BP_SAVECORRECTIONPARAMETERS:
		case BP_RESETCORRECTIONPARAMETERS:
		case BP_FIELDSTRENGTHCHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1056_SPATIAL_000:
		switch (pkt) {
		case BP_DATAINTERVALCHANGE:
		case BP_SETDATAINTERVAL:
		case BP_ZERO:
		case BP_SETCORRECTIONPARAMETERS:
		case BP_SAVECORRECTIONPARAMETERS:
		case BP_RESETCORRECTIONPARAMETERS:
		case BP_SPATIALDATA:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1056_ACCELEROMETER_200:
		switch (pkt) {
		case BP_DATAINTERVALCHANGE:
		case BP_SETDATAINTERVAL:
		case BP_SETCHANGETRIGGER:
		case BP_ACCELERATIONCHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1056_GYROSCOPE_200:
		switch (pkt) {
		case BP_DATAINTERVALCHANGE:
		case BP_SETDATAINTERVAL:
		case BP_ZERO:
		case BP_ANGULARRATEUPDATE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1056_MAGNETOMETER_200:
		switch (pkt) {
		case BP_DATAINTERVALCHANGE:
		case BP_SETDATAINTERVAL:
		case BP_SETCHANGETRIGGER:
		case BP_SETCORRECTIONPARAMETERS:
		case BP_SAVECORRECTIONPARAMETERS:
		case BP_RESETCORRECTIONPARAMETERS:
		case BP_FIELDSTRENGTHCHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1056_SPATIAL_200:
		switch (pkt) {
		case BP_DATAINTERVALCHANGE:
		case BP_SETDATAINTERVAL:
		case BP_ZERO:
		case BP_SETCORRECTIONPARAMETERS:
		case BP_SAVECORRECTIONPARAMETERS:
		case BP_RESETCORRECTIONPARAMETERS:
		case BP_SPATIALDATA:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1057_ENCODER_300:
		switch (pkt) {
		case BP_SETCHANGETRIGGER:
		case BP_SETDATAINTERVAL:
		case BP_POSITIONCHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1057_ENCODER_400:
		switch (pkt) {
		case BP_SETCHANGETRIGGER:
		case BP_SETENABLED:
		case BP_SETDATAINTERVAL:
		case BP_SETIOMODE:
		case BP_POSITIONCHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1058_VOLTAGEINPUT_100:
		switch (pkt) {
		case BP_SETCHANGETRIGGER:
		case BP_SETDATAINTERVAL:
		case BP_VOLTAGECHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1058_PHADAPTER_100:
		switch (pkt) {
		case BP_SETDATAINTERVAL:
		case BP_SETCHANGETRIGGER:
		case BP_SETCORRECTIONTEMPERATURE:
		case BP_PHCHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1059_ACCELEROMETER_400:
		switch (pkt) {
		case BP_SETCHANGETRIGGER:
		case BP_SETDATAINTERVAL:
		case BP_ACCELERATIONCHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1060_DCMOTOR_100:
		switch (pkt) {
		case BP_SETDATAINTERVAL:
		case BP_SETDUTYCYCLE:
		case BP_SETACCELERATION:
		case BP_DUTYCYCLECHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1060_DIGITALINPUT_100:
		switch (pkt) {
		case BP_STATECHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1061_RCSERVO_100:
		switch (pkt) {
		case BP_SETMINPULSEWIDTH:
		case BP_SETMAXPULSEWIDTH:
		case BP_SETVELOCITYLIMIT:
		case BP_SETACCELERATION:
		case BP_SETTARGETPOSITION:
		case BP_SETENGAGED:
		case BP_SETSPEEDRAMPINGSTATE:
		case BP_SETDATAINTERVAL:
		case BP_POSITIONCHANGE:
		case BP_VELOCITYCHANGE:
		case BP_TARGETPOSITIONREACHED:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1061_CURRENTINPUT_100:
		switch (pkt) {
		case BP_SETCHANGETRIGGER:
		case BP_SETDATAINTERVAL:
		case BP_CURRENTCHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1061_RCSERVO_200:
		switch (pkt) {
		case BP_SETMINPULSEWIDTH:
		case BP_SETMAXPULSEWIDTH:
		case BP_SETVELOCITYLIMIT:
		case BP_SETACCELERATION:
		case BP_SETTARGETPOSITION:
		case BP_SETENGAGED:
		case BP_SETSPEEDRAMPINGSTATE:
		case BP_SETDATAINTERVAL:
		case BP_POSITIONCHANGE:
		case BP_VELOCITYCHANGE:
		case BP_TARGETPOSITIONREACHED:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1061_CURRENTINPUT_200:
		switch (pkt) {
		case BP_SETCHANGETRIGGER:
		case BP_SETDATAINTERVAL:
		case BP_CURRENTCHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1061_RCSERVO_300:
		switch (pkt) {
		case BP_SETMINPULSEWIDTH:
		case BP_SETMAXPULSEWIDTH:
		case BP_SETVELOCITYLIMIT:
		case BP_SETACCELERATION:
		case BP_SETTARGETPOSITION:
		case BP_SETENGAGED:
		case BP_SETSPEEDRAMPINGSTATE:
		case BP_SETDATAINTERVAL:
		case BP_POSITIONCHANGE:
		case BP_VELOCITYCHANGE:
		case BP_TARGETPOSITIONREACHED:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1061_CURRENTINPUT_300:
		switch (pkt) {
		case BP_SETCHANGETRIGGER:
		case BP_SETDATAINTERVAL:
		case BP_CURRENTCHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_RCC0004_RCSERVO_400:
		switch (pkt) {
		case BP_SETMINPULSEWIDTH:
		case BP_SETMAXPULSEWIDTH:
		case BP_SETVELOCITYLIMIT:
		case BP_SETACCELERATION:
		case BP_SETTARGETPOSITION:
		case BP_SETENGAGED:
		case BP_SETSPEEDRAMPINGSTATE:
		case BP_SETVOLTAGE:
		case BP_SETDATAINTERVAL:
		case BP_POSITIONCHANGE:
		case BP_VELOCITYCHANGE:
		case BP_TARGETPOSITIONREACHED:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1062_STEPPER_100:
		switch (pkt) {
		case BP_SETTARGETPOSITION:
		case BP_SETVELOCITYLIMIT:
		case BP_SETACCELERATION:
		case BP_SETENGAGED:
		case BP_SETCONTROLMODE:
		case BP_SETDATAINTERVAL:
		case BP_POSITIONCHANGE:
		case BP_VELOCITYCHANGE:
		case BP_STOPPED:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1063_STEPPER_100:
		switch (pkt) {
		case BP_SETTARGETPOSITION:
		case BP_SETVELOCITYLIMIT:
		case BP_SETACCELERATION:
		case BP_SETCURRENTLIMIT:
		case BP_SETENGAGED:
		case BP_SETCONTROLMODE:
		case BP_SETDATAINTERVAL:
		case BP_POSITIONCHANGE:
		case BP_VELOCITYCHANGE:
		case BP_STOPPED:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1063_DIGITALINPUT_100:
		switch (pkt) {
		case BP_STATECHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1063_CURRENTINPUT_100:
		switch (pkt) {
		case BP_SETCHANGETRIGGER:
		case BP_SETDATAINTERVAL:
		case BP_CURRENTCHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1064_DCMOTOR_100:
		switch (pkt) {
		case BP_SETDATAINTERVAL:
		case BP_SETDUTYCYCLE:
		case BP_SETACCELERATION:
		case BP_DUTYCYCLECHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1064_CURRENTINPUT_100:
		switch (pkt) {
		case BP_SETCHANGETRIGGER:
		case BP_SETDATAINTERVAL:
		case BP_CURRENTCHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1065_DCMOTOR_100:
		switch (pkt) {
		case BP_SETDATAINTERVAL:
		case BP_SETDUTYCYCLE:
		case BP_SETACCELERATION:
		case BP_SETBRAKINGDUTYCYCLE:
		case BP_SETBACKEMFSENSINGSTATE:
		case BP_DUTYCYCLECHANGE:
		case BP_BACKEMFCHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1065_DIGITALINPUT_100:
		switch (pkt) {
		case BP_STATECHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1065_ENCODER_100:
		switch (pkt) {
		case BP_SETCHANGETRIGGER:
		case BP_SETDATAINTERVAL:
		case BP_POSITIONCHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1065_VOLTAGEINPUT_100:
		switch (pkt) {
		case BP_SETSENSORTYPE:
		case BP_SETCHANGETRIGGER:
		case BP_SETDATAINTERVAL:
		case BP_SETSENSORVALUECHANGETRIGGER:
		case BP_VOLTAGECHANGE:
		case BP_SENSORCHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1065_VOLTAGEINPUT_SUPPLY_100:
		switch (pkt) {
		case BP_SETCHANGETRIGGER:
		case BP_SETDATAINTERVAL:
		case BP_VOLTAGECHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1065_VOLTAGERATIOINPUT_100:
		switch (pkt) {
		case BP_SETSENSORTYPE:
		case BP_SETCHANGETRIGGER:
		case BP_SETDATAINTERVAL:
		case BP_SETSENSORVALUECHANGETRIGGER:
		case BP_VOLTAGERATIOCHANGE:
		case BP_SENSORCHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1065_CURRENTINPUT_100:
		switch (pkt) {
		case BP_SETCHANGETRIGGER:
		case BP_SETDATAINTERVAL:
		case BP_CURRENTCHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1066_RCSERVO_100:
		switch (pkt) {
		case BP_SETMINPULSEWIDTH:
		case BP_SETMAXPULSEWIDTH:
		case BP_SETVELOCITYLIMIT:
		case BP_SETACCELERATION:
		case BP_SETTARGETPOSITION:
		case BP_SETENGAGED:
		case BP_SETSPEEDRAMPINGSTATE:
		case BP_SETDATAINTERVAL:
		case BP_POSITIONCHANGE:
		case BP_VELOCITYCHANGE:
		case BP_TARGETPOSITIONREACHED:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1066_CURRENTINPUT_100:
		switch (pkt) {
		case BP_SETCHANGETRIGGER:
		case BP_SETDATAINTERVAL:
		case BP_CURRENTCHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1067_STEPPER_200:
		switch (pkt) {
		case BP_SETTARGETPOSITION:
		case BP_SETVELOCITYLIMIT:
		case BP_SETACCELERATION:
		case BP_SETCURRENTLIMIT:
		case BP_SETENGAGED:
		case BP_SETCONTROLMODE:
		case BP_SETDATAINTERVAL:
		case BP_POSITIONCHANGE:
		case BP_VELOCITYCHANGE:
		case BP_STOPPED:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1067_STEPPER_300:
		switch (pkt) {
		case BP_SETTARGETPOSITION:
		case BP_SETVELOCITYLIMIT:
		case BP_SETACCELERATION:
		case BP_SETCURRENTLIMIT:
		case BP_SETENGAGED:
		case BP_SETCONTROLMODE:
		case BP_SETDATAINTERVAL:
		case BP_POSITIONCHANGE:
		case BP_VELOCITYCHANGE:
		case BP_STOPPED:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1202_VOLTAGEINPUT_000:
		switch (pkt) {
		case BP_DATAINTERVALCHANGE:
		case BP_MINDATAINTERVALCHANGE:
		case BP_SETDATAINTERVAL:
		case BP_SETSENSORTYPE:
		case BP_SETCHANGETRIGGER:
		case BP_SETSENSORVALUECHANGETRIGGER:
		case BP_VOLTAGECHANGE:
		case BP_SENSORCHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1202_VOLTAGERATIOINPUT_000:
		switch (pkt) {
		case BP_DATAINTERVALCHANGE:
		case BP_MINDATAINTERVALCHANGE:
		case BP_SETDATAINTERVAL:
		case BP_SETSENSORTYPE:
		case BP_SETCHANGETRIGGER:
		case BP_SETSENSORVALUECHANGETRIGGER:
		case BP_VOLTAGERATIOCHANGE:
		case BP_SENSORCHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1202_DIGITALINPUT_000:
		switch (pkt) {
		case BP_STATECHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1202_DIGITALOUTPUT_000:
		switch (pkt) {
		case BP_SETSTATE:
		case BP_SETDUTYCYCLE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1202_VOLTAGEINPUT_120:
		switch (pkt) {
		case BP_DATAINTERVALCHANGE:
		case BP_MINDATAINTERVALCHANGE:
		case BP_SETDATAINTERVAL:
		case BP_SETSENSORTYPE:
		case BP_SETCHANGETRIGGER:
		case BP_SETSENSORVALUECHANGETRIGGER:
		case BP_VOLTAGECHANGE:
		case BP_SENSORCHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1202_VOLTAGERATIOINPUT_120:
		switch (pkt) {
		case BP_DATAINTERVALCHANGE:
		case BP_MINDATAINTERVALCHANGE:
		case BP_SETDATAINTERVAL:
		case BP_SETSENSORTYPE:
		case BP_SETCHANGETRIGGER:
		case BP_SETSENSORVALUECHANGETRIGGER:
		case BP_VOLTAGERATIOCHANGE:
		case BP_SENSORCHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1202_DIGITALINPUT_120:
		switch (pkt) {
		case BP_STATECHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1202_DIGITALOUTPUT_120:
		switch (pkt) {
		case BP_SETSTATE:
		case BP_SETDUTYCYCLE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1202_TEXTLCD_000:
		switch (pkt) {
		case BP_SETBACKLIGHT:
		case BP_SETCONTRAST:
		case BP_SETCURSORBLINK:
		case BP_SETCURSORON:
		case BP_SETCHARACTERBITMAP:
		case BP_CLEAR:
		case BP_FLUSH:
		case BP_WRITETEXT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1202_VOLTAGEINPUT_300:
		switch (pkt) {
		case BP_DATAINTERVALCHANGE:
		case BP_MINDATAINTERVALCHANGE:
		case BP_SETDATAINTERVAL:
		case BP_SETSENSORTYPE:
		case BP_SETCHANGETRIGGER:
		case BP_SETSENSORVALUECHANGETRIGGER:
		case BP_VOLTAGECHANGE:
		case BP_SENSORCHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1202_VOLTAGERATIOINPUT_300:
		switch (pkt) {
		case BP_DATAINTERVALCHANGE:
		case BP_MINDATAINTERVALCHANGE:
		case BP_SETDATAINTERVAL:
		case BP_SETSENSORTYPE:
		case BP_SETCHANGETRIGGER:
		case BP_SETSENSORVALUECHANGETRIGGER:
		case BP_VOLTAGERATIOCHANGE:
		case BP_SENSORCHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1202_DIGITALINPUT_300:
		switch (pkt) {
		case BP_STATECHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1202_DIGITALOUTPUT_300:
		switch (pkt) {
		case BP_SETSTATE:
		case BP_SETDUTYCYCLE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1202_TEXTLCD_200:
		switch (pkt) {
		case BP_SETBACKLIGHT:
		case BP_SETCONTRAST:
		case BP_SETCURSORBLINK:
		case BP_SETCURSORON:
		case BP_SETCHARACTERBITMAP:
		case BP_CLEAR:
		case BP_FLUSH:
		case BP_WRITETEXT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1204_TEXTLCD_000:
		switch (pkt) {
		case BP_SETBACKLIGHT:
		case BP_SETCONTRAST:
		case BP_SETCURSORBLINK:
		case BP_SETCURSORON:
		case BP_SETSCREENSIZE:
		case BP_SETCHARACTERBITMAP:
		case BP_CLEAR:
		case BP_FLUSH:
		case BP_INITIALIZE:
		case BP_WRITETEXT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1215_TEXTLCD_000:
		switch (pkt) {
		case BP_SETBACKLIGHT:
		case BP_SETCONTRAST:
		case BP_SETCURSORBLINK:
		case BP_SETCURSORON:
		case BP_SETCHARACTERBITMAP:
		case BP_CLEAR:
		case BP_FLUSH:
		case BP_WRITETEXT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1219_TEXTLCD_000:
		switch (pkt) {
		case BP_SETBACKLIGHT:
		case BP_SETCONTRAST:
		case BP_SETCURSORBLINK:
		case BP_SETCURSORON:
		case BP_SETCHARACTERBITMAP:
		case BP_CLEAR:
		case BP_FLUSH:
		case BP_WRITETEXT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1219_DIGITALINPUT_000:
		switch (pkt) {
		case BP_STATECHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1219_DIGITALOUTPUT_000:
		switch (pkt) {
		case BP_SETSTATE:
		case BP_SETDUTYCYCLE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_HUB_DIGITALINPUT_100:
		switch (pkt) {
		case BP_STATECHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_HUB_DIGITALOUTPUT_100:
		switch (pkt) {
		case BP_SETDUTYCYCLE:
		case BP_SETSTATE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_HUB_DIGITALOUTPUT_110:
		switch (pkt) {
		case BP_SETDUTYCYCLE:
		case BP_SETSTATE:
		case BP_FAILSAFERESET:
		case BP_SETFAILSAFETIME:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_HUB_VOLTAGEINPUT_100:
		switch (pkt) {
		case BP_SETDATAINTERVAL:
		case BP_SETSENSORTYPE:
		case BP_SETCHANGETRIGGER:
		case BP_SETSENSORVALUECHANGETRIGGER:
		case BP_VOLTAGECHANGE:
		case BP_SENSORCHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_HUB_VOLTAGEINPUT_110:
		switch (pkt) {
		case BP_SETDATAINTERVAL:
		case BP_SETSENSORTYPE:
		case BP_SETCHANGETRIGGER:
		case BP_SETSENSORVALUECHANGETRIGGER:
		case BP_VOLTAGECHANGE:
		case BP_SENSORCHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_HUB_VOLTAGERATIOINPUT_100:
		switch (pkt) {
		case BP_SETDATAINTERVAL:
		case BP_SETSENSORTYPE:
		case BP_SETCHANGETRIGGER:
		case BP_SETSENSORVALUECHANGETRIGGER:
		case BP_VOLTAGERATIOCHANGE:
		case BP_SENSORCHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_ADP1000_PHSENSOR_100:
		switch (pkt) {
		case BP_SETDATAINTERVAL:
		case BP_SETCHANGETRIGGER:
		case BP_SETCORRECTIONTEMPERATURE:
		case BP_PHCHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_ADP1000_VOLTAGEINPUT_100:
		switch (pkt) {
		case BP_SETDATAINTERVAL:
		case BP_SETSENSORTYPE:
		case BP_SETCHANGETRIGGER:
		case BP_SETVOLTAGERANGE:
		case BP_SETSENSORVALUECHANGETRIGGER:
		case BP_VOLTAGECHANGE:
		case BP_SENSORCHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DAQ1000_VOLTAGERATIOINPUT_100:
		switch (pkt) {
		case BP_SETDATAINTERVAL:
		case BP_SETSENSORTYPE:
		case BP_SETCHANGETRIGGER:
		case BP_SETSENSORVALUECHANGETRIGGER:
		case BP_VOLTAGERATIOCHANGE:
		case BP_SENSORCHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DAQ1000_VOLTAGEINPUT_100:
		switch (pkt) {
		case BP_SETDATAINTERVAL:
		case BP_SETSENSORTYPE:
		case BP_SETCHANGETRIGGER:
		case BP_SETSENSORVALUECHANGETRIGGER:
		case BP_VOLTAGECHANGE:
		case BP_SENSORCHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DAQ1000_VOLTAGERATIOINPUT_110:
		switch (pkt) {
		case BP_SETDATAINTERVAL:
		case BP_SETSENSORTYPE:
		case BP_SETCHANGETRIGGER:
		case BP_SETSENSORVALUECHANGETRIGGER:
		case BP_VOLTAGERATIOCHANGE:
		case BP_SENSORCHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DAQ1000_VOLTAGEINPUT_110:
		switch (pkt) {
		case BP_SETDATAINTERVAL:
		case BP_SETSENSORTYPE:
		case BP_SETCHANGETRIGGER:
		case BP_SETSENSORVALUECHANGETRIGGER:
		case BP_VOLTAGECHANGE:
		case BP_SENSORCHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_OUT1000_VOLTAGEOUTPUT_100:
		switch (pkt) {
		case BP_SETENABLED:
		case BP_SETVOLTAGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_OUT1000_VOLTAGEOUTPUT_110:
		switch (pkt) {
		case BP_SETENABLED:
		case BP_SETVOLTAGE:
		case BP_FAILSAFERESET:
		case BP_SETFAILSAFETIME:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_OUT1001_VOLTAGEOUTPUT_100:
		switch (pkt) {
		case BP_SETVOLTAGE:
		case BP_SETVOLTAGERANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_OUT1001_VOLTAGEOUTPUT_110:
		switch (pkt) {
		case BP_SETVOLTAGE:
		case BP_SETVOLTAGERANGE:
		case BP_FAILSAFERESET:
		case BP_SETFAILSAFETIME:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_OUT1002_VOLTAGEOUTPUT_100:
		switch (pkt) {
		case BP_SETVOLTAGE:
		case BP_SETVOLTAGERANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_OUT1002_VOLTAGEOUTPUT_110:
		switch (pkt) {
		case BP_SETVOLTAGE:
		case BP_SETVOLTAGERANGE:
		case BP_FAILSAFERESET:
		case BP_SETFAILSAFETIME:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DAQ1200_DIGITALINPUT_100:
		switch (pkt) {
		case BP_STATECHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_OUT1100_DIGITALOUTPUT_100:
		switch (pkt) {
		case BP_SETDUTYCYCLE:
		case BP_SETSTATE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_OUT1100_DIGITALOUTPUT_110:
		switch (pkt) {
		case BP_SETDUTYCYCLE:
		case BP_SETSTATE:
		case BP_FAILSAFERESET:
		case BP_SETFAILSAFETIME:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_OUT1100_DIGITALOUTPUT_120:
		switch (pkt) {
		case BP_SETDUTYCYCLE:
		case BP_SETSTATE:
		case BP_SETFREQUENCY:
		case BP_FAILSAFERESET:
		case BP_SETFAILSAFETIME:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DAQ1300_DIGITALINPUT_100:
		switch (pkt) {
		case BP_STATECHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DAQ1301_DIGITALINPUT_100:
		switch (pkt) {
		case BP_STATECHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DAQ1400_VOLTAGEINPUT_100:
		switch (pkt) {
		case BP_SETDATAINTERVAL:
		case BP_SETPOWERSUPPLY:
		case BP_SETSENSORTYPE:
		case BP_SETCHANGETRIGGER:
		case BP_SETSENSORVALUECHANGETRIGGER:
		case BP_VOLTAGECHANGE:
		case BP_SENSORCHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DAQ1400_CURRENTINPUT_100:
		switch (pkt) {
		case BP_SETDATAINTERVAL:
		case BP_SETCHANGETRIGGER:
		case BP_SETPOWERSUPPLY:
		case BP_CURRENTCHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DAQ1400_DIGITALINPUT_100:
		switch (pkt) {
		case BP_SETINPUTMODE:
		case BP_SETPOWERSUPPLY:
		case BP_STATECHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DAQ1400_FREQUENCYCOUNTER_100:
		switch (pkt) {
		case BP_FREQUENCYDATA:
		case BP_SETDATAINTERVAL:
		case BP_SETINPUTMODE:
		case BP_SETPOWERSUPPLY:
		case BP_FREQUENCYCHANGE:
		case BP_COUNTCHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DAQ1400_VOLTAGEINPUT_120:
		switch (pkt) {
		case BP_SETDATAINTERVAL:
		case BP_SETPOWERSUPPLY:
		case BP_SETSENSORTYPE:
		case BP_SETCHANGETRIGGER:
		case BP_SETSENSORVALUECHANGETRIGGER:
		case BP_VOLTAGECHANGE:
		case BP_SENSORCHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DAQ1400_CURRENTINPUT_120:
		switch (pkt) {
		case BP_SETDATAINTERVAL:
		case BP_SETCHANGETRIGGER:
		case BP_SETPOWERSUPPLY:
		case BP_CURRENTCHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DAQ1400_DIGITALINPUT_120:
		switch (pkt) {
		case BP_SETINPUTMODE:
		case BP_SETPOWERSUPPLY:
		case BP_STATECHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DAQ1400_FREQUENCYCOUNTER_120:
		switch (pkt) {
		case BP_FREQUENCYDATA:
		case BP_SETDATAINTERVAL:
		case BP_SETINPUTMODE:
		case BP_SETPOWERSUPPLY:
		case BP_FREQUENCYCHANGE:
		case BP_COUNTCHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DAQ1500_VOLTAGERATIOINPUT_100:
		switch (pkt) {
		case BP_SETBRIDGEGAIN:
		case BP_SETDATAINTERVAL:
		case BP_SETCHANGETRIGGER:
		case BP_SETENABLED:
		case BP_VOLTAGERATIOCHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_VCP1100_CURRENTINPUT_100:
		switch (pkt) {
		case BP_SETDATAINTERVAL:
		case BP_SETCHANGETRIGGER:
		case BP_CURRENTCHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_VCP1100_CURRENTINPUT_200:
		switch (pkt) {
		case BP_SETDATAINTERVAL:
		case BP_SETCHANGETRIGGER:
		case BP_CURRENTCHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1000_DCMOTOR_100:
		switch (pkt) {
		case BP_SETDATAINTERVAL:
		case BP_SETACCELERATION:
		case BP_SETBRAKINGDUTYCYCLE:
		case BP_SETCURRENTLIMIT:
		case BP_SETCURRENTREGULATORGAIN:
		case BP_SETDUTYCYCLE:
		case BP_SETFANMODE:
		case BP_BRAKINGSTRENGTHCHANGE:
		case BP_DUTYCYCLECHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1000_ENCODER_100:
		switch (pkt) {
		case BP_SETDATAINTERVAL:
		case BP_SETCHANGETRIGGER:
		case BP_SETIOMODE:
		case BP_POSITIONCHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1000_VOLTAGERATIOINPUT_100:
		switch (pkt) {
		case BP_SETDATAINTERVAL:
		case BP_SETCHANGETRIGGER:
		case BP_SETSENSORTYPE:
		case BP_SETSENSORVALUECHANGETRIGGER:
		case BP_VOLTAGERATIOCHANGE:
		case BP_SENSORCHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1000_TEMPERATURESENSOR_IC_100:
		switch (pkt) {
		case BP_SETDATAINTERVAL:
		case BP_SETCHANGETRIGGER:
		case BP_TEMPERATURECHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1000_CURRENTINPUT_100:
		switch (pkt) {
		case BP_SETCHANGETRIGGER:
		case BP_SETDATAINTERVAL:
		case BP_CURRENTCHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1000_DCMOTOR_200:
		switch (pkt) {
		case BP_SETDATAINTERVAL:
		case BP_SETACCELERATION:
		case BP_SETBRAKINGDUTYCYCLE:
		case BP_SETCURRENTLIMIT:
		case BP_SETCURRENTREGULATORGAIN:
		case BP_SETDUTYCYCLE:
		case BP_SETFANMODE:
		case BP_BRAKINGSTRENGTHCHANGE:
		case BP_DUTYCYCLECHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1000_ENCODER_200:
		switch (pkt) {
		case BP_SETDATAINTERVAL:
		case BP_SETCHANGETRIGGER:
		case BP_SETIOMODE:
		case BP_POSITIONCHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1000_VOLTAGERATIOINPUT_200:
		switch (pkt) {
		case BP_SETDATAINTERVAL:
		case BP_SETCHANGETRIGGER:
		case BP_SETSENSORTYPE:
		case BP_SETSENSORVALUECHANGETRIGGER:
		case BP_VOLTAGERATIOCHANGE:
		case BP_SENSORCHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1000_TEMPERATURESENSOR_IC_200:
		switch (pkt) {
		case BP_SETDATAINTERVAL:
		case BP_SETCHANGETRIGGER:
		case BP_TEMPERATURECHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1000_CURRENTINPUT_200:
		switch (pkt) {
		case BP_SETCHANGETRIGGER:
		case BP_SETDATAINTERVAL:
		case BP_CURRENTCHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1000_MOTORPOSITIONCONTROLLER_200:
		switch (pkt) {
		case BP_SETACCELERATION:
		case BP_SETDATAINTERVAL:
		case BP_SETCURRENTLIMIT:
		case BP_SETCURRENTREGULATORGAIN:
		case BP_SETENGAGED:
		case BP_SETDUTYCYCLE:
		case BP_SETTARGETPOSITION:
		case BP_SETFANMODE:
		case BP_SETDEADBAND:
		case BP_SETIOMODE:
		case BP_SETKP:
		case BP_SETKD:
		case BP_SETKI:
		case BP_FAILSAFERESET:
		case BP_SETFAILSAFETIME:
		case BP_POSITIONCHANGE:
		case BP_DUTYCYCLECHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1000_DCMOTOR_210:
		switch (pkt) {
		case BP_SETDATAINTERVAL:
		case BP_SETACCELERATION:
		case BP_SETBRAKINGDUTYCYCLE:
		case BP_SETCURRENTLIMIT:
		case BP_SETCURRENTREGULATORGAIN:
		case BP_SETDUTYCYCLE:
		case BP_SETFANMODE:
		case BP_FAILSAFERESET:
		case BP_SETFAILSAFETIME:
		case BP_BRAKINGSTRENGTHCHANGE:
		case BP_DUTYCYCLECHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1000_ENCODER_210:
		switch (pkt) {
		case BP_SETDATAINTERVAL:
		case BP_SETCHANGETRIGGER:
		case BP_SETIOMODE:
		case BP_POSITIONCHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1000_VOLTAGERATIOINPUT_210:
		switch (pkt) {
		case BP_SETDATAINTERVAL:
		case BP_SETCHANGETRIGGER:
		case BP_SETSENSORTYPE:
		case BP_SETSENSORVALUECHANGETRIGGER:
		case BP_VOLTAGERATIOCHANGE:
		case BP_SENSORCHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1000_TEMPERATURESENSOR_IC_210:
		switch (pkt) {
		case BP_SETDATAINTERVAL:
		case BP_SETCHANGETRIGGER:
		case BP_TEMPERATURECHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1000_CURRENTINPUT_210:
		switch (pkt) {
		case BP_SETCHANGETRIGGER:
		case BP_SETDATAINTERVAL:
		case BP_CURRENTCHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1000_MOTORPOSITIONCONTROLLER_210:
		switch (pkt) {
		case BP_SETACCELERATION:
		case BP_SETDATAINTERVAL:
		case BP_SETCURRENTLIMIT:
		case BP_SETCURRENTREGULATORGAIN:
		case BP_SETENGAGED:
		case BP_SETDUTYCYCLE:
		case BP_SETTARGETPOSITION:
		case BP_SETFANMODE:
		case BP_SETDEADBAND:
		case BP_SETIOMODE:
		case BP_SETKP:
		case BP_SETKD:
		case BP_SETKI:
		case BP_FAILSAFERESET:
		case BP_SETFAILSAFETIME:
		case BP_POSITIONCHANGE:
		case BP_DUTYCYCLECHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1001_DCMOTOR_100:
		switch (pkt) {
		case BP_SETDATAINTERVAL:
		case BP_SETACCELERATION:
		case BP_SETBRAKINGDUTYCYCLE:
		case BP_SETCURRENTLIMIT:
		case BP_SETDUTYCYCLE:
		case BP_BRAKINGSTRENGTHCHANGE:
		case BP_DUTYCYCLECHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1001_ENCODER_100:
		switch (pkt) {
		case BP_SETDATAINTERVAL:
		case BP_SETCHANGETRIGGER:
		case BP_POSITIONCHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1001_MOTORPOSITIONCONTROLLER_100:
		switch (pkt) {
		case BP_SETACCELERATION:
		case BP_SETDATAINTERVAL:
		case BP_SETCURRENTLIMIT:
		case BP_SETENGAGED:
		case BP_SETDUTYCYCLE:
		case BP_SETTARGETPOSITION:
		case BP_SETDEADBAND:
		case BP_SETKP:
		case BP_SETKD:
		case BP_SETKI:
		case BP_POSITIONCHANGE:
		case BP_DUTYCYCLECHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1001_DCMOTOR_120:
		switch (pkt) {
		case BP_SETDATAINTERVAL:
		case BP_SETACCELERATION:
		case BP_SETBRAKINGDUTYCYCLE:
		case BP_SETCURRENTLIMIT:
		case BP_SETDUTYCYCLE:
		case BP_FAILSAFERESET:
		case BP_SETFAILSAFETIME:
		case BP_BRAKINGSTRENGTHCHANGE:
		case BP_DUTYCYCLECHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1001_ENCODER_120:
		switch (pkt) {
		case BP_SETDATAINTERVAL:
		case BP_SETCHANGETRIGGER:
		case BP_POSITIONCHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1001_MOTORPOSITIONCONTROLLER_120:
		switch (pkt) {
		case BP_SETACCELERATION:
		case BP_SETDATAINTERVAL:
		case BP_SETCURRENTLIMIT:
		case BP_SETENGAGED:
		case BP_SETDUTYCYCLE:
		case BP_SETTARGETPOSITION:
		case BP_SETDEADBAND:
		case BP_SETKP:
		case BP_SETKD:
		case BP_SETKI:
		case BP_FAILSAFERESET:
		case BP_SETFAILSAFETIME:
		case BP_POSITIONCHANGE:
		case BP_DUTYCYCLECHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1002_DCMOTOR_100:
		switch (pkt) {
		case BP_SETDATAINTERVAL:
		case BP_SETACCELERATION:
		case BP_SETBRAKINGDUTYCYCLE:
		case BP_SETCURRENTLIMIT:
		case BP_SETDUTYCYCLE:
		case BP_BRAKINGSTRENGTHCHANGE:
		case BP_DUTYCYCLECHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1002_ENCODER_100:
		switch (pkt) {
		case BP_SETDATAINTERVAL:
		case BP_SETCHANGETRIGGER:
		case BP_POSITIONCHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1002_MOTORPOSITIONCONTROLLER_100:
		switch (pkt) {
		case BP_SETACCELERATION:
		case BP_SETDATAINTERVAL:
		case BP_SETCURRENTLIMIT:
		case BP_SETENGAGED:
		case BP_SETDUTYCYCLE:
		case BP_SETTARGETPOSITION:
		case BP_SETDEADBAND:
		case BP_SETKP:
		case BP_SETKD:
		case BP_SETKI:
		case BP_POSITIONCHANGE:
		case BP_DUTYCYCLECHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1002_DCMOTOR_110:
		switch (pkt) {
		case BP_SETDATAINTERVAL:
		case BP_SETACCELERATION:
		case BP_SETBRAKINGDUTYCYCLE:
		case BP_SETCURRENTLIMIT:
		case BP_SETDUTYCYCLE:
		case BP_FAILSAFERESET:
		case BP_SETFAILSAFETIME:
		case BP_BRAKINGSTRENGTHCHANGE:
		case BP_DUTYCYCLECHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1002_ENCODER_110:
		switch (pkt) {
		case BP_SETDATAINTERVAL:
		case BP_SETCHANGETRIGGER:
		case BP_POSITIONCHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1002_MOTORPOSITIONCONTROLLER_110:
		switch (pkt) {
		case BP_SETACCELERATION:
		case BP_SETDATAINTERVAL:
		case BP_SETCURRENTLIMIT:
		case BP_SETENGAGED:
		case BP_SETDUTYCYCLE:
		case BP_SETTARGETPOSITION:
		case BP_SETDEADBAND:
		case BP_SETKP:
		case BP_SETKD:
		case BP_SETKI:
		case BP_FAILSAFERESET:
		case BP_SETFAILSAFETIME:
		case BP_POSITIONCHANGE:
		case BP_DUTYCYCLECHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1003_DCMOTOR_100:
		switch (pkt) {
		case BP_SETDATAINTERVAL:
		case BP_SETACCELERATION:
		case BP_SETBRAKINGDUTYCYCLE:
		case BP_SETCURRENTLIMIT:
		case BP_SETDUTYCYCLE:
		case BP_BRAKINGSTRENGTHCHANGE:
		case BP_DUTYCYCLECHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1003_DCMOTOR_110:
		switch (pkt) {
		case BP_SETDATAINTERVAL:
		case BP_SETACCELERATION:
		case BP_SETBRAKINGDUTYCYCLE:
		case BP_SETCURRENTLIMIT:
		case BP_SETDUTYCYCLE:
		case BP_FAILSAFERESET:
		case BP_SETFAILSAFETIME:
		case BP_BRAKINGSTRENGTHCHANGE:
		case BP_DUTYCYCLECHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1100_BLDCMOTOR_100:
		switch (pkt) {
		case BP_SETDATAINTERVAL:
		case BP_SETACCELERATION:
		case BP_SETBRAKINGDUTYCYCLE:
		case BP_SETDUTYCYCLE:
		case BP_SETSTALLVELOCITY:
		case BP_BRAKINGSTRENGTHCHANGE:
		case BP_POSITIONCHANGE:
		case BP_DUTYCYCLECHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1100_TEMPERATURESENSOR_IC_100:
		switch (pkt) {
		case BP_SETDATAINTERVAL:
		case BP_SETCHANGETRIGGER:
		case BP_TEMPERATURECHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1100_MOTORPOSITIONCONTROLLER_100:
		switch (pkt) {
		case BP_SETDATAINTERVAL:
		case BP_SETACCELERATION:
		case BP_SETDUTYCYCLE:
		case BP_SETENGAGED:
		case BP_SETTARGETPOSITION:
		case BP_SETDEADBAND:
		case BP_SETKP:
		case BP_SETKD:
		case BP_SETKI:
		case BP_SETSTALLVELOCITY:
		case BP_FAILSAFERESET:
		case BP_SETFAILSAFETIME:
		case BP_POSITIONCHANGE:
		case BP_DUTYCYCLECHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1100_BLDCMOTOR_120:
		switch (pkt) {
		case BP_SETDATAINTERVAL:
		case BP_SETACCELERATION:
		case BP_SETBRAKINGDUTYCYCLE:
		case BP_SETDUTYCYCLE:
		case BP_SETSTALLVELOCITY:
		case BP_FAILSAFERESET:
		case BP_SETFAILSAFETIME:
		case BP_BRAKINGSTRENGTHCHANGE:
		case BP_POSITIONCHANGE:
		case BP_DUTYCYCLECHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1100_TEMPERATURESENSOR_IC_120:
		switch (pkt) {
		case BP_SETDATAINTERVAL:
		case BP_SETCHANGETRIGGER:
		case BP_TEMPERATURECHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1100_MOTORPOSITIONCONTROLLER_120:
		switch (pkt) {
		case BP_SETDATAINTERVAL:
		case BP_SETACCELERATION:
		case BP_SETDUTYCYCLE:
		case BP_SETENGAGED:
		case BP_SETTARGETPOSITION:
		case BP_SETDEADBAND:
		case BP_SETKP:
		case BP_SETKD:
		case BP_SETKI:
		case BP_SETSTALLVELOCITY:
		case BP_FAILSAFERESET:
		case BP_SETFAILSAFETIME:
		case BP_POSITIONCHANGE:
		case BP_DUTYCYCLECHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DST1000_DISTANCESENSOR_100:
		switch (pkt) {
		case BP_SETDATAINTERVAL:
		case BP_SETCHANGETRIGGER:
		case BP_DISTANCECHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DST1001_DISTANCESENSOR_100:
		switch (pkt) {
		case BP_SETDATAINTERVAL:
		case BP_SETCHANGETRIGGER:
		case BP_DISTANCECHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DST1002_DISTANCESENSOR_100:
		switch (pkt) {
		case BP_SETDATAINTERVAL:
		case BP_SETCHANGETRIGGER:
		case BP_DISTANCECHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DST1200_DISTANCESENSOR_100:
		switch (pkt) {
		case BP_SETDATAINTERVAL:
		case BP_SETCHANGETRIGGER:
		case BP_SETSONARQUIETMODE:
		case BP_DISTANCECHANGE:
		case BP_SONARUPDATE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_ENC1000_ENCODER_100:
		switch (pkt) {
		case BP_SETDATAINTERVAL:
		case BP_SETCHANGETRIGGER:
		case BP_SETENABLED:
		case BP_SETIOMODE:
		case BP_POSITIONCHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_HIN1101_ENCODER_100:
		switch (pkt) {
		case BP_SETDATAINTERVAL:
		case BP_SETCHANGETRIGGER:
		case BP_SETENABLED:
		case BP_POSITIONCHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_HIN1101_DIGITALINPUT_100:
		switch (pkt) {
		case BP_STATECHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_HIN1000_CAPACITIVETOUCH_100:
		switch (pkt) {
		case BP_SETCHANGETRIGGER:
		case BP_SETDATAINTERVAL:
		case BP_SETSENSITIVITY:
		case BP_TOUCHINPUTVALUECHANGE:
		case BP_TOUCHINPUTEND:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_HIN1001_CAPACITIVETOUCH_BUTTONS_100:
		switch (pkt) {
		case BP_SETCHANGETRIGGER:
		case BP_SETDATAINTERVAL:
		case BP_SETSENSITIVITY:
		case BP_TOUCHINPUTVALUECHANGE:
		case BP_TOUCHINPUTEND:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_HIN1001_CAPACITIVETOUCH_WHEEL_100:
		switch (pkt) {
		case BP_SETCHANGETRIGGER:
		case BP_SETDATAINTERVAL:
		case BP_SETSENSITIVITY:
		case BP_TOUCHINPUTVALUECHANGE:
		case BP_TOUCHINPUTEND:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_HIN1100_VOLTAGERATIOINPUT_100:
		switch (pkt) {
		case BP_SETDATAINTERVAL:
		case BP_SETCHANGETRIGGER:
		case BP_VOLTAGERATIOCHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_HIN1100_DIGITALINPUT_100:
		switch (pkt) {
		case BP_STATECHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_HUM1000_HUMIDITYSENSOR_100:
		switch (pkt) {
		case BP_SETDATAINTERVAL:
		case BP_SETCHANGETRIGGER:
		case BP_HUMIDITYCHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_HUM1000_TEMPERATURESENSOR_IC_100:
		switch (pkt) {
		case BP_SETDATAINTERVAL:
		case BP_SETCHANGETRIGGER:
		case BP_TEMPERATURECHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_HUM1001_HUMIDITYSENSOR_100:
		switch (pkt) {
		case BP_SETDATAINTERVAL:
		case BP_SETCHANGETRIGGER:
		case BP_HUMIDITYCHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_HUM1001_TEMPERATURESENSOR_IC_100:
		switch (pkt) {
		case BP_SETDATAINTERVAL:
		case BP_SETCHANGETRIGGER:
		case BP_TEMPERATURECHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_LCD1100_LCD_100:
		switch (pkt) {
		case BP_SETBACKLIGHT:
		case BP_SETCONTRAST:
		case BP_SETFRAMEBUFFER:
		case BP_SETSLEEP:
		case BP_SETCHARACTERBITMAP:
		case BP_CLEAR:
		case BP_COPY:
		case BP_DRAWLINE:
		case BP_DRAWPIXEL:
		case BP_DRAWRECT:
		case BP_FLUSH:
		case BP_SETFONTSIZE:
		case BP_SAVEFRAMEBUFFER:
		case BP_WRITEBITMAP:
		case BP_WRITETEXT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_LED1000_DIGITALOUTPUT_100:
		switch (pkt) {
		case BP_SETDUTYCYCLE:
		case BP_SETLEDCURRENTLIMIT:
		case BP_SETLEDFORWARDVOLTAGE:
		case BP_SETSTATE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_LUX1000_LIGHTSENSOR_100:
		switch (pkt) {
		case BP_SETDATAINTERVAL:
		case BP_SETCHANGETRIGGER:
		case BP_ILLUMINANCECHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_HUM1100_VOLTAGERATIOINPUT_100:
		switch (pkt) {
		case BP_SETDATAINTERVAL:
		case BP_SETCHANGETRIGGER:
		case BP_VOLTAGERATIOCHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_MOT1100_ACCELEROMETER_200:
		switch (pkt) {
		case BP_SETDATAINTERVAL:
		case BP_SETCHANGETRIGGER:
		case BP_ACCELERATIONCHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_MOT1101_ACCELEROMETER_100:
		switch (pkt) {
		case BP_SETDATAINTERVAL:
		case BP_SETCHANGETRIGGER:
		case BP_ACCELERATIONCHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_MOT1101_GYROSCOPE_100:
		switch (pkt) {
		case BP_SETDATAINTERVAL:
		case BP_ZERO:
		case BP_ANGULARRATEUPDATE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_MOT1101_MAGNETOMETER_100:
		switch (pkt) {
		case BP_SETDATAINTERVAL:
		case BP_SETCHANGETRIGGER:
		case BP_SETCORRECTIONPARAMETERS:
		case BP_SAVECORRECTIONPARAMETERS:
		case BP_RESETCORRECTIONPARAMETERS:
		case BP_FIELDSTRENGTHCHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_MOT1101_SPATIAL_100:
		switch (pkt) {
		case BP_SETDATAINTERVAL:
		case BP_ZERO:
		case BP_SETCORRECTIONPARAMETERS:
		case BP_SAVECORRECTIONPARAMETERS:
		case BP_RESETCORRECTIONPARAMETERS:
		case BP_SPATIALDATA:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_MOT1102_ACCELEROMETER_200:
		switch (pkt) {
		case BP_SETDATAINTERVAL:
		case BP_SETCHANGETRIGGER:
		case BP_ACCELERATIONCHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_MOT1102_GYROSCOPE_200:
		switch (pkt) {
		case BP_SETDATAINTERVAL:
		case BP_ZERO:
		case BP_ANGULARRATEUPDATE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_MOT1102_MAGNETOMETER_200:
		switch (pkt) {
		case BP_SETDATAINTERVAL:
		case BP_SETCHANGETRIGGER:
		case BP_SETCORRECTIONPARAMETERS:
		case BP_SAVECORRECTIONPARAMETERS:
		case BP_RESETCORRECTIONPARAMETERS:
		case BP_FIELDSTRENGTHCHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_MOT1102_SPATIAL_200:
		switch (pkt) {
		case BP_SETDATAINTERVAL:
		case BP_SETSPATIALALGORITHM:
		case BP_SETSPATIALALGORITHMMAGGAIN:
		case BP_ZERO:
		case BP_SETCORRECTIONPARAMETERS:
		case BP_SAVECORRECTIONPARAMETERS:
		case BP_RESETCORRECTIONPARAMETERS:
		case BP_SETAHRSPARAMETERS:
		case BP_ZEROSPATIALALGORITHM:
		case BP_SPATIALDATA:
		case BP_SPATIALALGDATA:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_MOT0109_ACCELEROMETER_100:
		switch (pkt) {
		case BP_DATAINTERVALCHANGE:
		case BP_SETDATAINTERVAL:
		case BP_SETCHANGETRIGGER:
		case BP_SETSPATIALPRECISION:
		case BP_SETHEATINGENABLED:
		case BP_ACCELERATIONCHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_MOT0109_GYROSCOPE_100:
		switch (pkt) {
		case BP_DATAINTERVALCHANGE:
		case BP_SETDATAINTERVAL:
		case BP_SETSPATIALPRECISION:
		case BP_SETHEATINGENABLED:
		case BP_ZERO:
		case BP_ANGULARRATEUPDATE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_MOT0109_MAGNETOMETER_100:
		switch (pkt) {
		case BP_DATAINTERVALCHANGE:
		case BP_SETDATAINTERVAL:
		case BP_SETCHANGETRIGGER:
		case BP_SETHEATINGENABLED:
		case BP_SETCORRECTIONPARAMETERS:
		case BP_SAVECORRECTIONPARAMETERS:
		case BP_RESETCORRECTIONPARAMETERS:
		case BP_FIELDSTRENGTHCHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_MOT0109_SPATIAL_100:
		switch (pkt) {
		case BP_DATAINTERVALCHANGE:
		case BP_SETDATAINTERVAL:
		case BP_SETSPATIALPRECISION:
		case BP_SETSPATIALALGORITHM:
		case BP_SETSPATIALALGORITHMMAGGAIN:
		case BP_SETHEATINGENABLED:
		case BP_ZERO:
		case BP_SETCORRECTIONPARAMETERS:
		case BP_SAVECORRECTIONPARAMETERS:
		case BP_RESETCORRECTIONPARAMETERS:
		case BP_SETAHRSPARAMETERS:
		case BP_ZEROSPATIALALGORITHM:
		case BP_SPATIALDATA:
		case BP_SPATIALALGDATA:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_MOT0109_TEMPERATURESENSOR_100:
		switch (pkt) {
		case BP_DATAINTERVALCHANGE:
		case BP_SETDATAINTERVAL:
		case BP_SETCHANGETRIGGER:
		case BP_TEMPERATURECHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_PRE1000_PRESSURESENSOR_100:
		switch (pkt) {
		case BP_SETDATAINTERVAL:
		case BP_SETCHANGETRIGGER:
		case BP_PRESSURECHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_RCC1000_RCSERVO_100:
		switch (pkt) {
		case BP_SETMAXPULSEWIDTH:
		case BP_SETMINPULSEWIDTH:
		case BP_SETTARGETPOSITION:
		case BP_SETENGAGED:
		case BP_SETVOLTAGE:
		case BP_SETVELOCITYLIMIT:
		case BP_SETACCELERATION:
		case BP_SETSPEEDRAMPINGSTATE:
		case BP_TARGETPOSITIONREACHED:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_RCC1000_RCSERVO_110:
		switch (pkt) {
		case BP_SETMAXPULSEWIDTH:
		case BP_SETMINPULSEWIDTH:
		case BP_SETTARGETPOSITION:
		case BP_SETENGAGED:
		case BP_SETVOLTAGE:
		case BP_SETVELOCITYLIMIT:
		case BP_SETACCELERATION:
		case BP_SETSPEEDRAMPINGSTATE:
		case BP_FAILSAFERESET:
		case BP_SETFAILSAFETIME:
		case BP_TARGETPOSITIONREACHED:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_REL1000_DIGITALOUTPUT_100:
		switch (pkt) {
		case BP_SETDUTYCYCLE:
		case BP_SETSTATE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_REL1000_DIGITALOUTPUT_110:
		switch (pkt) {
		case BP_SETDUTYCYCLE:
		case BP_SETSTATE:
		case BP_FAILSAFERESET:
		case BP_SETFAILSAFETIME:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_REL1100_DIGITALOUTPUT_100:
		switch (pkt) {
		case BP_SETDUTYCYCLE:
		case BP_SETSTATE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_REL1100_DIGITALOUTPUT_110:
		switch (pkt) {
		case BP_SETDUTYCYCLE:
		case BP_SETSTATE:
		case BP_FAILSAFERESET:
		case BP_SETFAILSAFETIME:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_REL1100_DIGITALOUTPUT_120:
		switch (pkt) {
		case BP_SETDUTYCYCLE:
		case BP_SETSTATE:
		case BP_SETFREQUENCY:
		case BP_FAILSAFERESET:
		case BP_SETFAILSAFETIME:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_REL1101_DIGITALOUTPUT_100:
		switch (pkt) {
		case BP_SETDUTYCYCLE:
		case BP_SETSTATE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_REL1101_DIGITALOUTPUT_110:
		switch (pkt) {
		case BP_SETDUTYCYCLE:
		case BP_SETSTATE:
		case BP_FAILSAFERESET:
		case BP_SETFAILSAFETIME:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_REL1101_1_DIGITALOUTPUT_FREQ_100:
		switch (pkt) {
		case BP_SETDUTYCYCLE:
		case BP_SETSTATE:
		case BP_SETFREQUENCY:
		case BP_FAILSAFERESET:
		case BP_SETFAILSAFETIME:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_REL1101_1_DIGITALOUTPUT_100:
		switch (pkt) {
		case BP_SETDUTYCYCLE:
		case BP_SETSTATE:
		case BP_FAILSAFERESET:
		case BP_SETFAILSAFETIME:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_SAF1000_POWERGUARD_100:
		switch (pkt) {
		case BP_SETENABLED:
		case BP_SETOVERVOLTAGE:
		case BP_SETFANMODE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_SAF1000_VOLTAGEINPUT_100:
		switch (pkt) {
		case BP_SETDATAINTERVAL:
		case BP_SETCHANGETRIGGER:
		case BP_VOLTAGECHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_SAF1000_TEMPERATURESENSOR_IC_100:
		switch (pkt) {
		case BP_SETDATAINTERVAL:
		case BP_SETCHANGETRIGGER:
		case BP_TEMPERATURECHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_SAF1000_POWERGUARD_110:
		switch (pkt) {
		case BP_SETENABLED:
		case BP_SETOVERVOLTAGE:
		case BP_SETFANMODE:
		case BP_FAILSAFERESET:
		case BP_SETFAILSAFETIME:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_SAF1000_VOLTAGEINPUT_110:
		switch (pkt) {
		case BP_SETDATAINTERVAL:
		case BP_SETCHANGETRIGGER:
		case BP_VOLTAGECHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_SAF1000_TEMPERATURESENSOR_IC_110:
		switch (pkt) {
		case BP_SETDATAINTERVAL:
		case BP_SETCHANGETRIGGER:
		case BP_TEMPERATURECHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_SND1000_SOUNDSENSOR_100:
		switch (pkt) {
		case BP_SETDATAINTERVAL:
		case BP_SETSPLRANGE:
		case BP_SETCHANGETRIGGER:
		case BP_DBCHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_STC1000_STEPPER_100:
		switch (pkt) {
		case BP_SETDATAINTERVAL:
		case BP_SETTARGETPOSITION:
		case BP_SETVELOCITYLIMIT:
		case BP_SETACCELERATION:
		case BP_SETCURRENTLIMIT:
		case BP_SETENGAGED:
		case BP_SETCONTROLMODE:
		case BP_SETHOLDINGCURRENTLIMIT:
		case BP_POSITIONCHANGE:
		case BP_VELOCITYCHANGE:
		case BP_STOPPED:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_STC1000_STEPPER_110:
		switch (pkt) {
		case BP_SETDATAINTERVAL:
		case BP_SETTARGETPOSITION:
		case BP_SETVELOCITYLIMIT:
		case BP_SETACCELERATION:
		case BP_SETCURRENTLIMIT:
		case BP_SETENGAGED:
		case BP_SETCONTROLMODE:
		case BP_SETHOLDINGCURRENTLIMIT:
		case BP_FAILSAFERESET:
		case BP_SETFAILSAFETIME:
		case BP_POSITIONCHANGE:
		case BP_VELOCITYCHANGE:
		case BP_STOPPED:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_STC1001_STEPPER_100:
		switch (pkt) {
		case BP_SETDATAINTERVAL:
		case BP_SETTARGETPOSITION:
		case BP_SETVELOCITYLIMIT:
		case BP_SETACCELERATION:
		case BP_SETCURRENTLIMIT:
		case BP_SETENGAGED:
		case BP_SETCONTROLMODE:
		case BP_SETHOLDINGCURRENTLIMIT:
		case BP_POSITIONCHANGE:
		case BP_VELOCITYCHANGE:
		case BP_STOPPED:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_STC1001_STEPPER_110:
		switch (pkt) {
		case BP_SETDATAINTERVAL:
		case BP_SETTARGETPOSITION:
		case BP_SETVELOCITYLIMIT:
		case BP_SETACCELERATION:
		case BP_SETCURRENTLIMIT:
		case BP_SETENGAGED:
		case BP_SETCONTROLMODE:
		case BP_SETHOLDINGCURRENTLIMIT:
		case BP_FAILSAFERESET:
		case BP_SETFAILSAFETIME:
		case BP_POSITIONCHANGE:
		case BP_VELOCITYCHANGE:
		case BP_STOPPED:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_STC1002_STEPPER_100:
		switch (pkt) {
		case BP_SETDATAINTERVAL:
		case BP_SETTARGETPOSITION:
		case BP_SETVELOCITYLIMIT:
		case BP_SETACCELERATION:
		case BP_SETCURRENTLIMIT:
		case BP_SETENGAGED:
		case BP_SETCONTROLMODE:
		case BP_SETHOLDINGCURRENTLIMIT:
		case BP_POSITIONCHANGE:
		case BP_VELOCITYCHANGE:
		case BP_STOPPED:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_STC1002_STEPPER_110:
		switch (pkt) {
		case BP_SETDATAINTERVAL:
		case BP_SETTARGETPOSITION:
		case BP_SETVELOCITYLIMIT:
		case BP_SETACCELERATION:
		case BP_SETCURRENTLIMIT:
		case BP_SETENGAGED:
		case BP_SETCONTROLMODE:
		case BP_SETHOLDINGCURRENTLIMIT:
		case BP_FAILSAFERESET:
		case BP_SETFAILSAFETIME:
		case BP_POSITIONCHANGE:
		case BP_VELOCITYCHANGE:
		case BP_STOPPED:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_STC1003_STEPPER_100:
		switch (pkt) {
		case BP_SETDATAINTERVAL:
		case BP_SETTARGETPOSITION:
		case BP_SETVELOCITYLIMIT:
		case BP_SETACCELERATION:
		case BP_SETCURRENTLIMIT:
		case BP_SETENGAGED:
		case BP_SETCONTROLMODE:
		case BP_SETHOLDINGCURRENTLIMIT:
		case BP_POSITIONCHANGE:
		case BP_VELOCITYCHANGE:
		case BP_STOPPED:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_STC1003_STEPPER_110:
		switch (pkt) {
		case BP_SETDATAINTERVAL:
		case BP_SETTARGETPOSITION:
		case BP_SETVELOCITYLIMIT:
		case BP_SETACCELERATION:
		case BP_SETCURRENTLIMIT:
		case BP_SETENGAGED:
		case BP_SETCONTROLMODE:
		case BP_SETHOLDINGCURRENTLIMIT:
		case BP_FAILSAFERESET:
		case BP_SETFAILSAFETIME:
		case BP_POSITIONCHANGE:
		case BP_VELOCITYCHANGE:
		case BP_STOPPED:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_STC1003_STEPPER_200:
		switch (pkt) {
		case BP_SETDATAINTERVAL:
		case BP_SETTARGETPOSITION:
		case BP_SETVELOCITYLIMIT:
		case BP_SETACCELERATION:
		case BP_SETCURRENTLIMIT:
		case BP_SETENGAGED:
		case BP_SETCONTROLMODE:
		case BP_SETHOLDINGCURRENTLIMIT:
		case BP_FAILSAFERESET:
		case BP_SETFAILSAFETIME:
		case BP_POSITIONCHANGE:
		case BP_VELOCITYCHANGE:
		case BP_STOPPED:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_STC1003_VOLTAGEINPUT_200:
		switch (pkt) {
		case BP_SETDATAINTERVAL:
		case BP_SETCHANGETRIGGER:
		case BP_VOLTAGECHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_TMP1000_TEMPERATURESENSOR_IC_100:
		switch (pkt) {
		case BP_SETDATAINTERVAL:
		case BP_SETCHANGETRIGGER:
		case BP_TEMPERATURECHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_TMP1100_TEMPERATURESENSOR_THERMOCOUPLE_100:
		switch (pkt) {
		case BP_SETDATAINTERVAL:
		case BP_SETCHANGETRIGGER:
		case BP_SETTHERMOCOUPLETYPE:
		case BP_TEMPERATURECHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_TMP1100_TEMPERATURESENSOR_IC_100:
		switch (pkt) {
		case BP_SETDATAINTERVAL:
		case BP_SETCHANGETRIGGER:
		case BP_TEMPERATURECHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_TMP1100_VOLTAGEINPUT_100:
		switch (pkt) {
		case BP_SETDATAINTERVAL:
		case BP_SETCHANGETRIGGER:
		case BP_VOLTAGECHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_TMP1101_TEMPERATURESENSOR_THERMOCOUPLE_100:
		switch (pkt) {
		case BP_SETDATAINTERVAL:
		case BP_SETCHANGETRIGGER:
		case BP_SETTHERMOCOUPLETYPE:
		case BP_TEMPERATURECHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_TMP1101_TEMPERATURESENSOR_IC_100:
		switch (pkt) {
		case BP_SETDATAINTERVAL:
		case BP_SETCHANGETRIGGER:
		case BP_TEMPERATURECHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_TMP1101_VOLTAGEINPUT_100:
		switch (pkt) {
		case BP_SETDATAINTERVAL:
		case BP_SETCHANGETRIGGER:
		case BP_VOLTAGECHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_TMP1101_TEMPERATURESENSOR_THERMOCOUPLE_200:
		switch (pkt) {
		case BP_SETDATAINTERVAL:
		case BP_SETCHANGETRIGGER:
		case BP_SETTHERMOCOUPLETYPE:
		case BP_TEMPERATURECHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_TMP1101_TEMPERATURESENSOR_IC_200:
		switch (pkt) {
		case BP_SETDATAINTERVAL:
		case BP_SETCHANGETRIGGER:
		case BP_TEMPERATURECHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_TMP1101_VOLTAGEINPUT_200:
		switch (pkt) {
		case BP_SETDATAINTERVAL:
		case BP_SETCHANGETRIGGER:
		case BP_VOLTAGECHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_TMP1200_TEMPERATURESENSOR_RTD_100:
		switch (pkt) {
		case BP_SETDATAINTERVAL:
		case BP_SETCHANGETRIGGER:
		case BP_SETRTDTYPE:
		case BP_SETRTDWIRESETUP:
		case BP_TEMPERATURECHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_TMP1200_RESISTANCEINPUT_100:
		switch (pkt) {
		case BP_SETDATAINTERVAL:
		case BP_SETCHANGETRIGGER:
		case BP_SETRTDWIRESETUP:
		case BP_RESISTANCECHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_VCP1000_VOLTAGEINPUT_100:
		switch (pkt) {
		case BP_SETDATAINTERVAL:
		case BP_SETCHANGETRIGGER:
		case BP_SETVOLTAGERANGE:
		case BP_VOLTAGECHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_VCP1001_VOLTAGEINPUT_100:
		switch (pkt) {
		case BP_SETDATAINTERVAL:
		case BP_SETCHANGETRIGGER:
		case BP_SETVOLTAGERANGE:
		case BP_VOLTAGECHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_VCP1001_VOLTAGEINPUT_110:
		switch (pkt) {
		case BP_SETDATAINTERVAL:
		case BP_SETCHANGETRIGGER:
		case BP_SETVOLTAGERANGE:
		case BP_VOLTAGECHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_VCP1002_VOLTAGEINPUT_100:
		switch (pkt) {
		case BP_SETDATAINTERVAL:
		case BP_SETCHANGETRIGGER:
		case BP_SETVOLTAGERANGE:
		case BP_VOLTAGECHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_VCP1002_VOLTAGEINPUT_110:
		switch (pkt) {
		case BP_SETDATAINTERVAL:
		case BP_SETCHANGETRIGGER:
		case BP_SETVOLTAGERANGE:
		case BP_VOLTAGECHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_HUB0000_HUB_100:
		switch (pkt) {
		case BP_SETFIRMWAREUPGRADEFLAG:
		case BP_SETPORTMODE:
		case BP_SETPORTPOWER:
		case BP_SETCALIBRATIONVALUES:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_HUB0000_HUB_300:
		switch (pkt) {
		case BP_SETFIRMWAREUPGRADEFLAG:
		case BP_SETPORTMODE:
		case BP_SETPORTPOWER:
		case BP_SETCALIBRATIONVALUES:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_HUB0001_HUB_100:
		switch (pkt) {
		case BP_SETFIRMWAREUPGRADEFLAG:
		case BP_SETPORTMODE:
		case BP_SETPORTPOWER:
		case BP_SETCALIBRATIONVALUES:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_HUB0004_HUB_100:
		switch (pkt) {
		case BP_SETFIRMWAREUPGRADEFLAG:
		case BP_SETPORTMODE:
		case BP_SETPORTPOWER:
		case BP_SETCALIBRATIONVALUES:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_HUB0004_HUB_200:
		switch (pkt) {
		case BP_SETFIRMWAREUPGRADEFLAG:
		case BP_SETPORTMODE:
		case BP_SETPORTPOWER:
		case BP_SETCALIBRATIONVALUES:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_HUB5000_HUB_100:
		switch (pkt) {
		case BP_SETFIRMWAREUPGRADEFLAG:
		case BP_SETPORTMODE:
		case BP_SETPORTPOWER:
		case BP_SETCALIBRATIONVALUES:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_HUB5000_HUB_200:
		switch (pkt) {
		case BP_SETFIRMWAREUPGRADEFLAG:
		case BP_SETPORTMODE:
		case BP_SETPORTPOWER:
		case BP_SETCALIBRATIONVALUES:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_M3_USB_FIRMWARE_UPGRADE_000:
		switch (pkt) {
		case BP_SENDFIRMWARE:
		case BP_PROGRESSCHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_STM32_USB_FIRMWARE_UPGRADE_100:
		switch (pkt) {
		case BP_SENDFIRMWARE:
		case BP_PROGRESSCHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_STM32F0_FIRMWARE_UPGRADE_100:
		switch (pkt) {
		case BP_DEVICEINFO:
		case BP_SENDFIRMWARE:
		case BP_PROGRESSCHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_STM32G0_FIRMWARE_UPGRADE_110:
		switch (pkt) {
		case BP_DEVICEINFO:
		case BP_SENDFIRMWARE:
		case BP_PROGRESSCHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_STM8S_FIRMWARE_UPGRADE_100:
		switch (pkt) {
		case BP_DEVICEINFO:
		case BP_SENDFIRMWARE:
		case BP_PROGRESSCHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_M3_SPI_FIRMWARE_UPGRADE_000:
		switch (pkt) {
		case BP_SENDFIRMWARE:
		case BP_PROGRESSCHANGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DICTIONARY:
		switch (pkt) {
		case BP_DICTIONARYADD:
		case BP_DICTIONARYUPDATE:
		case BP_DICTIONARYSET:
		case BP_DICTIONARYREMOVE:
		case BP_DICTIONARYREMOVEALL:
		case BP_DICTIONARYGET:
		case BP_DICTIONARYSCAN:
		case BP_DICTIONARYADDED:
		case BP_DICTIONARYUPDATED:
		case BP_DICTIONARYREMOVED:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_USB_UNKNOWN:
		switch (pkt) {
		case BP_SENDPACKET:
		case BP_SENDCHPACKET:
		case BP_SENDDEVPACKET:
		case BP_READCHPACKET:
		case BP_READDEVPACKET:
		case BP_PACKET:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_VINT_UNKNOWN:
		switch (pkt) {
		case BP_SENDPACKET:
		case BP_SENDCHPACKET:
		case BP_SENDDEVPACKET:
		case BP_READCHPACKET:
		case BP_READDEVPACKET:
		case BP_PACKET:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_SPI_UNKNOWN:
		switch (pkt) {
		case BP_SENDPACKET:
		case BP_SENDCHPACKET:
		case BP_SENDDEVPACKET:
		case BP_READCHPACKET:
		case BP_READDEVPACKET:
		case BP_PACKET:
			return (1);
		default:
			return (0);
		}
	default:
		return (0);
	}
}
