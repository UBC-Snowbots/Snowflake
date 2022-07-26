#include "phidgetbase.h"
#include "phidget.h"

API_IRETURN
Phidget_enumFromString(const char *name) {

	if (mos_strcasecmp(name, "ENCODER_IO_MODE_PUSH_PULL") == 0)
		return (1);

	if (mos_strcasecmp(name, "ENCODER_IO_MODE_LINE_DRIVER_2K2") == 0)
		return (2);

	if (mos_strcasecmp(name, "ENCODER_IO_MODE_LINE_DRIVER_10K") == 0)
		return (3);

	if (mos_strcasecmp(name, "ENCODER_IO_MODE_OPEN_COLLECTOR_2K2") == 0)
		return (4);

	if (mos_strcasecmp(name, "ENCODER_IO_MODE_OPEN_COLLECTOR_10K") == 0)
		return (5);

	if (mos_strcasecmp(name, "EPHIDGET_OK") == 0)
		return (0);

	if (mos_strcasecmp(name, "EPHIDGET_UNKNOWNVALLOW") == 0)
		return (61);

	if (mos_strcasecmp(name, "EPHIDGET_NOENT") == 0)
		return (2);

	if (mos_strcasecmp(name, "EPHIDGET_TIMEOUT") == 0)
		return (3);

	if (mos_strcasecmp(name, "EPHIDGET_KEEPALIVE") == 0)
		return (58);

	if (mos_strcasecmp(name, "EPHIDGET_INTERRUPTED") == 0)
		return (4);

	if (mos_strcasecmp(name, "EPHIDGET_IO") == 0)
		return (5);

	if (mos_strcasecmp(name, "EPHIDGET_NOMEMORY") == 0)
		return (6);

	if (mos_strcasecmp(name, "EPHIDGET_ACCESS") == 0)
		return (7);

	if (mos_strcasecmp(name, "EPHIDGET_FAULT") == 0)
		return (8);

	if (mos_strcasecmp(name, "EPHIDGET_BUSY") == 0)
		return (9);

	if (mos_strcasecmp(name, "EPHIDGET_EXIST") == 0)
		return (10);

	if (mos_strcasecmp(name, "EPHIDGET_NOTDIR") == 0)
		return (11);

	if (mos_strcasecmp(name, "EPHIDGET_ISDIR") == 0)
		return (12);

	if (mos_strcasecmp(name, "EPHIDGET_INVALID") == 0)
		return (13);

	if (mos_strcasecmp(name, "EPHIDGET_NFILE") == 0)
		return (14);

	if (mos_strcasecmp(name, "EPHIDGET_MFILE") == 0)
		return (15);

	if (mos_strcasecmp(name, "EPHIDGET_NOSPC") == 0)
		return (16);

	if (mos_strcasecmp(name, "EPHIDGET_FBIG") == 0)
		return (17);

	if (mos_strcasecmp(name, "EPHIDGET_ROFS") == 0)
		return (18);

	if (mos_strcasecmp(name, "EPHIDGET_RO") == 0)
		return (19);

	if (mos_strcasecmp(name, "EPHIDGET_UNSUPPORTED") == 0)
		return (20);

	if (mos_strcasecmp(name, "EPHIDGET_INVALIDARG") == 0)
		return (21);

	if (mos_strcasecmp(name, "EPHIDGET_PERM") == 0)
		return (1);

	if (mos_strcasecmp(name, "EPHIDGET_NOTEMPTY") == 0)
		return (26);

	if (mos_strcasecmp(name, "EPHIDGET_UNEXPECTED") == 0)
		return (28);

	if (mos_strcasecmp(name, "EPHIDGET_DUPLICATE") == 0)
		return (27);

	if (mos_strcasecmp(name, "EPHIDGET_BADPASSWORD") == 0)
		return (37);

	if (mos_strcasecmp(name, "EPHIDGET_NETUNAVAIL") == 0)
		return (45);

	if (mos_strcasecmp(name, "EPHIDGET_CONNREF") == 0)
		return (35);

	if (mos_strcasecmp(name, "EPHIDGET_CONNRESET") == 0)
		return (46);

	if (mos_strcasecmp(name, "EPHIDGET_HOSTUNREACH") == 0)
		return (48);

	if (mos_strcasecmp(name, "EPHIDGET_NODEV") == 0)
		return (40);

	if (mos_strcasecmp(name, "EPHIDGET_WRONGDEVICE") == 0)
		return (50);

	if (mos_strcasecmp(name, "EPHIDGET_PIPE") == 0)
		return (41);

	if (mos_strcasecmp(name, "EPHIDGET_RESOLV") == 0)
		return (44);

	if (mos_strcasecmp(name, "EPHIDGET_UNKNOWNVAL") == 0)
		return (51);

	if (mos_strcasecmp(name, "EPHIDGET_NOTATTACHED") == 0)
		return (52);

	if (mos_strcasecmp(name, "EPHIDGET_INVALIDPACKET") == 0)
		return (53);

	if (mos_strcasecmp(name, "EPHIDGET_2BIG") == 0)
		return (54);

	if (mos_strcasecmp(name, "EPHIDGET_BADVERSION") == 0)
		return (55);

	if (mos_strcasecmp(name, "EPHIDGET_CLOSED") == 0)
		return (56);

	if (mos_strcasecmp(name, "EPHIDGET_NOTCONFIGURED") == 0)
		return (57);

	if (mos_strcasecmp(name, "EPHIDGET_EOF") == 0)
		return (31);

	if (mos_strcasecmp(name, "EPHIDGET_FAILSAFE") == 0)
		return (59);

	if (mos_strcasecmp(name, "EPHIDGET_UNKNOWNVALHIGH") == 0)
		return (60);

	if (mos_strcasecmp(name, "EPHIDGET_AGAIN") == 0)
		return (22);

	if (mos_strcasecmp(name, "EEPHIDGET_BADVERSION") == 0)
		return (1);

	if (mos_strcasecmp(name, "EEPHIDGET_OUTOFRANGELOW") == 0)
		return (4115);

	if (mos_strcasecmp(name, "EEPHIDGET_NETWORK") == 0)
		return (3);

	if (mos_strcasecmp(name, "EEPHIDGET_DISPATCH") == 0)
		return (4);

	if (mos_strcasecmp(name, "EEPHIDGET_FAILURE") == 0)
		return (5);

	if (mos_strcasecmp(name, "EEPHIDGET_OK") == 0)
		return (4096);

	if (mos_strcasecmp(name, "EEPHIDGET_OVERRUN") == 0)
		return (4098);

	if (mos_strcasecmp(name, "EEPHIDGET_PACKETLOST") == 0)
		return (4099);

	if (mos_strcasecmp(name, "EEPHIDGET_WRAP") == 0)
		return (4100);

	if (mos_strcasecmp(name, "EEPHIDGET_OVERTEMP") == 0)
		return (4101);

	if (mos_strcasecmp(name, "EEPHIDGET_OVERCURRENT") == 0)
		return (4102);

	if (mos_strcasecmp(name, "EEPHIDGET_BUSY") == 0)
		return (2);

	if (mos_strcasecmp(name, "EEPHIDGET_BADPOWER") == 0)
		return (4104);

	if (mos_strcasecmp(name, "EEPHIDGET_SATURATION") == 0)
		return (4105);

	if (mos_strcasecmp(name, "EEPHIDGET_OVERVOLTAGE") == 0)
		return (4107);

	if (mos_strcasecmp(name, "EEPHIDGET_FAILSAFE") == 0)
		return (4108);

	if (mos_strcasecmp(name, "EEPHIDGET_VOLTAGEERROR") == 0)
		return (4109);

	if (mos_strcasecmp(name, "EEPHIDGET_ENERGYDUMP") == 0)
		return (4110);

	if (mos_strcasecmp(name, "EEPHIDGET_MOTORSTALL") == 0)
		return (4111);

	if (mos_strcasecmp(name, "EEPHIDGET_INVALIDSTATE") == 0)
		return (4112);

	if (mos_strcasecmp(name, "EEPHIDGET_BADCONNECTION") == 0)
		return (4113);

	if (mos_strcasecmp(name, "EEPHIDGET_OUTOFRANGEHIGH") == 0)
		return (4114);

	if (mos_strcasecmp(name, "EEPHIDGET_OUTOFRANGE") == 0)
		return (4103);

	if (mos_strcasecmp(name, "PHIDID_NOTHING") == 0)
		return (0);

	if (mos_strcasecmp(name, "PHIDID_UNKNOWN") == 0)
		return (125);

	if (mos_strcasecmp(name, "PHIDID_DIGITALINPUT_PORT") == 0)
		return (95);

	if (mos_strcasecmp(name, "PHIDID_DIGITALOUTPUT_PORT") == 0)
		return (96);

	if (mos_strcasecmp(name, "PHIDID_VOLTAGEINPUT_PORT") == 0)
		return (97);

	if (mos_strcasecmp(name, "PHIDID_VOLTAGERATIOINPUT_PORT") == 0)
		return (98);

	if (mos_strcasecmp(name, "PHIDID_DICTIONARY") == 0)
		return (111);

	if (mos_strcasecmp(name, "PHIDID_1000") == 0)
		return (2);

	if (mos_strcasecmp(name, "PHIDID_1001") == 0)
		return (3);

	if (mos_strcasecmp(name, "PHIDID_1002") == 0)
		return (4);

	if (mos_strcasecmp(name, "PHIDID_1008") == 0)
		return (5);

	if (mos_strcasecmp(name, "PHIDID_1010_1013_1018_1019") == 0)
		return (6);

	if (mos_strcasecmp(name, "PHIDID_1011") == 0)
		return (7);

	if (mos_strcasecmp(name, "PHIDID_1012") == 0)
		return (8);

	if (mos_strcasecmp(name, "PHIDID_1014") == 0)
		return (9);

	if (mos_strcasecmp(name, "PHIDID_1015") == 0)
		return (10);

	if (mos_strcasecmp(name, "PHIDID_1016") == 0)
		return (11);

	if (mos_strcasecmp(name, "PHIDID_1017") == 0)
		return (12);

	if (mos_strcasecmp(name, "PHIDID_1023") == 0)
		return (13);

	if (mos_strcasecmp(name, "PHIDID_1024") == 0)
		return (14);

	if (mos_strcasecmp(name, "PHIDID_1030") == 0)
		return (15);

	if (mos_strcasecmp(name, "PHIDID_1031") == 0)
		return (16);

	if (mos_strcasecmp(name, "PHIDID_1032") == 0)
		return (17);

	if (mos_strcasecmp(name, "PHIDID_1040") == 0)
		return (18);

	if (mos_strcasecmp(name, "PHIDID_1041") == 0)
		return (19);

	if (mos_strcasecmp(name, "PHIDID_1042") == 0)
		return (20);

	if (mos_strcasecmp(name, "PHIDID_1043") == 0)
		return (21);

	if (mos_strcasecmp(name, "PHIDID_1044") == 0)
		return (22);

	if (mos_strcasecmp(name, "PHIDID_1045") == 0)
		return (23);

	if (mos_strcasecmp(name, "PHIDID_1046") == 0)
		return (24);

	if (mos_strcasecmp(name, "PHIDID_1047") == 0)
		return (25);

	if (mos_strcasecmp(name, "PHIDID_1048") == 0)
		return (26);

	if (mos_strcasecmp(name, "PHIDID_1049") == 0)
		return (27);

	if (mos_strcasecmp(name, "PHIDID_1051") == 0)
		return (28);

	if (mos_strcasecmp(name, "PHIDID_1052") == 0)
		return (29);

	if (mos_strcasecmp(name, "PHIDID_1053") == 0)
		return (30);

	if (mos_strcasecmp(name, "PHIDID_1054") == 0)
		return (31);

	if (mos_strcasecmp(name, "PHIDID_1055") == 0)
		return (32);

	if (mos_strcasecmp(name, "PHIDID_1056") == 0)
		return (33);

	if (mos_strcasecmp(name, "PHIDID_1057") == 0)
		return (34);

	if (mos_strcasecmp(name, "PHIDID_1058") == 0)
		return (35);

	if (mos_strcasecmp(name, "PHIDID_1059") == 0)
		return (36);

	if (mos_strcasecmp(name, "PHIDID_1060") == 0)
		return (37);

	if (mos_strcasecmp(name, "PHIDID_1061") == 0)
		return (38);

	if (mos_strcasecmp(name, "PHIDID_1062") == 0)
		return (39);

	if (mos_strcasecmp(name, "PHIDID_1063") == 0)
		return (40);

	if (mos_strcasecmp(name, "PHIDID_1064") == 0)
		return (41);

	if (mos_strcasecmp(name, "PHIDID_1065") == 0)
		return (42);

	if (mos_strcasecmp(name, "PHIDID_1066") == 0)
		return (43);

	if (mos_strcasecmp(name, "PHIDID_1067") == 0)
		return (44);

	if (mos_strcasecmp(name, "PHIDID_1202_1203") == 0)
		return (45);

	if (mos_strcasecmp(name, "PHIDID_1204") == 0)
		return (46);

	if (mos_strcasecmp(name, "PHIDID_1215__1218") == 0)
		return (47);

	if (mos_strcasecmp(name, "PHIDID_1219__1222") == 0)
		return (48);

	if (mos_strcasecmp(name, "PHIDID_ADP1000") == 0)
		return (49);

	if (mos_strcasecmp(name, "PHIDID_DAQ1000") == 0)
		return (51);

	if (mos_strcasecmp(name, "PHIDID_DAQ1200") == 0)
		return (52);

	if (mos_strcasecmp(name, "PHIDID_DAQ1300") == 0)
		return (53);

	if (mos_strcasecmp(name, "PHIDID_DAQ1301") == 0)
		return (54);

	if (mos_strcasecmp(name, "PHIDID_DAQ1400") == 0)
		return (55);

	if (mos_strcasecmp(name, "PHIDID_DAQ1500") == 0)
		return (56);

	if (mos_strcasecmp(name, "PHIDID_DCC1000") == 0)
		return (57);

	if (mos_strcasecmp(name, "PHIDID_DCC1001") == 0)
		return (110);

	if (mos_strcasecmp(name, "PHIDID_DCC1002") == 0)
		return (117);

	if (mos_strcasecmp(name, "PHIDID_DCC1003") == 0)
		return (120);

	if (mos_strcasecmp(name, "PHIDID_DCC1100") == 0)
		return (108);

	if (mos_strcasecmp(name, "PHIDID_DST1000") == 0)
		return (58);

	if (mos_strcasecmp(name, "PHIDID_DST1001") == 0)
		return (121);

	if (mos_strcasecmp(name, "PHIDID_DST1002") == 0)
		return (126);

	if (mos_strcasecmp(name, "PHIDID_DST1200") == 0)
		return (59);

	if (mos_strcasecmp(name, "PHIDID_ENC1000") == 0)
		return (60);

	if (mos_strcasecmp(name, "PHIDID_FIRMWARE_UPGRADE_SPI") == 0)
		return (104);

	if (mos_strcasecmp(name, "PHIDID_FIRMWARE_UPGRADE_STM32F0") == 0)
		return (102);

	if (mos_strcasecmp(name, "PHIDID_FIRMWARE_UPGRADE_STM32G0") == 0)
		return (143);

	if (mos_strcasecmp(name, "PHIDID_FIRMWARE_UPGRADE_STM8S") == 0)
		return (103);

	if (mos_strcasecmp(name, "PHIDID_FIRMWARE_UPGRADE_USB") == 0)
		return (101);

	if (mos_strcasecmp(name, "PHIDID_HIN1000") == 0)
		return (61);

	if (mos_strcasecmp(name, "PHIDID_HIN1001") == 0)
		return (62);

	if (mos_strcasecmp(name, "PHIDID_HIN1100") == 0)
		return (63);

	if (mos_strcasecmp(name, "PHIDID_HIN1101") == 0)
		return (109);

	if (mos_strcasecmp(name, "PHIDID_HUB0000") == 0)
		return (64);

	if (mos_strcasecmp(name, "PHIDID_HUB0001") == 0)
		return (142);

	if (mos_strcasecmp(name, "PHIDID_HUB0004") == 0)
		return (67);

	if (mos_strcasecmp(name, "PHIDID_HUB5000") == 0)
		return (123);

	if (mos_strcasecmp(name, "PHIDID_HUM1000") == 0)
		return (69);

	if (mos_strcasecmp(name, "PHIDID_HUM1001") == 0)
		return (127);

	if (mos_strcasecmp(name, "PHIDID_HUM1100") == 0)
		return (136);

	if (mos_strcasecmp(name, "PHIDID_INTERFACEKIT_4_8_8") == 0)
		return (1);

	if (mos_strcasecmp(name, "PHIDID_LCD1100") == 0)
		return (70);

	if (mos_strcasecmp(name, "PHIDID_LED1000") == 0)
		return (71);

	if (mos_strcasecmp(name, "PHIDID_LUX1000") == 0)
		return (72);

	if (mos_strcasecmp(name, "PHIDID_MOT0109") == 0)
		return (140);

	if (mos_strcasecmp(name, "PHIDID_MOT1100") == 0)
		return (73);

	if (mos_strcasecmp(name, "PHIDID_MOT1101") == 0)
		return (74);

	if (mos_strcasecmp(name, "PHIDID_MOT1102") == 0)
		return (137);

	if (mos_strcasecmp(name, "PHIDID_OUT1000") == 0)
		return (75);

	if (mos_strcasecmp(name, "PHIDID_OUT1001") == 0)
		return (76);

	if (mos_strcasecmp(name, "PHIDID_OUT1002") == 0)
		return (77);

	if (mos_strcasecmp(name, "PHIDID_OUT1100") == 0)
		return (78);

	if (mos_strcasecmp(name, "PHIDID_PRE1000") == 0)
		return (79);

	if (mos_strcasecmp(name, "PHIDID_RCC0004") == 0)
		return (124);

	if (mos_strcasecmp(name, "PHIDID_RCC1000") == 0)
		return (80);

	if (mos_strcasecmp(name, "PHIDID_REL1000") == 0)
		return (81);

	if (mos_strcasecmp(name, "PHIDID_REL1100") == 0)
		return (82);

	if (mos_strcasecmp(name, "PHIDID_REL1101") == 0)
		return (83);

	if (mos_strcasecmp(name, "PHIDID_SAF1000") == 0)
		return (84);

	if (mos_strcasecmp(name, "PHIDID_SND1000") == 0)
		return (85);

	if (mos_strcasecmp(name, "PHIDID_STC1000") == 0)
		return (86);

	if (mos_strcasecmp(name, "PHIDID_STC1001") == 0)
		return (115);

	if (mos_strcasecmp(name, "PHIDID_STC1002") == 0)
		return (118);

	if (mos_strcasecmp(name, "PHIDID_STC1003") == 0)
		return (119);

	if (mos_strcasecmp(name, "PHIDID_TMP1000") == 0)
		return (87);

	if (mos_strcasecmp(name, "PHIDID_TMP1100") == 0)
		return (88);

	if (mos_strcasecmp(name, "PHIDID_TMP1101") == 0)
		return (89);

	if (mos_strcasecmp(name, "PHIDID_TMP1200") == 0)
		return (90);

	if (mos_strcasecmp(name, "PHIDID_VCP1000") == 0)
		return (92);

	if (mos_strcasecmp(name, "PHIDID_VCP1001") == 0)
		return (93);

	if (mos_strcasecmp(name, "PHIDID_VCP1002") == 0)
		return (94);

	if (mos_strcasecmp(name, "PHIDID_VCP1100") == 0)
		return (105);

	if (mos_strcasecmp(name, "PHIDGET_LOG_CRITICAL") == 0)
		return (1);

	if (mos_strcasecmp(name, "PHIDGET_LOG_ERROR") == 0)
		return (2);

	if (mos_strcasecmp(name, "PHIDGET_LOG_WARNING") == 0)
		return (3);

	if (mos_strcasecmp(name, "PHIDGET_LOG_INFO") == 0)
		return (4);

	if (mos_strcasecmp(name, "PHIDGET_LOG_DEBUG") == 0)
		return (5);

	if (mos_strcasecmp(name, "PHIDGET_LOG_VERBOSE") == 0)
		return (6);

	if (mos_strcasecmp(name, "PHIDCLASS_NOTHING") == 0)
		return (0);

	if (mos_strcasecmp(name, "PHIDCLASS_ACCELEROMETER") == 0)
		return (1);

	if (mos_strcasecmp(name, "PHIDCLASS_ADVANCEDSERVO") == 0)
		return (2);

	if (mos_strcasecmp(name, "PHIDCLASS_ANALOG") == 0)
		return (3);

	if (mos_strcasecmp(name, "PHIDCLASS_BRIDGE") == 0)
		return (4);

	if (mos_strcasecmp(name, "PHIDCLASS_DATAADAPTER") == 0)
		return (25);

	if (mos_strcasecmp(name, "PHIDCLASS_DICTIONARY") == 0)
		return (24);

	if (mos_strcasecmp(name, "PHIDCLASS_ENCODER") == 0)
		return (5);

	if (mos_strcasecmp(name, "PHIDCLASS_FIRMWAREUPGRADE") == 0)
		return (23);

	if (mos_strcasecmp(name, "PHIDCLASS_FREQUENCYCOUNTER") == 0)
		return (6);

	if (mos_strcasecmp(name, "PHIDCLASS_GENERIC") == 0)
		return (22);

	if (mos_strcasecmp(name, "PHIDCLASS_GPS") == 0)
		return (7);

	if (mos_strcasecmp(name, "PHIDCLASS_HUB") == 0)
		return (8);

	if (mos_strcasecmp(name, "PHIDCLASS_INTERFACEKIT") == 0)
		return (9);

	if (mos_strcasecmp(name, "PHIDCLASS_IR") == 0)
		return (10);

	if (mos_strcasecmp(name, "PHIDCLASS_LED") == 0)
		return (11);

	if (mos_strcasecmp(name, "PHIDCLASS_MESHDONGLE") == 0)
		return (12);

	if (mos_strcasecmp(name, "PHIDCLASS_MOTORCONTROL") == 0)
		return (13);

	if (mos_strcasecmp(name, "PHIDCLASS_PHSENSOR") == 0)
		return (14);

	if (mos_strcasecmp(name, "PHIDCLASS_RFID") == 0)
		return (15);

	if (mos_strcasecmp(name, "PHIDCLASS_SERVO") == 0)
		return (16);

	if (mos_strcasecmp(name, "PHIDCLASS_SPATIAL") == 0)
		return (17);

	if (mos_strcasecmp(name, "PHIDCLASS_STEPPER") == 0)
		return (18);

	if (mos_strcasecmp(name, "PHIDCLASS_TEMPERATURESENSOR") == 0)
		return (19);

	if (mos_strcasecmp(name, "PHIDCLASS_TEXTLCD") == 0)
		return (20);

	if (mos_strcasecmp(name, "PHIDCLASS_VINT") == 0)
		return (21);

	if (mos_strcasecmp(name, "PHIDCHCLASS_NOTHING") == 0)
		return (0);

	if (mos_strcasecmp(name, "PHIDCHCLASS_ACCELEROMETER") == 0)
		return (1);

	if (mos_strcasecmp(name, "PHIDCHCLASS_BLDCMOTOR") == 0)
		return (35);

	if (mos_strcasecmp(name, "PHIDCHCLASS_CAPACITIVETOUCH") == 0)
		return (14);

	if (mos_strcasecmp(name, "PHIDCHCLASS_CURRENTINPUT") == 0)
		return (2);

	if (mos_strcasecmp(name, "PHIDCHCLASS_CURRENTOUTPUT") == 0)
		return (38);

	if (mos_strcasecmp(name, "PHIDCHCLASS_DATAADAPTER") == 0)
		return (3);

	if (mos_strcasecmp(name, "PHIDCHCLASS_DCMOTOR") == 0)
		return (4);

	if (mos_strcasecmp(name, "PHIDCHCLASS_DICTIONARY") == 0)
		return (36);

	if (mos_strcasecmp(name, "PHIDCHCLASS_DIGITALINPUT") == 0)
		return (5);

	if (mos_strcasecmp(name, "PHIDCHCLASS_DIGITALOUTPUT") == 0)
		return (6);

	if (mos_strcasecmp(name, "PHIDCHCLASS_DISTANCESENSOR") == 0)
		return (7);

	if (mos_strcasecmp(name, "PHIDCHCLASS_ENCODER") == 0)
		return (8);

	if (mos_strcasecmp(name, "PHIDCHCLASS_FIRMWAREUPGRADE") == 0)
		return (32);

	if (mos_strcasecmp(name, "PHIDCHCLASS_FREQUENCYCOUNTER") == 0)
		return (9);

	if (mos_strcasecmp(name, "PHIDCHCLASS_GENERIC") == 0)
		return (33);

	if (mos_strcasecmp(name, "PHIDCHCLASS_GPS") == 0)
		return (10);

	if (mos_strcasecmp(name, "PHIDCHCLASS_GYROSCOPE") == 0)
		return (12);

	if (mos_strcasecmp(name, "PHIDCHCLASS_HUB") == 0)
		return (13);

	if (mos_strcasecmp(name, "PHIDCHCLASS_HUMIDITYSENSOR") == 0)
		return (15);

	if (mos_strcasecmp(name, "PHIDCHCLASS_IR") == 0)
		return (16);

	if (mos_strcasecmp(name, "PHIDCHCLASS_LCD") == 0)
		return (11);

	if (mos_strcasecmp(name, "PHIDCHCLASS_LIGHTSENSOR") == 0)
		return (17);

	if (mos_strcasecmp(name, "PHIDCHCLASS_MAGNETOMETER") == 0)
		return (18);

	if (mos_strcasecmp(name, "PHIDCHCLASS_MESHDONGLE") == 0)
		return (19);

	if (mos_strcasecmp(name, "PHIDCHCLASS_MOTORPOSITIONCONTROLLER") == 0)
		return (34);

	if (mos_strcasecmp(name, "PHIDCHCLASS_PHSENSOR") == 0)
		return (37);

	if (mos_strcasecmp(name, "PHIDCHCLASS_POWERGUARD") == 0)
		return (20);

	if (mos_strcasecmp(name, "PHIDCHCLASS_PRESSURESENSOR") == 0)
		return (21);

	if (mos_strcasecmp(name, "PHIDCHCLASS_RCSERVO") == 0)
		return (22);

	if (mos_strcasecmp(name, "PHIDCHCLASS_RESISTANCEINPUT") == 0)
		return (23);

	if (mos_strcasecmp(name, "PHIDCHCLASS_RFID") == 0)
		return (24);

	if (mos_strcasecmp(name, "PHIDCHCLASS_SOUNDSENSOR") == 0)
		return (25);

	if (mos_strcasecmp(name, "PHIDCHCLASS_SPATIAL") == 0)
		return (26);

	if (mos_strcasecmp(name, "PHIDCHCLASS_STEPPER") == 0)
		return (27);

	if (mos_strcasecmp(name, "PHIDCHCLASS_TEMPERATURESENSOR") == 0)
		return (28);

	if (mos_strcasecmp(name, "PHIDCHCLASS_VOLTAGEINPUT") == 0)
		return (29);

	if (mos_strcasecmp(name, "PHIDCHCLASS_VOLTAGEOUTPUT") == 0)
		return (30);

	if (mos_strcasecmp(name, "PHIDCHCLASS_VOLTAGERATIOINPUT") == 0)
		return (31);

	if (mos_strcasecmp(name, "PHIDCHSUBCLASS_NONE") == 0)
		return (1);

	if (mos_strcasecmp(name, "PHIDCHSUBCLASS_DIGITALOUTPUT_DUTY_CYCLE") == 0)
		return (16);

	if (mos_strcasecmp(name, "PHIDCHSUBCLASS_DIGITALOUTPUT_FREQUENCY") == 0)
		return (18);

	if (mos_strcasecmp(name, "PHIDCHSUBCLASS_DIGITALOUTPUT_LED_DRIVER") == 0)
		return (17);

	if (mos_strcasecmp(name, "PHIDCHSUBCLASS_ENCODER_MODE_SETTABLE") == 0)
		return (96);

	if (mos_strcasecmp(name, "PHIDCHSUBCLASS_LCD_GRAPHIC") == 0)
		return (80);

	if (mos_strcasecmp(name, "PHIDCHSUBCLASS_LCD_TEXT") == 0)
		return (81);

	if (mos_strcasecmp(name, "PHIDCHSUBCLASS_SPATIAL_AHRS") == 0)
		return (112);

	if (mos_strcasecmp(name, "PHIDCHSUBCLASS_TEMPERATURESENSOR_RTD") == 0)
		return (32);

	if (mos_strcasecmp(name, "PHIDCHSUBCLASS_TEMPERATURESENSOR_THERMOCOUPLE") == 0)
		return (33);

	if (mos_strcasecmp(name, "PHIDCHSUBCLASS_VOLTAGEINPUT_SENSOR_PORT") == 0)
		return (48);

	if (mos_strcasecmp(name, "PHIDCHSUBCLASS_VOLTAGERATIOINPUT_BRIDGE") == 0)
		return (65);

	if (mos_strcasecmp(name, "PHIDCHSUBCLASS_VOLTAGERATIOINPUT_SENSOR_PORT") == 0)
		return (64);

	if (mos_strcasecmp(name, "MESHMODE_ROUTER") == 0)
		return (1);

	if (mos_strcasecmp(name, "MESHMODE_SLEEPYENDDEVICE") == 0)
		return (2);

	if (mos_strcasecmp(name, "POWER_SUPPLY_OFF") == 0)
		return (1);

	if (mos_strcasecmp(name, "POWER_SUPPLY_12V") == 0)
		return (2);

	if (mos_strcasecmp(name, "POWER_SUPPLY_24V") == 0)
		return (3);

	if (mos_strcasecmp(name, "RTD_WIRE_SETUP_2WIRE") == 0)
		return (1);

	if (mos_strcasecmp(name, "RTD_WIRE_SETUP_3WIRE") == 0)
		return (2);

	if (mos_strcasecmp(name, "RTD_WIRE_SETUP_4WIRE") == 0)
		return (3);

	if (mos_strcasecmp(name, "INPUT_MODE_NPN") == 0)
		return (1);

	if (mos_strcasecmp(name, "INPUT_MODE_PNP") == 0)
		return (2);

	if (mos_strcasecmp(name, "FAN_MODE_OFF") == 0)
		return (1);

	if (mos_strcasecmp(name, "FAN_MODE_ON") == 0)
		return (2);

	if (mos_strcasecmp(name, "FAN_MODE_AUTO") == 0)
		return (3);

	if (mos_strcasecmp(name, "SPATIAL_PRECISION_HYBRID") == 0)
		return (0);

	if (mos_strcasecmp(name, "SPATIAL_PRECISION_HIGH") == 0)
		return (1);

	if (mos_strcasecmp(name, "SPATIAL_PRECISION_LOW") == 0)
		return (2);

	if (mos_strcasecmp(name, "PHIDUNIT_NONE") == 0)
		return (0);

	if (mos_strcasecmp(name, "PHIDUNIT_WATT") == 0)
		return (17);

	if (mos_strcasecmp(name, "PHIDUNIT_PERCENT") == 0)
		return (2);

	if (mos_strcasecmp(name, "PHIDUNIT_DECIBEL") == 0)
		return (3);

	if (mos_strcasecmp(name, "PHIDUNIT_MILLIMETER") == 0)
		return (4);

	if (mos_strcasecmp(name, "PHIDUNIT_CENTIMETER") == 0)
		return (5);

	if (mos_strcasecmp(name, "PHIDUNIT_METER") == 0)
		return (6);

	if (mos_strcasecmp(name, "PHIDUNIT_GRAM") == 0)
		return (7);

	if (mos_strcasecmp(name, "PHIDUNIT_KILOGRAM") == 0)
		return (8);

	if (mos_strcasecmp(name, "PHIDUNIT_BOOLEAN") == 0)
		return (1);

	if (mos_strcasecmp(name, "PHIDUNIT_AMPERE") == 0)
		return (10);

	if (mos_strcasecmp(name, "PHIDUNIT_KILOPASCAL") == 0)
		return (11);

	if (mos_strcasecmp(name, "PHIDUNIT_VOLT") == 0)
		return (12);

	if (mos_strcasecmp(name, "PHIDUNIT_DEGREE_CELCIUS") == 0)
		return (13);

	if (mos_strcasecmp(name, "PHIDUNIT_LUX") == 0)
		return (14);

	if (mos_strcasecmp(name, "PHIDUNIT_GAUSS") == 0)
		return (15);

	if (mos_strcasecmp(name, "PHIDUNIT_PH") == 0)
		return (16);

	if (mos_strcasecmp(name, "PHIDUNIT_MILLIAMPERE") == 0)
		return (9);

	if (mos_strcasecmp(name, "PHIDGETSERVER_NONE") == 0)
		return (0);

	if (mos_strcasecmp(name, "PHIDGETSERVER_DEVICELISTENER") == 0)
		return (1);

	if (mos_strcasecmp(name, "PHIDGETSERVER_DEVICE") == 0)
		return (2);

	if (mos_strcasecmp(name, "PHIDGETSERVER_DEVICEREMOTE") == 0)
		return (3);

	if (mos_strcasecmp(name, "PHIDGETSERVER_WWWLISTENER") == 0)
		return (4);

	if (mos_strcasecmp(name, "PHIDGETSERVER_WWW") == 0)
		return (5);

	if (mos_strcasecmp(name, "PHIDGETSERVER_WWWREMOTE") == 0)
		return (6);

	if (mos_strcasecmp(name, "PHIDGETSERVER_SBC") == 0)
		return (7);

	if (mos_strcasecmp(name, "BRIDGE_GAIN_1") == 0)
		return (1);

	if (mos_strcasecmp(name, "BRIDGE_GAIN_2") == 0)
		return (2);

	if (mos_strcasecmp(name, "BRIDGE_GAIN_4") == 0)
		return (3);

	if (mos_strcasecmp(name, "BRIDGE_GAIN_8") == 0)
		return (4);

	if (mos_strcasecmp(name, "BRIDGE_GAIN_16") == 0)
		return (5);

	if (mos_strcasecmp(name, "BRIDGE_GAIN_32") == 0)
		return (6);

	if (mos_strcasecmp(name, "BRIDGE_GAIN_64") == 0)
		return (7);

	if (mos_strcasecmp(name, "BRIDGE_GAIN_128") == 0)
		return (8);

	if (mos_strcasecmp(name, "SENSOR_TYPE_VOLTAGERATIO") == 0)
		return (0);

	if (mos_strcasecmp(name, "SENSOR_TYPE_3522") == 0)
		return (35220);

	if (mos_strcasecmp(name, "SENSOR_TYPE_1101_SHARP_2Y0A21") == 0)
		return (11012);

	if (mos_strcasecmp(name, "SENSOR_TYPE_1101_SHARP_2Y0A02") == 0)
		return (11013);

	if (mos_strcasecmp(name, "SENSOR_TYPE_1102") == 0)
		return (11020);

	if (mos_strcasecmp(name, "SENSOR_TYPE_1103") == 0)
		return (11030);

	if (mos_strcasecmp(name, "SENSOR_TYPE_1104") == 0)
		return (11040);

	if (mos_strcasecmp(name, "SENSOR_TYPE_1105") == 0)
		return (11050);

	if (mos_strcasecmp(name, "SENSOR_TYPE_1106") == 0)
		return (11060);

	if (mos_strcasecmp(name, "SENSOR_TYPE_1107") == 0)
		return (11070);

	if (mos_strcasecmp(name, "SENSOR_TYPE_1108") == 0)
		return (11080);

	if (mos_strcasecmp(name, "SENSOR_TYPE_1109") == 0)
		return (11090);

	if (mos_strcasecmp(name, "SENSOR_TYPE_1110") == 0)
		return (11100);

	if (mos_strcasecmp(name, "SENSOR_TYPE_1111") == 0)
		return (11110);

	if (mos_strcasecmp(name, "SENSOR_TYPE_1112") == 0)
		return (11120);

	if (mos_strcasecmp(name, "SENSOR_TYPE_1113") == 0)
		return (11130);

	if (mos_strcasecmp(name, "SENSOR_TYPE_1115") == 0)
		return (11150);

	if (mos_strcasecmp(name, "SENSOR_TYPE_1116") == 0)
		return (11160);

	if (mos_strcasecmp(name, "SENSOR_TYPE_1118_AC") == 0)
		return (11181);

	if (mos_strcasecmp(name, "SENSOR_TYPE_1118_DC") == 0)
		return (11182);

	if (mos_strcasecmp(name, "SENSOR_TYPE_1119_AC") == 0)
		return (11191);

	if (mos_strcasecmp(name, "SENSOR_TYPE_1119_DC") == 0)
		return (11192);

	if (mos_strcasecmp(name, "SENSOR_TYPE_1120") == 0)
		return (11200);

	if (mos_strcasecmp(name, "SENSOR_TYPE_1121") == 0)
		return (11210);

	if (mos_strcasecmp(name, "SENSOR_TYPE_1101_SHARP_2D120X") == 0)
		return (11011);

	if (mos_strcasecmp(name, "SENSOR_TYPE_1122_DC") == 0)
		return (11222);

	if (mos_strcasecmp(name, "SENSOR_TYPE_1124") == 0)
		return (11240);

	if (mos_strcasecmp(name, "SENSOR_TYPE_1125_HUMIDITY") == 0)
		return (11251);

	if (mos_strcasecmp(name, "SENSOR_TYPE_1125_TEMPERATURE") == 0)
		return (11252);

	if (mos_strcasecmp(name, "SENSOR_TYPE_1126") == 0)
		return (11260);

	if (mos_strcasecmp(name, "SENSOR_TYPE_1128") == 0)
		return (11280);

	if (mos_strcasecmp(name, "SENSOR_TYPE_1129") == 0)
		return (11290);

	if (mos_strcasecmp(name, "SENSOR_TYPE_1131") == 0)
		return (11310);

	if (mos_strcasecmp(name, "SENSOR_TYPE_1134") == 0)
		return (11340);

	if (mos_strcasecmp(name, "SENSOR_TYPE_1136") == 0)
		return (11360);

	if (mos_strcasecmp(name, "SENSOR_TYPE_1137") == 0)
		return (11370);

	if (mos_strcasecmp(name, "SENSOR_TYPE_1138") == 0)
		return (11380);

	if (mos_strcasecmp(name, "SENSOR_TYPE_1139") == 0)
		return (11390);

	if (mos_strcasecmp(name, "SENSOR_TYPE_1140") == 0)
		return (11400);

	if (mos_strcasecmp(name, "SENSOR_TYPE_1141") == 0)
		return (11410);

	if (mos_strcasecmp(name, "SENSOR_TYPE_1146") == 0)
		return (11460);

	if (mos_strcasecmp(name, "SENSOR_TYPE_3120") == 0)
		return (31200);

	if (mos_strcasecmp(name, "SENSOR_TYPE_3121") == 0)
		return (31210);

	if (mos_strcasecmp(name, "SENSOR_TYPE_3122") == 0)
		return (31220);

	if (mos_strcasecmp(name, "SENSOR_TYPE_3123") == 0)
		return (31230);

	if (mos_strcasecmp(name, "SENSOR_TYPE_3130") == 0)
		return (31300);

	if (mos_strcasecmp(name, "SENSOR_TYPE_3520") == 0)
		return (35200);

	if (mos_strcasecmp(name, "SENSOR_TYPE_3521") == 0)
		return (35210);

	if (mos_strcasecmp(name, "SENSOR_TYPE_1122_AC") == 0)
		return (11221);

	if (mos_strcasecmp(name, "LED_FORWARD_VOLTAGE_1_7V") == 0)
		return (1);

	if (mos_strcasecmp(name, "LED_FORWARD_VOLTAGE_2_75V") == 0)
		return (2);

	if (mos_strcasecmp(name, "LED_FORWARD_VOLTAGE_3_2V") == 0)
		return (3);

	if (mos_strcasecmp(name, "LED_FORWARD_VOLTAGE_3_9V") == 0)
		return (4);

	if (mos_strcasecmp(name, "LED_FORWARD_VOLTAGE_4_0V") == 0)
		return (5);

	if (mos_strcasecmp(name, "LED_FORWARD_VOLTAGE_4_8V") == 0)
		return (6);

	if (mos_strcasecmp(name, "LED_FORWARD_VOLTAGE_5_0V") == 0)
		return (7);

	if (mos_strcasecmp(name, "LED_FORWARD_VOLTAGE_5_6V") == 0)
		return (8);

	if (mos_strcasecmp(name, "RCSERVO_VOLTAGE_5V") == 0)
		return (1);

	if (mos_strcasecmp(name, "RCSERVO_VOLTAGE_6V") == 0)
		return (2);

	if (mos_strcasecmp(name, "RCSERVO_VOLTAGE_7_4V") == 0)
		return (3);

	if (mos_strcasecmp(name, "VOLTAGE_OUTPUT_RANGE_10V") == 0)
		return (1);

	if (mos_strcasecmp(name, "VOLTAGE_OUTPUT_RANGE_5V") == 0)
		return (2);

	if (mos_strcasecmp(name, "VOLTAGE_RANGE_10mV") == 0)
		return (1);

	if (mos_strcasecmp(name, "VOLTAGE_RANGE_AUTO") == 0)
		return (11);

	if (mos_strcasecmp(name, "VOLTAGE_RANGE_200mV") == 0)
		return (3);

	if (mos_strcasecmp(name, "VOLTAGE_RANGE_312_5mV") == 0)
		return (4);

	if (mos_strcasecmp(name, "VOLTAGE_RANGE_400mV") == 0)
		return (5);

	if (mos_strcasecmp(name, "VOLTAGE_RANGE_40mV") == 0)
		return (2);

	if (mos_strcasecmp(name, "VOLTAGE_RANGE_2V") == 0)
		return (7);

	if (mos_strcasecmp(name, "VOLTAGE_RANGE_5V") == 0)
		return (8);

	if (mos_strcasecmp(name, "VOLTAGE_RANGE_15V") == 0)
		return (9);

	if (mos_strcasecmp(name, "VOLTAGE_RANGE_40V") == 0)
		return (10);

	if (mos_strcasecmp(name, "VOLTAGE_RANGE_1000mV") == 0)
		return (6);

	if (mos_strcasecmp(name, "SENSOR_TYPE_VOLTAGE") == 0)
		return (0);

	if (mos_strcasecmp(name, "SENSOR_TYPE_VCP4114") == 0)
		return (41140);

	if (mos_strcasecmp(name, "SENSOR_TYPE_1117") == 0)
		return (11170);

	if (mos_strcasecmp(name, "SENSOR_TYPE_1123") == 0)
		return (11230);

	if (mos_strcasecmp(name, "SENSOR_TYPE_1127") == 0)
		return (11270);

	if (mos_strcasecmp(name, "SENSOR_TYPE_1130_PH") == 0)
		return (11301);

	if (mos_strcasecmp(name, "SENSOR_TYPE_1130_ORP") == 0)
		return (11302);

	if (mos_strcasecmp(name, "SENSOR_TYPE_1132") == 0)
		return (11320);

	if (mos_strcasecmp(name, "SENSOR_TYPE_1133") == 0)
		return (11330);

	if (mos_strcasecmp(name, "SENSOR_TYPE_1135") == 0)
		return (11350);

	if (mos_strcasecmp(name, "SENSOR_TYPE_1142") == 0)
		return (11420);

	if (mos_strcasecmp(name, "SENSOR_TYPE_1143") == 0)
		return (11430);

	if (mos_strcasecmp(name, "SENSOR_TYPE_3500") == 0)
		return (35000);

	if (mos_strcasecmp(name, "SENSOR_TYPE_3501") == 0)
		return (35010);

	if (mos_strcasecmp(name, "SENSOR_TYPE_3502") == 0)
		return (35020);

	if (mos_strcasecmp(name, "SENSOR_TYPE_3503") == 0)
		return (35030);

	if (mos_strcasecmp(name, "SENSOR_TYPE_3507") == 0)
		return (35070);

	if (mos_strcasecmp(name, "SENSOR_TYPE_3508") == 0)
		return (35080);

	if (mos_strcasecmp(name, "SENSOR_TYPE_3509") == 0)
		return (35090);

	if (mos_strcasecmp(name, "SENSOR_TYPE_1114") == 0)
		return (11140);

	if (mos_strcasecmp(name, "SENSOR_TYPE_3511") == 0)
		return (35110);

	if (mos_strcasecmp(name, "SENSOR_TYPE_3512") == 0)
		return (35120);

	if (mos_strcasecmp(name, "SENSOR_TYPE_3513") == 0)
		return (35130);

	if (mos_strcasecmp(name, "SENSOR_TYPE_3514") == 0)
		return (35140);

	if (mos_strcasecmp(name, "SENSOR_TYPE_3515") == 0)
		return (35150);

	if (mos_strcasecmp(name, "SENSOR_TYPE_3516") == 0)
		return (35160);

	if (mos_strcasecmp(name, "SENSOR_TYPE_3517") == 0)
		return (35170);

	if (mos_strcasecmp(name, "SENSOR_TYPE_3518") == 0)
		return (35180);

	if (mos_strcasecmp(name, "SENSOR_TYPE_3519") == 0)
		return (35190);

	if (mos_strcasecmp(name, "SENSOR_TYPE_3584") == 0)
		return (35840);

	if (mos_strcasecmp(name, "SENSOR_TYPE_3585") == 0)
		return (35850);

	if (mos_strcasecmp(name, "SENSOR_TYPE_3586") == 0)
		return (35860);

	if (mos_strcasecmp(name, "SENSOR_TYPE_3587") == 0)
		return (35870);

	if (mos_strcasecmp(name, "SENSOR_TYPE_3588") == 0)
		return (35880);

	if (mos_strcasecmp(name, "SENSOR_TYPE_3589") == 0)
		return (35890);

	if (mos_strcasecmp(name, "SENSOR_TYPE_MOT2002_LOW") == 0)
		return (20020);

	if (mos_strcasecmp(name, "SENSOR_TYPE_MOT2002_MED") == 0)
		return (20021);

	if (mos_strcasecmp(name, "SENSOR_TYPE_MOT2002_HIGH") == 0)
		return (20022);

	if (mos_strcasecmp(name, "SENSOR_TYPE_3510") == 0)
		return (35100);

	if (mos_strcasecmp(name, "PROTOCOL_EM4100") == 0)
		return (1);

	if (mos_strcasecmp(name, "PROTOCOL_ISO11785_FDX_B") == 0)
		return (2);

	if (mos_strcasecmp(name, "PROTOCOL_PHIDGETS") == 0)
		return (3);

	if (mos_strcasecmp(name, "SPATIAL_ALGORITHM_NONE") == 0)
		return (0);

	if (mos_strcasecmp(name, "SPATIAL_ALGORITHM_AHRS") == 0)
		return (1);

	if (mos_strcasecmp(name, "SPATIAL_ALGORITHM_IMU") == 0)
		return (2);

	if (mos_strcasecmp(name, "RTD_TYPE_PT100_3850") == 0)
		return (1);

	if (mos_strcasecmp(name, "RTD_TYPE_PT1000_3850") == 0)
		return (2);

	if (mos_strcasecmp(name, "RTD_TYPE_PT100_3920") == 0)
		return (3);

	if (mos_strcasecmp(name, "RTD_TYPE_PT1000_3920") == 0)
		return (4);

	if (mos_strcasecmp(name, "THERMOCOUPLE_TYPE_J") == 0)
		return (1);

	if (mos_strcasecmp(name, "THERMOCOUPLE_TYPE_K") == 0)
		return (2);

	if (mos_strcasecmp(name, "THERMOCOUPLE_TYPE_E") == 0)
		return (3);

	if (mos_strcasecmp(name, "THERMOCOUPLE_TYPE_T") == 0)
		return (4);

	if (mos_strcasecmp(name, "FILTER_TYPE_ZERO_CROSSING") == 0)
		return (1);

	if (mos_strcasecmp(name, "FILTER_TYPE_LOGIC_LEVEL") == 0)
		return (2);

	if (mos_strcasecmp(name, "IR_ENCODING_UNKNOWN") == 0)
		return (1);

	if (mos_strcasecmp(name, "IR_ENCODING_SPACE") == 0)
		return (2);

	if (mos_strcasecmp(name, "IR_ENCODING_PULSE") == 0)
		return (3);

	if (mos_strcasecmp(name, "IR_ENCODING_BIPHASE") == 0)
		return (4);

	if (mos_strcasecmp(name, "IR_ENCODING_RC5") == 0)
		return (5);

	if (mos_strcasecmp(name, "IR_ENCODING_RC6") == 0)
		return (6);

	if (mos_strcasecmp(name, "IR_LENGTH_UNKNOWN") == 0)
		return (1);

	if (mos_strcasecmp(name, "IR_LENGTH_CONSTANT") == 0)
		return (2);

	if (mos_strcasecmp(name, "IR_LENGTH_VARIABLE") == 0)
		return (3);

	if (mos_strcasecmp(name, "CONTROL_MODE_STEP") == 0)
		return (0);

	if (mos_strcasecmp(name, "CONTROL_MODE_RUN") == 0)
		return (1);

	if (mos_strcasecmp(name, "FONT_User1") == 0)
		return (1);

	if (mos_strcasecmp(name, "FONT_User2") == 0)
		return (2);

	if (mos_strcasecmp(name, "FONT_6x10") == 0)
		return (3);

	if (mos_strcasecmp(name, "FONT_5x8") == 0)
		return (4);

	if (mos_strcasecmp(name, "FONT_6x12") == 0)
		return (5);

	if (mos_strcasecmp(name, "SCREEN_SIZE_NONE") == 0)
		return (1);

	if (mos_strcasecmp(name, "SCREEN_SIZE_64x128") == 0)
		return (13);

	if (mos_strcasecmp(name, "SCREEN_SIZE_2x8") == 0)
		return (3);

	if (mos_strcasecmp(name, "SCREEN_SIZE_1x16") == 0)
		return (4);

	if (mos_strcasecmp(name, "SCREEN_SIZE_2x16") == 0)
		return (5);

	if (mos_strcasecmp(name, "SCREEN_SIZE_4x16") == 0)
		return (6);

	if (mos_strcasecmp(name, "SCREEN_SIZE_1x8") == 0)
		return (2);

	if (mos_strcasecmp(name, "SCREEN_SIZE_4x20") == 0)
		return (8);

	if (mos_strcasecmp(name, "SCREEN_SIZE_2x24") == 0)
		return (9);

	if (mos_strcasecmp(name, "SCREEN_SIZE_1x40") == 0)
		return (10);

	if (mos_strcasecmp(name, "SCREEN_SIZE_2x40") == 0)
		return (11);

	if (mos_strcasecmp(name, "SCREEN_SIZE_4x40") == 0)
		return (12);

	if (mos_strcasecmp(name, "SCREEN_SIZE_2x20") == 0)
		return (7);

	if (mos_strcasecmp(name, "PIXEL_STATE_OFF") == 0)
		return (0);

	if (mos_strcasecmp(name, "PIXEL_STATE_ON") == 0)
		return (1);

	if (mos_strcasecmp(name, "PIXEL_STATE_INVERT") == 0)
		return (2);

	if (mos_strcasecmp(name, "PARITY_MODE_NONE") == 0)
		return (1);

	if (mos_strcasecmp(name, "PARITY_MODE_EVEN") == 0)
		return (2);

	if (mos_strcasecmp(name, "PARITY_MODE_ODD") == 0)
		return (3);

	if (mos_strcasecmp(name, "STOP_BITS_ONE") == 0)
		return (1);

	if (mos_strcasecmp(name, "STOP_BITS_TWO") == 0)
		return (2);

	if (mos_strcasecmp(name, "HANDSHAKE_MODE_NONE") == 0)
		return (1);

	if (mos_strcasecmp(name, "HANDSHAKE_MODE_REQUEST_TO_SEND") == 0)
		return (2);

	if (mos_strcasecmp(name, "HANDSHAKE_MODE_READY_TO_RECEIVE") == 0)
		return (3);

	if (mos_strcasecmp(name, "PROTOCOL_RS485") == 0)
		return (1);

	if (mos_strcasecmp(name, "PROTOCOL_RS422") == 0)
		return (2);

	if (mos_strcasecmp(name, "PROTOCOL_DMX512") == 0)
		return (3);

	if (mos_strcasecmp(name, "PROTOCOL_MODBUS_RTU") == 0)
		return (4);

	if (mos_strcasecmp(name, "PROTOCOL_SPI") == 0)
		return (5);

	if (mos_strcasecmp(name, "PROTOCOL_I2C") == 0)
		return (6);

	if (mos_strcasecmp(name, "PROTOCOL_UART") == 0)
		return (7);

	if (mos_strcasecmp(name, "PROTOCOL_RS232") == 0)
		return (8);

	if (mos_strcasecmp(name, "SPI_MODE_0") == 0)
		return (1);

	if (mos_strcasecmp(name, "SPI_MODE_1") == 0)
		return (2);

	if (mos_strcasecmp(name, "SPI_MODE_2") == 0)
		return (3);

	if (mos_strcasecmp(name, "SPI_MODE_3") == 0)
		return (4);

	if (mos_strcasecmp(name, "ENDIANNESS_MSB_FIRST") == 0)
		return (1);

	if (mos_strcasecmp(name, "ENDIANNESS_LSB_FIRST") == 0)
		return (2);

	if (mos_strcasecmp(name, "IO_VOLTAGE_EXTERN") == 0)
		return (1);

	if (mos_strcasecmp(name, "IO_VOLTAGE_1_8V") == 0)
		return (2);

	if (mos_strcasecmp(name, "IO_VOLTAGE_2_5V") == 0)
		return (3);

	if (mos_strcasecmp(name, "IO_VOLTAGE_3_3V") == 0)
		return (4);

	if (mos_strcasecmp(name, "IO_VOLTAGE_5_0V") == 0)
		return (5);

	if (mos_strcasecmp(name, "PACKET_ERROR_OK") == 0)
		return (0);

	if (mos_strcasecmp(name, "PACKET_ERROR_UNKNOWN") == 0)
		return (1);

	if (mos_strcasecmp(name, "PACKET_ERROR_TIMEOUT") == 0)
		return (2);

	if (mos_strcasecmp(name, "PACKET_ERROR_FORMAT") == 0)
		return (3);

	if (mos_strcasecmp(name, "PACKET_ERROR_INVALID") == 0)
		return (4);

	if (mos_strcasecmp(name, "PACKET_ERROR_OVERRUN") == 0)
		return (5);

	if (mos_strcasecmp(name, "PACKET_ERROR_CORRUPT") == 0)
		return (6);

	if (mos_strcasecmp(name, "SPL_RANGE_102dB") == 0)
		return (1);

	if (mos_strcasecmp(name, "SPL_RANGE_82dB") == 0)
		return (2);

	if (mos_strcasecmp(name, "PORT_MODE_VINT_PORT") == 0)
		return (0);

	if (mos_strcasecmp(name, "PORT_MODE_DIGITAL_INPUT") == 0)
		return (1);

	if (mos_strcasecmp(name, "PORT_MODE_DIGITAL_OUTPUT") == 0)
		return (2);

	if (mos_strcasecmp(name, "PORT_MODE_VOLTAGE_INPUT") == 0)
		return (3);

	if (mos_strcasecmp(name, "PORT_MODE_VOLTAGE_RATIO_INPUT") == 0)
		return (4);

	return (-1);
}

API_CRETURN
Phidget_enumString(const char *family, int id) {

	if (mos_strcasecmp(family, "EncoderIOMode") == 0) {
		switch (id) {
		case 1:
			return ("ENCODER_IO_MODE_PUSH_PULL");
		case 2:
			return ("ENCODER_IO_MODE_LINE_DRIVER_2K2");
		case 3:
			return ("ENCODER_IO_MODE_LINE_DRIVER_10K");
		case 4:
			return ("ENCODER_IO_MODE_OPEN_COLLECTOR_2K2");
		case 5:
			return ("ENCODER_IO_MODE_OPEN_COLLECTOR_10K");
		default:
			return ("");
		}
	}

	if (mos_strcasecmp(family, "PhidgetReturnCode") == 0) {
		switch (id) {
		case 0:
			return ("EPHIDGET_OK");
		case 61:
			return ("EPHIDGET_UNKNOWNVALLOW");
		case 2:
			return ("EPHIDGET_NOENT");
		case 3:
			return ("EPHIDGET_TIMEOUT");
		case 58:
			return ("EPHIDGET_KEEPALIVE");
		case 4:
			return ("EPHIDGET_INTERRUPTED");
		case 5:
			return ("EPHIDGET_IO");
		case 6:
			return ("EPHIDGET_NOMEMORY");
		case 7:
			return ("EPHIDGET_ACCESS");
		case 8:
			return ("EPHIDGET_FAULT");
		case 9:
			return ("EPHIDGET_BUSY");
		case 10:
			return ("EPHIDGET_EXIST");
		case 11:
			return ("EPHIDGET_NOTDIR");
		case 12:
			return ("EPHIDGET_ISDIR");
		case 13:
			return ("EPHIDGET_INVALID");
		case 14:
			return ("EPHIDGET_NFILE");
		case 15:
			return ("EPHIDGET_MFILE");
		case 16:
			return ("EPHIDGET_NOSPC");
		case 17:
			return ("EPHIDGET_FBIG");
		case 18:
			return ("EPHIDGET_ROFS");
		case 19:
			return ("EPHIDGET_RO");
		case 20:
			return ("EPHIDGET_UNSUPPORTED");
		case 21:
			return ("EPHIDGET_INVALIDARG");
		case 1:
			return ("EPHIDGET_PERM");
		case 26:
			return ("EPHIDGET_NOTEMPTY");
		case 28:
			return ("EPHIDGET_UNEXPECTED");
		case 27:
			return ("EPHIDGET_DUPLICATE");
		case 37:
			return ("EPHIDGET_BADPASSWORD");
		case 45:
			return ("EPHIDGET_NETUNAVAIL");
		case 35:
			return ("EPHIDGET_CONNREF");
		case 46:
			return ("EPHIDGET_CONNRESET");
		case 48:
			return ("EPHIDGET_HOSTUNREACH");
		case 40:
			return ("EPHIDGET_NODEV");
		case 50:
			return ("EPHIDGET_WRONGDEVICE");
		case 41:
			return ("EPHIDGET_PIPE");
		case 44:
			return ("EPHIDGET_RESOLV");
		case 51:
			return ("EPHIDGET_UNKNOWNVAL");
		case 52:
			return ("EPHIDGET_NOTATTACHED");
		case 53:
			return ("EPHIDGET_INVALIDPACKET");
		case 54:
			return ("EPHIDGET_2BIG");
		case 55:
			return ("EPHIDGET_BADVERSION");
		case 56:
			return ("EPHIDGET_CLOSED");
		case 57:
			return ("EPHIDGET_NOTCONFIGURED");
		case 31:
			return ("EPHIDGET_EOF");
		case 59:
			return ("EPHIDGET_FAILSAFE");
		case 60:
			return ("EPHIDGET_UNKNOWNVALHIGH");
		case 22:
			return ("EPHIDGET_AGAIN");
		default:
			return ("");
		}
	}

	if (mos_strcasecmp(family, "ErrorEventCode") == 0) {
		switch (id) {
		case 1:
			return ("EEPHIDGET_BADVERSION");
		case 4115:
			return ("EEPHIDGET_OUTOFRANGELOW");
		case 3:
			return ("EEPHIDGET_NETWORK");
		case 4:
			return ("EEPHIDGET_DISPATCH");
		case 5:
			return ("EEPHIDGET_FAILURE");
		case 4096:
			return ("EEPHIDGET_OK");
		case 4098:
			return ("EEPHIDGET_OVERRUN");
		case 4099:
			return ("EEPHIDGET_PACKETLOST");
		case 4100:
			return ("EEPHIDGET_WRAP");
		case 4101:
			return ("EEPHIDGET_OVERTEMP");
		case 4102:
			return ("EEPHIDGET_OVERCURRENT");
		case 2:
			return ("EEPHIDGET_BUSY");
		case 4104:
			return ("EEPHIDGET_BADPOWER");
		case 4105:
			return ("EEPHIDGET_SATURATION");
		case 4107:
			return ("EEPHIDGET_OVERVOLTAGE");
		case 4108:
			return ("EEPHIDGET_FAILSAFE");
		case 4109:
			return ("EEPHIDGET_VOLTAGEERROR");
		case 4110:
			return ("EEPHIDGET_ENERGYDUMP");
		case 4111:
			return ("EEPHIDGET_MOTORSTALL");
		case 4112:
			return ("EEPHIDGET_INVALIDSTATE");
		case 4113:
			return ("EEPHIDGET_BADCONNECTION");
		case 4114:
			return ("EEPHIDGET_OUTOFRANGEHIGH");
		case 4103:
			return ("EEPHIDGET_OUTOFRANGE");
		default:
			return ("");
		}
	}

	if (mos_strcasecmp(family, "DeviceID") == 0) {
		switch (id) {
		case 0:
			return ("PHIDID_NOTHING");
		case 125:
			return ("PHIDID_UNKNOWN");
		case 95:
			return ("PHIDID_DIGITALINPUT_PORT");
		case 96:
			return ("PHIDID_DIGITALOUTPUT_PORT");
		case 97:
			return ("PHIDID_VOLTAGEINPUT_PORT");
		case 98:
			return ("PHIDID_VOLTAGERATIOINPUT_PORT");
		case 111:
			return ("PHIDID_DICTIONARY");
		case 2:
			return ("PHIDID_1000");
		case 3:
			return ("PHIDID_1001");
		case 4:
			return ("PHIDID_1002");
		case 5:
			return ("PHIDID_1008");
		case 6:
			return ("PHIDID_1010_1013_1018_1019");
		case 7:
			return ("PHIDID_1011");
		case 8:
			return ("PHIDID_1012");
		case 9:
			return ("PHIDID_1014");
		case 10:
			return ("PHIDID_1015");
		case 11:
			return ("PHIDID_1016");
		case 12:
			return ("PHIDID_1017");
		case 13:
			return ("PHIDID_1023");
		case 14:
			return ("PHIDID_1024");
		case 15:
			return ("PHIDID_1030");
		case 16:
			return ("PHIDID_1031");
		case 17:
			return ("PHIDID_1032");
		case 18:
			return ("PHIDID_1040");
		case 19:
			return ("PHIDID_1041");
		case 20:
			return ("PHIDID_1042");
		case 21:
			return ("PHIDID_1043");
		case 22:
			return ("PHIDID_1044");
		case 23:
			return ("PHIDID_1045");
		case 24:
			return ("PHIDID_1046");
		case 25:
			return ("PHIDID_1047");
		case 26:
			return ("PHIDID_1048");
		case 27:
			return ("PHIDID_1049");
		case 28:
			return ("PHIDID_1051");
		case 29:
			return ("PHIDID_1052");
		case 30:
			return ("PHIDID_1053");
		case 31:
			return ("PHIDID_1054");
		case 32:
			return ("PHIDID_1055");
		case 33:
			return ("PHIDID_1056");
		case 34:
			return ("PHIDID_1057");
		case 35:
			return ("PHIDID_1058");
		case 36:
			return ("PHIDID_1059");
		case 37:
			return ("PHIDID_1060");
		case 38:
			return ("PHIDID_1061");
		case 39:
			return ("PHIDID_1062");
		case 40:
			return ("PHIDID_1063");
		case 41:
			return ("PHIDID_1064");
		case 42:
			return ("PHIDID_1065");
		case 43:
			return ("PHIDID_1066");
		case 44:
			return ("PHIDID_1067");
		case 45:
			return ("PHIDID_1202_1203");
		case 46:
			return ("PHIDID_1204");
		case 47:
			return ("PHIDID_1215__1218");
		case 48:
			return ("PHIDID_1219__1222");
		case 49:
			return ("PHIDID_ADP1000");
		case 51:
			return ("PHIDID_DAQ1000");
		case 52:
			return ("PHIDID_DAQ1200");
		case 53:
			return ("PHIDID_DAQ1300");
		case 54:
			return ("PHIDID_DAQ1301");
		case 55:
			return ("PHIDID_DAQ1400");
		case 56:
			return ("PHIDID_DAQ1500");
		case 57:
			return ("PHIDID_DCC1000");
		case 110:
			return ("PHIDID_DCC1001");
		case 117:
			return ("PHIDID_DCC1002");
		case 120:
			return ("PHIDID_DCC1003");
		case 108:
			return ("PHIDID_DCC1100");
		case 58:
			return ("PHIDID_DST1000");
		case 121:
			return ("PHIDID_DST1001");
		case 126:
			return ("PHIDID_DST1002");
		case 59:
			return ("PHIDID_DST1200");
		case 60:
			return ("PHIDID_ENC1000");
		case 104:
			return ("PHIDID_FIRMWARE_UPGRADE_SPI");
		case 102:
			return ("PHIDID_FIRMWARE_UPGRADE_STM32F0");
		case 143:
			return ("PHIDID_FIRMWARE_UPGRADE_STM32G0");
		case 103:
			return ("PHIDID_FIRMWARE_UPGRADE_STM8S");
		case 101:
			return ("PHIDID_FIRMWARE_UPGRADE_USB");
		case 61:
			return ("PHIDID_HIN1000");
		case 62:
			return ("PHIDID_HIN1001");
		case 63:
			return ("PHIDID_HIN1100");
		case 109:
			return ("PHIDID_HIN1101");
		case 64:
			return ("PHIDID_HUB0000");
		case 142:
			return ("PHIDID_HUB0001");
		case 67:
			return ("PHIDID_HUB0004");
		case 123:
			return ("PHIDID_HUB5000");
		case 69:
			return ("PHIDID_HUM1000");
		case 127:
			return ("PHIDID_HUM1001");
		case 136:
			return ("PHIDID_HUM1100");
		case 1:
			return ("PHIDID_INTERFACEKIT_4_8_8");
		case 70:
			return ("PHIDID_LCD1100");
		case 71:
			return ("PHIDID_LED1000");
		case 72:
			return ("PHIDID_LUX1000");
		case 140:
			return ("PHIDID_MOT0109");
		case 73:
			return ("PHIDID_MOT1100");
		case 74:
			return ("PHIDID_MOT1101");
		case 137:
			return ("PHIDID_MOT1102");
		case 75:
			return ("PHIDID_OUT1000");
		case 76:
			return ("PHIDID_OUT1001");
		case 77:
			return ("PHIDID_OUT1002");
		case 78:
			return ("PHIDID_OUT1100");
		case 79:
			return ("PHIDID_PRE1000");
		case 124:
			return ("PHIDID_RCC0004");
		case 80:
			return ("PHIDID_RCC1000");
		case 81:
			return ("PHIDID_REL1000");
		case 82:
			return ("PHIDID_REL1100");
		case 83:
			return ("PHIDID_REL1101");
		case 84:
			return ("PHIDID_SAF1000");
		case 85:
			return ("PHIDID_SND1000");
		case 86:
			return ("PHIDID_STC1000");
		case 115:
			return ("PHIDID_STC1001");
		case 118:
			return ("PHIDID_STC1002");
		case 119:
			return ("PHIDID_STC1003");
		case 87:
			return ("PHIDID_TMP1000");
		case 88:
			return ("PHIDID_TMP1100");
		case 89:
			return ("PHIDID_TMP1101");
		case 90:
			return ("PHIDID_TMP1200");
		case 92:
			return ("PHIDID_VCP1000");
		case 93:
			return ("PHIDID_VCP1001");
		case 94:
			return ("PHIDID_VCP1002");
		case 105:
			return ("PHIDID_VCP1100");
		default:
			return ("");
		}
	}

	if (mos_strcasecmp(family, "LogLevel") == 0) {
		switch (id) {
		case 1:
			return ("PHIDGET_LOG_CRITICAL");
		case 2:
			return ("PHIDGET_LOG_ERROR");
		case 3:
			return ("PHIDGET_LOG_WARNING");
		case 4:
			return ("PHIDGET_LOG_INFO");
		case 5:
			return ("PHIDGET_LOG_DEBUG");
		case 6:
			return ("PHIDGET_LOG_VERBOSE");
		default:
			return ("");
		}
	}

	if (mos_strcasecmp(family, "DeviceClass") == 0) {
		switch (id) {
		case 0:
			return ("PHIDCLASS_NOTHING");
		case 1:
			return ("PHIDCLASS_ACCELEROMETER");
		case 2:
			return ("PHIDCLASS_ADVANCEDSERVO");
		case 3:
			return ("PHIDCLASS_ANALOG");
		case 4:
			return ("PHIDCLASS_BRIDGE");
		case 25:
			return ("PHIDCLASS_DATAADAPTER");
		case 24:
			return ("PHIDCLASS_DICTIONARY");
		case 5:
			return ("PHIDCLASS_ENCODER");
		case 23:
			return ("PHIDCLASS_FIRMWAREUPGRADE");
		case 6:
			return ("PHIDCLASS_FREQUENCYCOUNTER");
		case 22:
			return ("PHIDCLASS_GENERIC");
		case 7:
			return ("PHIDCLASS_GPS");
		case 8:
			return ("PHIDCLASS_HUB");
		case 9:
			return ("PHIDCLASS_INTERFACEKIT");
		case 10:
			return ("PHIDCLASS_IR");
		case 11:
			return ("PHIDCLASS_LED");
		case 12:
			return ("PHIDCLASS_MESHDONGLE");
		case 13:
			return ("PHIDCLASS_MOTORCONTROL");
		case 14:
			return ("PHIDCLASS_PHSENSOR");
		case 15:
			return ("PHIDCLASS_RFID");
		case 16:
			return ("PHIDCLASS_SERVO");
		case 17:
			return ("PHIDCLASS_SPATIAL");
		case 18:
			return ("PHIDCLASS_STEPPER");
		case 19:
			return ("PHIDCLASS_TEMPERATURESENSOR");
		case 20:
			return ("PHIDCLASS_TEXTLCD");
		case 21:
			return ("PHIDCLASS_VINT");
		default:
			return ("");
		}
	}

	if (mos_strcasecmp(family, "ChannelClass") == 0) {
		switch (id) {
		case 0:
			return ("PHIDCHCLASS_NOTHING");
		case 1:
			return ("PHIDCHCLASS_ACCELEROMETER");
		case 35:
			return ("PHIDCHCLASS_BLDCMOTOR");
		case 14:
			return ("PHIDCHCLASS_CAPACITIVETOUCH");
		case 2:
			return ("PHIDCHCLASS_CURRENTINPUT");
		case 38:
			return ("PHIDCHCLASS_CURRENTOUTPUT");
		case 3:
			return ("PHIDCHCLASS_DATAADAPTER");
		case 4:
			return ("PHIDCHCLASS_DCMOTOR");
		case 36:
			return ("PHIDCHCLASS_DICTIONARY");
		case 5:
			return ("PHIDCHCLASS_DIGITALINPUT");
		case 6:
			return ("PHIDCHCLASS_DIGITALOUTPUT");
		case 7:
			return ("PHIDCHCLASS_DISTANCESENSOR");
		case 8:
			return ("PHIDCHCLASS_ENCODER");
		case 32:
			return ("PHIDCHCLASS_FIRMWAREUPGRADE");
		case 9:
			return ("PHIDCHCLASS_FREQUENCYCOUNTER");
		case 33:
			return ("PHIDCHCLASS_GENERIC");
		case 10:
			return ("PHIDCHCLASS_GPS");
		case 12:
			return ("PHIDCHCLASS_GYROSCOPE");
		case 13:
			return ("PHIDCHCLASS_HUB");
		case 15:
			return ("PHIDCHCLASS_HUMIDITYSENSOR");
		case 16:
			return ("PHIDCHCLASS_IR");
		case 11:
			return ("PHIDCHCLASS_LCD");
		case 17:
			return ("PHIDCHCLASS_LIGHTSENSOR");
		case 18:
			return ("PHIDCHCLASS_MAGNETOMETER");
		case 19:
			return ("PHIDCHCLASS_MESHDONGLE");
		case 34:
			return ("PHIDCHCLASS_MOTORPOSITIONCONTROLLER");
		case 37:
			return ("PHIDCHCLASS_PHSENSOR");
		case 20:
			return ("PHIDCHCLASS_POWERGUARD");
		case 21:
			return ("PHIDCHCLASS_PRESSURESENSOR");
		case 22:
			return ("PHIDCHCLASS_RCSERVO");
		case 23:
			return ("PHIDCHCLASS_RESISTANCEINPUT");
		case 24:
			return ("PHIDCHCLASS_RFID");
		case 25:
			return ("PHIDCHCLASS_SOUNDSENSOR");
		case 26:
			return ("PHIDCHCLASS_SPATIAL");
		case 27:
			return ("PHIDCHCLASS_STEPPER");
		case 28:
			return ("PHIDCHCLASS_TEMPERATURESENSOR");
		case 29:
			return ("PHIDCHCLASS_VOLTAGEINPUT");
		case 30:
			return ("PHIDCHCLASS_VOLTAGEOUTPUT");
		case 31:
			return ("PHIDCHCLASS_VOLTAGERATIOINPUT");
		default:
			return ("");
		}
	}

	if (mos_strcasecmp(family, "ChannelSubclass") == 0) {
		switch (id) {
		case 1:
			return ("PHIDCHSUBCLASS_NONE");
		case 16:
			return ("PHIDCHSUBCLASS_DIGITALOUTPUT_DUTY_CYCLE");
		case 18:
			return ("PHIDCHSUBCLASS_DIGITALOUTPUT_FREQUENCY");
		case 17:
			return ("PHIDCHSUBCLASS_DIGITALOUTPUT_LED_DRIVER");
		case 96:
			return ("PHIDCHSUBCLASS_ENCODER_MODE_SETTABLE");
		case 80:
			return ("PHIDCHSUBCLASS_LCD_GRAPHIC");
		case 81:
			return ("PHIDCHSUBCLASS_LCD_TEXT");
		case 112:
			return ("PHIDCHSUBCLASS_SPATIAL_AHRS");
		case 32:
			return ("PHIDCHSUBCLASS_TEMPERATURESENSOR_RTD");
		case 33:
			return ("PHIDCHSUBCLASS_TEMPERATURESENSOR_THERMOCOUPLE");
		case 48:
			return ("PHIDCHSUBCLASS_VOLTAGEINPUT_SENSOR_PORT");
		case 65:
			return ("PHIDCHSUBCLASS_VOLTAGERATIOINPUT_BRIDGE");
		case 64:
			return ("PHIDCHSUBCLASS_VOLTAGERATIOINPUT_SENSOR_PORT");
		default:
			return ("");
		}
	}

	if (mos_strcasecmp(family, "MeshMode") == 0) {
		switch (id) {
		case 1:
			return ("MESHMODE_ROUTER");
		case 2:
			return ("MESHMODE_SLEEPYENDDEVICE");
		default:
			return ("");
		}
	}

	if (mos_strcasecmp(family, "PowerSupply") == 0) {
		switch (id) {
		case 1:
			return ("POWER_SUPPLY_OFF");
		case 2:
			return ("POWER_SUPPLY_12V");
		case 3:
			return ("POWER_SUPPLY_24V");
		default:
			return ("");
		}
	}

	if (mos_strcasecmp(family, "RTDWireSetup") == 0) {
		switch (id) {
		case 1:
			return ("RTD_WIRE_SETUP_2WIRE");
		case 2:
			return ("RTD_WIRE_SETUP_3WIRE");
		case 3:
			return ("RTD_WIRE_SETUP_4WIRE");
		default:
			return ("");
		}
	}

	if (mos_strcasecmp(family, "InputMode") == 0) {
		switch (id) {
		case 1:
			return ("INPUT_MODE_NPN");
		case 2:
			return ("INPUT_MODE_PNP");
		default:
			return ("");
		}
	}

	if (mos_strcasecmp(family, "FanMode") == 0) {
		switch (id) {
		case 1:
			return ("FAN_MODE_OFF");
		case 2:
			return ("FAN_MODE_ON");
		case 3:
			return ("FAN_MODE_AUTO");
		default:
			return ("");
		}
	}

	if (mos_strcasecmp(family, "SpatialPrecision") == 0) {
		switch (id) {
		case 0:
			return ("SPATIAL_PRECISION_HYBRID");
		case 1:
			return ("SPATIAL_PRECISION_HIGH");
		case 2:
			return ("SPATIAL_PRECISION_LOW");
		default:
			return ("");
		}
	}

	if (mos_strcasecmp(family, "Unit") == 0) {
		switch (id) {
		case 0:
			return ("PHIDUNIT_NONE");
		case 17:
			return ("PHIDUNIT_WATT");
		case 2:
			return ("PHIDUNIT_PERCENT");
		case 3:
			return ("PHIDUNIT_DECIBEL");
		case 4:
			return ("PHIDUNIT_MILLIMETER");
		case 5:
			return ("PHIDUNIT_CENTIMETER");
		case 6:
			return ("PHIDUNIT_METER");
		case 7:
			return ("PHIDUNIT_GRAM");
		case 8:
			return ("PHIDUNIT_KILOGRAM");
		case 1:
			return ("PHIDUNIT_BOOLEAN");
		case 10:
			return ("PHIDUNIT_AMPERE");
		case 11:
			return ("PHIDUNIT_KILOPASCAL");
		case 12:
			return ("PHIDUNIT_VOLT");
		case 13:
			return ("PHIDUNIT_DEGREE_CELCIUS");
		case 14:
			return ("PHIDUNIT_LUX");
		case 15:
			return ("PHIDUNIT_GAUSS");
		case 16:
			return ("PHIDUNIT_PH");
		case 9:
			return ("PHIDUNIT_MILLIAMPERE");
		default:
			return ("");
		}
	}

	if (mos_strcasecmp(family, "PhidgetServerType") == 0) {
		switch (id) {
		case 0:
			return ("PHIDGETSERVER_NONE");
		case 1:
			return ("PHIDGETSERVER_DEVICELISTENER");
		case 2:
			return ("PHIDGETSERVER_DEVICE");
		case 3:
			return ("PHIDGETSERVER_DEVICEREMOTE");
		case 4:
			return ("PHIDGETSERVER_WWWLISTENER");
		case 5:
			return ("PHIDGETSERVER_WWW");
		case 6:
			return ("PHIDGETSERVER_WWWREMOTE");
		case 7:
			return ("PHIDGETSERVER_SBC");
		default:
			return ("");
		}
	}

	if (mos_strcasecmp(family, "BridgeGain") == 0) {
		switch (id) {
		case 1:
			return ("BRIDGE_GAIN_1");
		case 2:
			return ("BRIDGE_GAIN_2");
		case 3:
			return ("BRIDGE_GAIN_4");
		case 4:
			return ("BRIDGE_GAIN_8");
		case 5:
			return ("BRIDGE_GAIN_16");
		case 6:
			return ("BRIDGE_GAIN_32");
		case 7:
			return ("BRIDGE_GAIN_64");
		case 8:
			return ("BRIDGE_GAIN_128");
		default:
			return ("");
		}
	}

	if (mos_strcasecmp(family, "VoltageRatioSensorType") == 0) {
		switch (id) {
		case 0:
			return ("SENSOR_TYPE_VOLTAGERATIO");
		case 35220:
			return ("SENSOR_TYPE_3522");
		case 11012:
			return ("SENSOR_TYPE_1101_SHARP_2Y0A21");
		case 11013:
			return ("SENSOR_TYPE_1101_SHARP_2Y0A02");
		case 11020:
			return ("SENSOR_TYPE_1102");
		case 11030:
			return ("SENSOR_TYPE_1103");
		case 11040:
			return ("SENSOR_TYPE_1104");
		case 11050:
			return ("SENSOR_TYPE_1105");
		case 11060:
			return ("SENSOR_TYPE_1106");
		case 11070:
			return ("SENSOR_TYPE_1107");
		case 11080:
			return ("SENSOR_TYPE_1108");
		case 11090:
			return ("SENSOR_TYPE_1109");
		case 11100:
			return ("SENSOR_TYPE_1110");
		case 11110:
			return ("SENSOR_TYPE_1111");
		case 11120:
			return ("SENSOR_TYPE_1112");
		case 11130:
			return ("SENSOR_TYPE_1113");
		case 11150:
			return ("SENSOR_TYPE_1115");
		case 11160:
			return ("SENSOR_TYPE_1116");
		case 11181:
			return ("SENSOR_TYPE_1118_AC");
		case 11182:
			return ("SENSOR_TYPE_1118_DC");
		case 11191:
			return ("SENSOR_TYPE_1119_AC");
		case 11192:
			return ("SENSOR_TYPE_1119_DC");
		case 11200:
			return ("SENSOR_TYPE_1120");
		case 11210:
			return ("SENSOR_TYPE_1121");
		case 11011:
			return ("SENSOR_TYPE_1101_SHARP_2D120X");
		case 11222:
			return ("SENSOR_TYPE_1122_DC");
		case 11240:
			return ("SENSOR_TYPE_1124");
		case 11251:
			return ("SENSOR_TYPE_1125_HUMIDITY");
		case 11252:
			return ("SENSOR_TYPE_1125_TEMPERATURE");
		case 11260:
			return ("SENSOR_TYPE_1126");
		case 11280:
			return ("SENSOR_TYPE_1128");
		case 11290:
			return ("SENSOR_TYPE_1129");
		case 11310:
			return ("SENSOR_TYPE_1131");
		case 11340:
			return ("SENSOR_TYPE_1134");
		case 11360:
			return ("SENSOR_TYPE_1136");
		case 11370:
			return ("SENSOR_TYPE_1137");
		case 11380:
			return ("SENSOR_TYPE_1138");
		case 11390:
			return ("SENSOR_TYPE_1139");
		case 11400:
			return ("SENSOR_TYPE_1140");
		case 11410:
			return ("SENSOR_TYPE_1141");
		case 11460:
			return ("SENSOR_TYPE_1146");
		case 31200:
			return ("SENSOR_TYPE_3120");
		case 31210:
			return ("SENSOR_TYPE_3121");
		case 31220:
			return ("SENSOR_TYPE_3122");
		case 31230:
			return ("SENSOR_TYPE_3123");
		case 31300:
			return ("SENSOR_TYPE_3130");
		case 35200:
			return ("SENSOR_TYPE_3520");
		case 35210:
			return ("SENSOR_TYPE_3521");
		case 11221:
			return ("SENSOR_TYPE_1122_AC");
		default:
			return ("");
		}
	}

	if (mos_strcasecmp(family, "LEDForwardVoltage") == 0) {
		switch (id) {
		case 1:
			return ("LED_FORWARD_VOLTAGE_1_7V");
		case 2:
			return ("LED_FORWARD_VOLTAGE_2_75V");
		case 3:
			return ("LED_FORWARD_VOLTAGE_3_2V");
		case 4:
			return ("LED_FORWARD_VOLTAGE_3_9V");
		case 5:
			return ("LED_FORWARD_VOLTAGE_4_0V");
		case 6:
			return ("LED_FORWARD_VOLTAGE_4_8V");
		case 7:
			return ("LED_FORWARD_VOLTAGE_5_0V");
		case 8:
			return ("LED_FORWARD_VOLTAGE_5_6V");
		default:
			return ("");
		}
	}

	if (mos_strcasecmp(family, "RCServoVoltage") == 0) {
		switch (id) {
		case 1:
			return ("RCSERVO_VOLTAGE_5V");
		case 2:
			return ("RCSERVO_VOLTAGE_6V");
		case 3:
			return ("RCSERVO_VOLTAGE_7_4V");
		default:
			return ("");
		}
	}

	if (mos_strcasecmp(family, "VoltageOutputRange") == 0) {
		switch (id) {
		case 1:
			return ("VOLTAGE_OUTPUT_RANGE_10V");
		case 2:
			return ("VOLTAGE_OUTPUT_RANGE_5V");
		default:
			return ("");
		}
	}

	if (mos_strcasecmp(family, "VoltageRange") == 0) {
		switch (id) {
		case 1:
			return ("VOLTAGE_RANGE_10mV");
		case 11:
			return ("VOLTAGE_RANGE_AUTO");
		case 3:
			return ("VOLTAGE_RANGE_200mV");
		case 4:
			return ("VOLTAGE_RANGE_312_5mV");
		case 5:
			return ("VOLTAGE_RANGE_400mV");
		case 2:
			return ("VOLTAGE_RANGE_40mV");
		case 7:
			return ("VOLTAGE_RANGE_2V");
		case 8:
			return ("VOLTAGE_RANGE_5V");
		case 9:
			return ("VOLTAGE_RANGE_15V");
		case 10:
			return ("VOLTAGE_RANGE_40V");
		case 6:
			return ("VOLTAGE_RANGE_1000mV");
		default:
			return ("");
		}
	}

	if (mos_strcasecmp(family, "VoltageSensorType") == 0) {
		switch (id) {
		case 0:
			return ("SENSOR_TYPE_VOLTAGE");
		case 41140:
			return ("SENSOR_TYPE_VCP4114");
		case 11170:
			return ("SENSOR_TYPE_1117");
		case 11230:
			return ("SENSOR_TYPE_1123");
		case 11270:
			return ("SENSOR_TYPE_1127");
		case 11301:
			return ("SENSOR_TYPE_1130_PH");
		case 11302:
			return ("SENSOR_TYPE_1130_ORP");
		case 11320:
			return ("SENSOR_TYPE_1132");
		case 11330:
			return ("SENSOR_TYPE_1133");
		case 11350:
			return ("SENSOR_TYPE_1135");
		case 11420:
			return ("SENSOR_TYPE_1142");
		case 11430:
			return ("SENSOR_TYPE_1143");
		case 35000:
			return ("SENSOR_TYPE_3500");
		case 35010:
			return ("SENSOR_TYPE_3501");
		case 35020:
			return ("SENSOR_TYPE_3502");
		case 35030:
			return ("SENSOR_TYPE_3503");
		case 35070:
			return ("SENSOR_TYPE_3507");
		case 35080:
			return ("SENSOR_TYPE_3508");
		case 35090:
			return ("SENSOR_TYPE_3509");
		case 11140:
			return ("SENSOR_TYPE_1114");
		case 35110:
			return ("SENSOR_TYPE_3511");
		case 35120:
			return ("SENSOR_TYPE_3512");
		case 35130:
			return ("SENSOR_TYPE_3513");
		case 35140:
			return ("SENSOR_TYPE_3514");
		case 35150:
			return ("SENSOR_TYPE_3515");
		case 35160:
			return ("SENSOR_TYPE_3516");
		case 35170:
			return ("SENSOR_TYPE_3517");
		case 35180:
			return ("SENSOR_TYPE_3518");
		case 35190:
			return ("SENSOR_TYPE_3519");
		case 35840:
			return ("SENSOR_TYPE_3584");
		case 35850:
			return ("SENSOR_TYPE_3585");
		case 35860:
			return ("SENSOR_TYPE_3586");
		case 35870:
			return ("SENSOR_TYPE_3587");
		case 35880:
			return ("SENSOR_TYPE_3588");
		case 35890:
			return ("SENSOR_TYPE_3589");
		case 20020:
			return ("SENSOR_TYPE_MOT2002_LOW");
		case 20021:
			return ("SENSOR_TYPE_MOT2002_MED");
		case 20022:
			return ("SENSOR_TYPE_MOT2002_HIGH");
		case 35100:
			return ("SENSOR_TYPE_3510");
		default:
			return ("");
		}
	}

	if (mos_strcasecmp(family, "RFIDProtocol") == 0) {
		switch (id) {
		case 1:
			return ("PROTOCOL_EM4100");
		case 2:
			return ("PROTOCOL_ISO11785_FDX_B");
		case 3:
			return ("PROTOCOL_PHIDGETS");
		default:
			return ("");
		}
	}

	if (mos_strcasecmp(family, "SpatialAlgorithm") == 0) {
		switch (id) {
		case 0:
			return ("SPATIAL_ALGORITHM_NONE");
		case 1:
			return ("SPATIAL_ALGORITHM_AHRS");
		case 2:
			return ("SPATIAL_ALGORITHM_IMU");
		default:
			return ("");
		}
	}

	if (mos_strcasecmp(family, "RTDType") == 0) {
		switch (id) {
		case 1:
			return ("RTD_TYPE_PT100_3850");
		case 2:
			return ("RTD_TYPE_PT1000_3850");
		case 3:
			return ("RTD_TYPE_PT100_3920");
		case 4:
			return ("RTD_TYPE_PT1000_3920");
		default:
			return ("");
		}
	}

	if (mos_strcasecmp(family, "ThermocoupleType") == 0) {
		switch (id) {
		case 1:
			return ("THERMOCOUPLE_TYPE_J");
		case 2:
			return ("THERMOCOUPLE_TYPE_K");
		case 3:
			return ("THERMOCOUPLE_TYPE_E");
		case 4:
			return ("THERMOCOUPLE_TYPE_T");
		default:
			return ("");
		}
	}

	if (mos_strcasecmp(family, "FilterType") == 0) {
		switch (id) {
		case 1:
			return ("FILTER_TYPE_ZERO_CROSSING");
		case 2:
			return ("FILTER_TYPE_LOGIC_LEVEL");
		default:
			return ("");
		}
	}

	if (mos_strcasecmp(family, "Encoding") == 0) {
		switch (id) {
		case 1:
			return ("IR_ENCODING_UNKNOWN");
		case 2:
			return ("IR_ENCODING_SPACE");
		case 3:
			return ("IR_ENCODING_PULSE");
		case 4:
			return ("IR_ENCODING_BIPHASE");
		case 5:
			return ("IR_ENCODING_RC5");
		case 6:
			return ("IR_ENCODING_RC6");
		default:
			return ("");
		}
	}

	if (mos_strcasecmp(family, "Length") == 0) {
		switch (id) {
		case 1:
			return ("IR_LENGTH_UNKNOWN");
		case 2:
			return ("IR_LENGTH_CONSTANT");
		case 3:
			return ("IR_LENGTH_VARIABLE");
		default:
			return ("");
		}
	}

	if (mos_strcasecmp(family, "ControlMode") == 0) {
		switch (id) {
		case 0:
			return ("CONTROL_MODE_STEP");
		case 1:
			return ("CONTROL_MODE_RUN");
		default:
			return ("");
		}
	}

	if (mos_strcasecmp(family, "LCDFont") == 0) {
		switch (id) {
		case 1:
			return ("FONT_User1");
		case 2:
			return ("FONT_User2");
		case 3:
			return ("FONT_6x10");
		case 4:
			return ("FONT_5x8");
		case 5:
			return ("FONT_6x12");
		default:
			return ("");
		}
	}

	if (mos_strcasecmp(family, "LCDScreenSize") == 0) {
		switch (id) {
		case 1:
			return ("SCREEN_SIZE_NONE");
		case 13:
			return ("SCREEN_SIZE_64x128");
		case 3:
			return ("SCREEN_SIZE_2x8");
		case 4:
			return ("SCREEN_SIZE_1x16");
		case 5:
			return ("SCREEN_SIZE_2x16");
		case 6:
			return ("SCREEN_SIZE_4x16");
		case 2:
			return ("SCREEN_SIZE_1x8");
		case 8:
			return ("SCREEN_SIZE_4x20");
		case 9:
			return ("SCREEN_SIZE_2x24");
		case 10:
			return ("SCREEN_SIZE_1x40");
		case 11:
			return ("SCREEN_SIZE_2x40");
		case 12:
			return ("SCREEN_SIZE_4x40");
		case 7:
			return ("SCREEN_SIZE_2x20");
		default:
			return ("");
		}
	}

	if (mos_strcasecmp(family, "LCDPixelState") == 0) {
		switch (id) {
		case 0:
			return ("PIXEL_STATE_OFF");
		case 1:
			return ("PIXEL_STATE_ON");
		case 2:
			return ("PIXEL_STATE_INVERT");
		default:
			return ("");
		}
	}

	if (mos_strcasecmp(family, "DataAdapterParity") == 0) {
		switch (id) {
		case 1:
			return ("PARITY_MODE_NONE");
		case 2:
			return ("PARITY_MODE_EVEN");
		case 3:
			return ("PARITY_MODE_ODD");
		default:
			return ("");
		}
	}

	if (mos_strcasecmp(family, "DataAdapterStopBits") == 0) {
		switch (id) {
		case 1:
			return ("STOP_BITS_ONE");
		case 2:
			return ("STOP_BITS_TWO");
		default:
			return ("");
		}
	}

	if (mos_strcasecmp(family, "DataAdapterHandshakeMode") == 0) {
		switch (id) {
		case 1:
			return ("HANDSHAKE_MODE_NONE");
		case 2:
			return ("HANDSHAKE_MODE_REQUEST_TO_SEND");
		case 3:
			return ("HANDSHAKE_MODE_READY_TO_RECEIVE");
		default:
			return ("");
		}
	}

	if (mos_strcasecmp(family, "DataAdapterProtocol") == 0) {
		switch (id) {
		case 1:
			return ("PROTOCOL_RS485");
		case 2:
			return ("PROTOCOL_RS422");
		case 3:
			return ("PROTOCOL_DMX512");
		case 4:
			return ("PROTOCOL_MODBUS_RTU");
		case 5:
			return ("PROTOCOL_SPI");
		case 6:
			return ("PROTOCOL_I2C");
		case 7:
			return ("PROTOCOL_UART");
		case 8:
			return ("PROTOCOL_RS232");
		default:
			return ("");
		}
	}

	if (mos_strcasecmp(family, "DataAdapterSPIMode") == 0) {
		switch (id) {
		case 1:
			return ("SPI_MODE_0");
		case 2:
			return ("SPI_MODE_1");
		case 3:
			return ("SPI_MODE_2");
		case 4:
			return ("SPI_MODE_3");
		default:
			return ("");
		}
	}

	if (mos_strcasecmp(family, "DataAdapterEndianness") == 0) {
		switch (id) {
		case 1:
			return ("ENDIANNESS_MSB_FIRST");
		case 2:
			return ("ENDIANNESS_LSB_FIRST");
		default:
			return ("");
		}
	}

	if (mos_strcasecmp(family, "DataAdapterIOVoltage") == 0) {
		switch (id) {
		case 1:
			return ("IO_VOLTAGE_EXTERN");
		case 2:
			return ("IO_VOLTAGE_1_8V");
		case 3:
			return ("IO_VOLTAGE_2_5V");
		case 4:
			return ("IO_VOLTAGE_3_3V");
		case 5:
			return ("IO_VOLTAGE_5_0V");
		default:
			return ("");
		}
	}

	if (mos_strcasecmp(family, "PacketErrorCode") == 0) {
		switch (id) {
		case 0:
			return ("PACKET_ERROR_OK");
		case 1:
			return ("PACKET_ERROR_UNKNOWN");
		case 2:
			return ("PACKET_ERROR_TIMEOUT");
		case 3:
			return ("PACKET_ERROR_FORMAT");
		case 4:
			return ("PACKET_ERROR_INVALID");
		case 5:
			return ("PACKET_ERROR_OVERRUN");
		case 6:
			return ("PACKET_ERROR_CORRUPT");
		default:
			return ("");
		}
	}

	if (mos_strcasecmp(family, "SPLRange") == 0) {
		switch (id) {
		case 1:
			return ("SPL_RANGE_102dB");
		case 2:
			return ("SPL_RANGE_82dB");
		default:
			return ("");
		}
	}

	if (mos_strcasecmp(family, "PortMode") == 0) {
		switch (id) {
		case 0:
			return ("PORT_MODE_VINT_PORT");
		case 1:
			return ("PORT_MODE_DIGITAL_INPUT");
		case 2:
			return ("PORT_MODE_DIGITAL_OUTPUT");
		case 3:
			return ("PORT_MODE_VOLTAGE_INPUT");
		case 4:
			return ("PORT_MODE_VOLTAGE_RATIO_INPUT");
		default:
			return ("");
		}
	}

	return ("");
}

int
supportedEncoderIOMode(PhidgetChannelHandle ch, Phidget_EncoderIOMode val) {

	switch (ch->UCD->uid) {
	case PHIDCHUID_1057_ENCODER_400:
		switch (val) {
		case ENCODER_IO_MODE_PUSH_PULL:
		case ENCODER_IO_MODE_LINE_DRIVER_2K2:
		case ENCODER_IO_MODE_LINE_DRIVER_10K:
		case ENCODER_IO_MODE_OPEN_COLLECTOR_2K2:
		case ENCODER_IO_MODE_OPEN_COLLECTOR_10K:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1000_ENCODER_100:
		switch (val) {
		case ENCODER_IO_MODE_PUSH_PULL:
		case ENCODER_IO_MODE_LINE_DRIVER_2K2:
		case ENCODER_IO_MODE_LINE_DRIVER_10K:
		case ENCODER_IO_MODE_OPEN_COLLECTOR_2K2:
		case ENCODER_IO_MODE_OPEN_COLLECTOR_10K:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1000_ENCODER_200:
		switch (val) {
		case ENCODER_IO_MODE_PUSH_PULL:
		case ENCODER_IO_MODE_LINE_DRIVER_2K2:
		case ENCODER_IO_MODE_LINE_DRIVER_10K:
		case ENCODER_IO_MODE_OPEN_COLLECTOR_2K2:
		case ENCODER_IO_MODE_OPEN_COLLECTOR_10K:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1000_MOTORPOSITIONCONTROLLER_200:
		switch (val) {
		case ENCODER_IO_MODE_PUSH_PULL:
		case ENCODER_IO_MODE_LINE_DRIVER_2K2:
		case ENCODER_IO_MODE_LINE_DRIVER_10K:
		case ENCODER_IO_MODE_OPEN_COLLECTOR_2K2:
		case ENCODER_IO_MODE_OPEN_COLLECTOR_10K:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1000_ENCODER_210:
		switch (val) {
		case ENCODER_IO_MODE_PUSH_PULL:
		case ENCODER_IO_MODE_LINE_DRIVER_2K2:
		case ENCODER_IO_MODE_LINE_DRIVER_10K:
		case ENCODER_IO_MODE_OPEN_COLLECTOR_2K2:
		case ENCODER_IO_MODE_OPEN_COLLECTOR_10K:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1000_MOTORPOSITIONCONTROLLER_210:
		switch (val) {
		case ENCODER_IO_MODE_PUSH_PULL:
		case ENCODER_IO_MODE_LINE_DRIVER_2K2:
		case ENCODER_IO_MODE_LINE_DRIVER_10K:
		case ENCODER_IO_MODE_OPEN_COLLECTOR_2K2:
		case ENCODER_IO_MODE_OPEN_COLLECTOR_10K:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_ENC1000_ENCODER_100:
		switch (val) {
		case ENCODER_IO_MODE_PUSH_PULL:
		case ENCODER_IO_MODE_LINE_DRIVER_2K2:
		case ENCODER_IO_MODE_LINE_DRIVER_10K:
		case ENCODER_IO_MODE_OPEN_COLLECTOR_2K2:
		case ENCODER_IO_MODE_OPEN_COLLECTOR_10K:
			return (1);
		default:
			return (0);
		}
	default:
		return (0);
	}
}

int
supportedDeviceID(PhidgetChannelHandle ch, Phidget_DeviceID val) {

	switch (ch->UCD->uid) {
	case PHIDCHUID_ifkit488_VOLTAGERATIOINPUT_000:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_INTERFACEKIT_4_8_8:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_ifkit488_DIGITALINPUT_000:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_INTERFACEKIT_4_8_8:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_ifkit488_DIGITALOUTPUT_000:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_INTERFACEKIT_4_8_8:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1000_RCSERVO_OLD1_200:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_1000:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1000_RCSERVO_OLD2_200:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_1000:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1000_RCSERVO_300:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_1000:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1000_RCSERVO_313:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_1000:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1001_RCSERVO_OLD1_200:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_1001:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1001_RCSERVO_OLD2_200:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_1001:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1001_RCSERVO_313:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_1001:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1001_RCSERVO_400:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_1001:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1002_VOLTAGEOUTPUT_100:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_1002:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1008_ACCELEROMETER_000:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_1008:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1011_VOLTAGEINPUT_000:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_1011:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1011_VOLTAGERATIOINPUT_000:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_1011:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1011_DIGITALINPUT_000:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_1011:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1011_DIGITALOUTPUT_000:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_1011:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1012_DIGITALINPUT_000:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_1012:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1012_DIGITALOUTPUT_000:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_1012:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1012_DIGITALINPUT_601:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_1012:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1012_DIGITALOUTPUT_601:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_1012:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1012_DIGITALINPUT_602:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_1012:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1012_DIGITALOUTPUT_602:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_1012:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1013_VOLTAGEINPUT_000:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_1010_1013_1018_1019:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1013_VOLTAGERATIOINPUT_000:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_1010_1013_1018_1019:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1013_DIGITALINPUT_000:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_1010_1013_1018_1019:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1013_DIGITALOUTPUT_000:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_1010_1013_1018_1019:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1018_VOLTAGEINPUT_821:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_1010_1013_1018_1019:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1018_VOLTAGERATIOINPUT_821:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_1010_1013_1018_1019:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1018_DIGITALINPUT_821:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_1010_1013_1018_1019:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1018_DIGITALOUTPUT_821:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_1010_1013_1018_1019:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1014_DIGITALOUTPUT_000:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_1014:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1014_DIGITALOUTPUT_704:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_1014:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1015_CAPACITIVETOUCH_000:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_1015:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1016_CAPACITIVETOUCH_000:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_1016:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1017_DIGITALOUTPUT_000:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_1017:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1018_VOLTAGEINPUT_900:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_1010_1013_1018_1019:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1018_VOLTAGERATIOINPUT_900:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_1010_1013_1018_1019:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1018_DIGITALINPUT_900:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_1010_1013_1018_1019:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1018_DIGITALOUTPUT_900:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_1010_1013_1018_1019:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1018_VOLTAGEINPUT_1000:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_1010_1013_1018_1019:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1018_VOLTAGERATIOINPUT_1000:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_1010_1013_1018_1019:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1018_DIGITALINPUT_1000:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_1010_1013_1018_1019:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1018_DIGITALOUTPUT_1000:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_1010_1013_1018_1019:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1023_RFID_000:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_1023:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1023_RFID_104:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_1023:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1023_RFID_200:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_1023:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1023_DIGITALOUTPUT_5V_200:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_1023:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1023_DIGITALOUTPUT_LED_200:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_1023:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1023_DIGITALOUTPUT_ONBOARD_LED_200:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_1023:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1023_RFID_201:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_1023:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1023_DIGITALOUTPUT_5V_201:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_1023:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1023_DIGITALOUTPUT_LED_201:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_1023:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1023_DIGITALOUTPUT_ONBOARD_LED_201:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_1023:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1024_RFID_100:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_1024:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1024_DIGITALOUTPUT_5V_100:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_1024:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1024_DIGITALOUTPUT_LED_100:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_1024:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1024_DIGITALOUTPUT_ONBOARD_LED_100:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_1024:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1030_DIGITALOUTPUT_100:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_1030:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1031_DIGITALOUTPUT_100:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_1031:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1032_DIGITALOUTPUT_200:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_1032:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1040_GPS_000:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_1040:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1041_ACCELEROMETER_200:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_1041:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1042_ACCELEROMETER_300:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_1042:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1042_GYROSCOPE_300:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_1042:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1042_MAGNETOMETER_300:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_1042:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1042_SPATIAL_300:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_1042:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1043_ACCELEROMETER_300:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_1043:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1044_ACCELEROMETER_400:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_1044:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1044_GYROSCOPE_400:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_1044:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1044_MAGNETOMETER_400:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_1044:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1044_SPATIAL_400:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_1044:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1044_ACCELEROMETER_500:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_1044:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1044_GYROSCOPE_500:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_1044:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1044_MAGNETOMETER_500:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_1044:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1044_SPATIAL_500:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_1044:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1044_ACCELEROMETER_510:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_1044:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1044_GYROSCOPE_510:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_1044:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1044_MAGNETOMETER_510:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_1044:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1044_SPATIAL_510:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_1044:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1045_TEMPERATURESENSOR_IR_100:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_1045:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1045_TEMPERATURESENSOR_IC_100:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_1045:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1046_VOLTAGERATIOINPUT_100:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_1046:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1046_VOLTAGERATIOINPUT_102:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_1046:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1047_ENCODER_100:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_1047:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1047_DIGITALINPUT_100:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_1047:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1047_ENCODER_200:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_1047:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1047_DIGITALINPUT_200:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_1047:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1048_TEMPERATURESENSOR_THERMOCOUPLE_100:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_1048:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1048_TEMPERATURESENSOR_IC_100:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_1048:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1048_VOLTAGEINPUT_100:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_1048:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1048_TEMPERATURESENSOR_THERMOCOUPLE_200:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_1048:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1048_TEMPERATURESENSOR_IC_200:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_1048:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1048_VOLTAGEINPUT_200:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_1048:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1049_ACCELEROMETER_000:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_1049:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1051_TEMPERATURESENSOR_THERMOCOUPLE_000:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_1051:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1051_TEMPERATURESENSOR_IC_000:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_1051:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1051_TEMPERATURESENSOR_THERMOCOUPLE_200:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_1051:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1051_TEMPERATURESENSOR_IC_200:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_1051:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1051_VOLTAGEINPUT_200:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_1051:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1051_TEMPERATURESENSOR_THERMOCOUPLE_300:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_1051:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1051_TEMPERATURESENSOR_IC_300:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_1051:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1051_VOLTAGEINPUT_300:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_1051:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1051_TEMPERATURESENSOR_THERMOCOUPLE_400:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_1051:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1051_TEMPERATURESENSOR_IC_400:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_1051:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1051_VOLTAGEINPUT_400:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_1051:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1052_ENCODER_000:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_1052:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1052_DIGITALINPUT_000:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_1052:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1052_ENCODER_101:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_1052:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1052_DIGITALINPUT_101:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_1052:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1052_ENCODER_110:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_1052:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1052_DIGITALINPUT_110:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_1052:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1053_ACCELEROMETER_300:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_1053:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1054_FREQUENCYCOUNTER_000:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_1054:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1055_IR_100:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_1055:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1055_IR_200_USB:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_1055:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1055_IR_200_VINT:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_1055:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1056_ACCELEROMETER_000:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_1056:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1056_GYROSCOPE_000:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_1056:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1056_MAGNETOMETER_000:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_1056:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1056_SPATIAL_000:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_1056:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1056_ACCELEROMETER_200:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_1056:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1056_GYROSCOPE_200:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_1056:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1056_MAGNETOMETER_200:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_1056:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1056_SPATIAL_200:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_1056:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1057_ENCODER_300:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_1057:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1057_ENCODER_400:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_1057:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1058_VOLTAGEINPUT_100:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_1058:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1058_PHADAPTER_100:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_1058:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1059_ACCELEROMETER_400:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_1059:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1060_DCMOTOR_100:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_1060:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1060_DIGITALINPUT_100:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_1060:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1061_RCSERVO_100:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_1061:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1061_CURRENTINPUT_100:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_1061:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1061_RCSERVO_200:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_1061:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1061_CURRENTINPUT_200:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_1061:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1061_RCSERVO_300:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_1061:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1061_CURRENTINPUT_300:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_1061:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_RCC0004_RCSERVO_400:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_RCC0004:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1062_STEPPER_100:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_1062:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1063_STEPPER_100:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_1063:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1063_DIGITALINPUT_100:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_1063:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1063_CURRENTINPUT_100:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_1063:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1064_DCMOTOR_100:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_1064:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1064_CURRENTINPUT_100:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_1064:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1065_DCMOTOR_100:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_1065:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1065_DIGITALINPUT_100:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_1065:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1065_ENCODER_100:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_1065:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1065_VOLTAGEINPUT_100:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_1065:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1065_VOLTAGEINPUT_SUPPLY_100:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_1065:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1065_VOLTAGERATIOINPUT_100:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_1065:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1065_CURRENTINPUT_100:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_1065:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1066_RCSERVO_100:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_1066:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1066_CURRENTINPUT_100:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_1066:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1067_STEPPER_200:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_1067:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1067_STEPPER_300:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_1067:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1202_VOLTAGEINPUT_000:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_1202_1203:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1202_VOLTAGERATIOINPUT_000:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_1202_1203:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1202_DIGITALINPUT_000:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_1202_1203:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1202_DIGITALOUTPUT_000:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_1202_1203:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1202_VOLTAGEINPUT_120:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_1202_1203:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1202_VOLTAGERATIOINPUT_120:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_1202_1203:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1202_DIGITALINPUT_120:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_1202_1203:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1202_DIGITALOUTPUT_120:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_1202_1203:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1202_TEXTLCD_000:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_1202_1203:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1202_VOLTAGEINPUT_300:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_1202_1203:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1202_VOLTAGERATIOINPUT_300:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_1202_1203:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1202_DIGITALINPUT_300:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_1202_1203:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1202_DIGITALOUTPUT_300:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_1202_1203:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1202_TEXTLCD_200:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_1202_1203:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1204_TEXTLCD_000:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_1204:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1215_TEXTLCD_000:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_1215__1218:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1219_TEXTLCD_000:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_1219__1222:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1219_DIGITALINPUT_000:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_1219__1222:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1219_DIGITALOUTPUT_000:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_1219__1222:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_HUB_DIGITALINPUT_100:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_DIGITALINPUT_PORT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_HUB_DIGITALOUTPUT_100:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_DIGITALOUTPUT_PORT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_HUB_DIGITALOUTPUT_110:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_DIGITALOUTPUT_PORT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_HUB_VOLTAGEINPUT_100:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_VOLTAGEINPUT_PORT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_HUB_VOLTAGEINPUT_110:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_VOLTAGEINPUT_PORT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_HUB_VOLTAGERATIOINPUT_100:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_VOLTAGERATIOINPUT_PORT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_ADP1000_PHSENSOR_100:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_ADP1000:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_ADP1000_VOLTAGEINPUT_100:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_ADP1000:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DAQ1000_VOLTAGERATIOINPUT_100:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_DAQ1000:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DAQ1000_VOLTAGEINPUT_100:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_DAQ1000:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DAQ1000_VOLTAGERATIOINPUT_110:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_DAQ1000:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DAQ1000_VOLTAGEINPUT_110:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_DAQ1000:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_OUT1000_VOLTAGEOUTPUT_100:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_OUT1000:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_OUT1000_VOLTAGEOUTPUT_110:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_OUT1000:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_OUT1001_VOLTAGEOUTPUT_100:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_OUT1001:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_OUT1001_VOLTAGEOUTPUT_110:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_OUT1001:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_OUT1002_VOLTAGEOUTPUT_100:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_OUT1002:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_OUT1002_VOLTAGEOUTPUT_110:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_OUT1002:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DAQ1200_DIGITALINPUT_100:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_DAQ1200:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_OUT1100_DIGITALOUTPUT_100:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_OUT1100:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_OUT1100_DIGITALOUTPUT_110:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_OUT1100:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_OUT1100_DIGITALOUTPUT_120:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_OUT1100:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DAQ1300_DIGITALINPUT_100:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_DAQ1300:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DAQ1301_DIGITALINPUT_100:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_DAQ1301:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DAQ1400_VOLTAGEINPUT_100:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_DAQ1400:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DAQ1400_CURRENTINPUT_100:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_DAQ1400:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DAQ1400_DIGITALINPUT_100:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_DAQ1400:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DAQ1400_FREQUENCYCOUNTER_100:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_DAQ1400:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DAQ1400_VOLTAGEINPUT_120:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_DAQ1400:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DAQ1400_CURRENTINPUT_120:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_DAQ1400:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DAQ1400_DIGITALINPUT_120:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_DAQ1400:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DAQ1400_FREQUENCYCOUNTER_120:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_DAQ1400:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DAQ1500_VOLTAGERATIOINPUT_100:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_DAQ1500:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_VCP1100_CURRENTINPUT_100:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_VCP1100:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_VCP1100_CURRENTINPUT_200:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_VCP1100:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1000_DCMOTOR_100:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_DCC1000:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1000_ENCODER_100:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_DCC1000:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1000_VOLTAGERATIOINPUT_100:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_DCC1000:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1000_TEMPERATURESENSOR_IC_100:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_DCC1000:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1000_CURRENTINPUT_100:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_DCC1000:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1000_DCMOTOR_200:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_DCC1000:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1000_ENCODER_200:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_DCC1000:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1000_VOLTAGERATIOINPUT_200:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_DCC1000:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1000_TEMPERATURESENSOR_IC_200:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_DCC1000:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1000_CURRENTINPUT_200:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_DCC1000:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1000_MOTORPOSITIONCONTROLLER_200:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_DCC1000:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1000_DCMOTOR_210:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_DCC1000:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1000_ENCODER_210:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_DCC1000:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1000_VOLTAGERATIOINPUT_210:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_DCC1000:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1000_TEMPERATURESENSOR_IC_210:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_DCC1000:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1000_CURRENTINPUT_210:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_DCC1000:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1000_MOTORPOSITIONCONTROLLER_210:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_DCC1000:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1001_DCMOTOR_100:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_DCC1001:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1001_ENCODER_100:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_DCC1001:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1001_MOTORPOSITIONCONTROLLER_100:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_DCC1001:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1001_DCMOTOR_120:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_DCC1001:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1001_ENCODER_120:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_DCC1001:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1001_MOTORPOSITIONCONTROLLER_120:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_DCC1001:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1002_DCMOTOR_100:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_DCC1002:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1002_ENCODER_100:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_DCC1002:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1002_MOTORPOSITIONCONTROLLER_100:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_DCC1002:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1002_DCMOTOR_110:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_DCC1002:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1002_ENCODER_110:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_DCC1002:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1002_MOTORPOSITIONCONTROLLER_110:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_DCC1002:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1003_DCMOTOR_100:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_DCC1003:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1003_DCMOTOR_110:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_DCC1003:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1100_BLDCMOTOR_100:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_DCC1100:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1100_TEMPERATURESENSOR_IC_100:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_DCC1100:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1100_MOTORPOSITIONCONTROLLER_100:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_DCC1100:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1100_BLDCMOTOR_120:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_DCC1100:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1100_TEMPERATURESENSOR_IC_120:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_DCC1100:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1100_MOTORPOSITIONCONTROLLER_120:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_DCC1100:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DST1000_DISTANCESENSOR_100:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_DST1000:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DST1001_DISTANCESENSOR_100:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_DST1001:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DST1002_DISTANCESENSOR_100:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_DST1002:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DST1200_DISTANCESENSOR_100:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_DST1200:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_ENC1000_ENCODER_100:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_ENC1000:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_HIN1101_ENCODER_100:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_HIN1101:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_HIN1101_DIGITALINPUT_100:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_HIN1101:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_HIN1000_CAPACITIVETOUCH_100:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_HIN1000:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_HIN1001_CAPACITIVETOUCH_BUTTONS_100:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_HIN1001:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_HIN1001_CAPACITIVETOUCH_WHEEL_100:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_HIN1001:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_HIN1100_VOLTAGERATIOINPUT_100:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_HIN1100:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_HIN1100_DIGITALINPUT_100:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_HIN1100:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_HUM1000_HUMIDITYSENSOR_100:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_HUM1000:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_HUM1000_TEMPERATURESENSOR_IC_100:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_HUM1000:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_HUM1001_HUMIDITYSENSOR_100:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_HUM1001:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_HUM1001_TEMPERATURESENSOR_IC_100:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_HUM1001:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_LCD1100_LCD_100:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_LCD1100:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_LED1000_DIGITALOUTPUT_100:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_LED1000:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_LUX1000_LIGHTSENSOR_100:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_LUX1000:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_HUM1100_VOLTAGERATIOINPUT_100:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_HUM1100:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_MOT1100_ACCELEROMETER_200:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_MOT1100:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_MOT1101_ACCELEROMETER_100:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_MOT1101:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_MOT1101_GYROSCOPE_100:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_MOT1101:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_MOT1101_MAGNETOMETER_100:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_MOT1101:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_MOT1101_SPATIAL_100:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_MOT1101:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_MOT1102_ACCELEROMETER_200:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_MOT1102:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_MOT1102_GYROSCOPE_200:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_MOT1102:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_MOT1102_MAGNETOMETER_200:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_MOT1102:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_MOT1102_SPATIAL_200:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_MOT1102:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_MOT0109_ACCELEROMETER_100:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_MOT0109:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_MOT0109_GYROSCOPE_100:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_MOT0109:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_MOT0109_MAGNETOMETER_100:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_MOT0109:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_MOT0109_SPATIAL_100:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_MOT0109:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_MOT0109_TEMPERATURESENSOR_100:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_MOT0109:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_PRE1000_PRESSURESENSOR_100:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_PRE1000:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_RCC1000_RCSERVO_100:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_RCC1000:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_RCC1000_RCSERVO_110:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_RCC1000:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_REL1000_DIGITALOUTPUT_100:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_REL1000:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_REL1000_DIGITALOUTPUT_110:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_REL1000:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_REL1100_DIGITALOUTPUT_100:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_REL1100:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_REL1100_DIGITALOUTPUT_110:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_REL1100:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_REL1100_DIGITALOUTPUT_120:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_REL1100:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_REL1101_DIGITALOUTPUT_100:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_REL1101:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_REL1101_DIGITALOUTPUT_110:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_REL1101:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_REL1101_1_DIGITALOUTPUT_FREQ_100:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_REL1101:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_REL1101_1_DIGITALOUTPUT_100:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_REL1101:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_SAF1000_POWERGUARD_100:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_SAF1000:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_SAF1000_VOLTAGEINPUT_100:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_SAF1000:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_SAF1000_TEMPERATURESENSOR_IC_100:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_SAF1000:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_SAF1000_POWERGUARD_110:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_SAF1000:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_SAF1000_VOLTAGEINPUT_110:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_SAF1000:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_SAF1000_TEMPERATURESENSOR_IC_110:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_SAF1000:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_SND1000_SOUNDSENSOR_100:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_SND1000:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_STC1000_STEPPER_100:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_STC1000:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_STC1000_STEPPER_110:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_STC1000:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_STC1001_STEPPER_100:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_STC1001:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_STC1001_STEPPER_110:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_STC1001:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_STC1002_STEPPER_100:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_STC1002:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_STC1002_STEPPER_110:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_STC1002:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_STC1003_STEPPER_100:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_STC1003:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_STC1003_STEPPER_110:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_STC1003:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_STC1003_STEPPER_200:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_STC1003:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_STC1003_VOLTAGEINPUT_200:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_STC1003:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_TMP1000_TEMPERATURESENSOR_IC_100:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_TMP1000:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_TMP1100_TEMPERATURESENSOR_THERMOCOUPLE_100:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_TMP1100:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_TMP1100_TEMPERATURESENSOR_IC_100:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_TMP1100:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_TMP1100_VOLTAGEINPUT_100:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_TMP1100:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_TMP1101_TEMPERATURESENSOR_THERMOCOUPLE_100:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_TMP1101:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_TMP1101_TEMPERATURESENSOR_IC_100:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_TMP1101:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_TMP1101_VOLTAGEINPUT_100:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_TMP1101:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_TMP1101_TEMPERATURESENSOR_THERMOCOUPLE_200:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_TMP1101:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_TMP1101_TEMPERATURESENSOR_IC_200:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_TMP1101:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_TMP1101_VOLTAGEINPUT_200:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_TMP1101:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_TMP1200_TEMPERATURESENSOR_RTD_100:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_TMP1200:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_TMP1200_RESISTANCEINPUT_100:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_TMP1200:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_VCP1000_VOLTAGEINPUT_100:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_VCP1000:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_VCP1001_VOLTAGEINPUT_100:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_VCP1001:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_VCP1001_VOLTAGEINPUT_110:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_VCP1001:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_VCP1002_VOLTAGEINPUT_100:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_VCP1002:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_VCP1002_VOLTAGEINPUT_110:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_VCP1002:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_HUB0000_HUB_100:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_HUB0000:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_HUB0000_HUB_300:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_HUB0000:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_HUB0001_HUB_100:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_HUB0001:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_HUB0004_HUB_100:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_HUB0004:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_HUB0004_HUB_200:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_HUB0004:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_HUB5000_HUB_100:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_HUB5000:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_HUB5000_HUB_200:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_HUB5000:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_M3_USB_FIRMWARE_UPGRADE_000:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_FIRMWARE_UPGRADE_USB:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_STM32_USB_FIRMWARE_UPGRADE_100:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_FIRMWARE_UPGRADE_USB:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_STM32F0_FIRMWARE_UPGRADE_100:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_FIRMWARE_UPGRADE_STM32F0:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_STM32G0_FIRMWARE_UPGRADE_110:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_FIRMWARE_UPGRADE_STM32G0:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_STM8S_FIRMWARE_UPGRADE_100:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_FIRMWARE_UPGRADE_STM8S:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_M3_SPI_FIRMWARE_UPGRADE_000:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_FIRMWARE_UPGRADE_SPI:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DICTIONARY:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_DICTIONARY:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_USB_UNKNOWN:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_UNKNOWN:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_VINT_UNKNOWN:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_UNKNOWN:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_SPI_UNKNOWN:
		switch (val) {
		case PHIDID_NOTHING:
		case PHIDID_UNKNOWN:
			return (1);
		default:
			return (0);
		}
	default:
		return (0);
	}
}

int
supportedDeviceClass(PhidgetChannelHandle ch, Phidget_DeviceClass val) {

	switch (ch->UCD->uid) {
	case PHIDCHUID_ifkit488_VOLTAGERATIOINPUT_000:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_INTERFACEKIT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_ifkit488_DIGITALINPUT_000:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_INTERFACEKIT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_ifkit488_DIGITALOUTPUT_000:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_INTERFACEKIT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1000_RCSERVO_OLD1_200:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_SERVO:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1000_RCSERVO_OLD2_200:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_SERVO:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1000_RCSERVO_300:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_SERVO:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1000_RCSERVO_313:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_SERVO:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1001_RCSERVO_OLD1_200:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_SERVO:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1001_RCSERVO_OLD2_200:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_SERVO:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1001_RCSERVO_313:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_SERVO:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1001_RCSERVO_400:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_SERVO:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1002_VOLTAGEOUTPUT_100:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_ANALOG:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1008_ACCELEROMETER_000:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_ACCELEROMETER:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1011_VOLTAGEINPUT_000:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_INTERFACEKIT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1011_VOLTAGERATIOINPUT_000:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_INTERFACEKIT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1011_DIGITALINPUT_000:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_INTERFACEKIT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1011_DIGITALOUTPUT_000:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_INTERFACEKIT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1012_DIGITALINPUT_000:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_INTERFACEKIT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1012_DIGITALOUTPUT_000:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_INTERFACEKIT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1012_DIGITALINPUT_601:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_INTERFACEKIT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1012_DIGITALOUTPUT_601:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_INTERFACEKIT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1012_DIGITALINPUT_602:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_INTERFACEKIT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1012_DIGITALOUTPUT_602:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_INTERFACEKIT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1013_VOLTAGEINPUT_000:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_INTERFACEKIT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1013_VOLTAGERATIOINPUT_000:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_INTERFACEKIT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1013_DIGITALINPUT_000:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_INTERFACEKIT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1013_DIGITALOUTPUT_000:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_INTERFACEKIT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1018_VOLTAGEINPUT_821:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_INTERFACEKIT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1018_VOLTAGERATIOINPUT_821:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_INTERFACEKIT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1018_DIGITALINPUT_821:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_INTERFACEKIT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1018_DIGITALOUTPUT_821:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_INTERFACEKIT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1014_DIGITALOUTPUT_000:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_INTERFACEKIT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1014_DIGITALOUTPUT_704:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_INTERFACEKIT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1015_CAPACITIVETOUCH_000:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_INTERFACEKIT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1016_CAPACITIVETOUCH_000:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_INTERFACEKIT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1017_DIGITALOUTPUT_000:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_INTERFACEKIT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1018_VOLTAGEINPUT_900:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_INTERFACEKIT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1018_VOLTAGERATIOINPUT_900:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_INTERFACEKIT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1018_DIGITALINPUT_900:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_INTERFACEKIT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1018_DIGITALOUTPUT_900:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_INTERFACEKIT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1018_VOLTAGEINPUT_1000:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_INTERFACEKIT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1018_VOLTAGERATIOINPUT_1000:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_INTERFACEKIT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1018_DIGITALINPUT_1000:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_INTERFACEKIT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1018_DIGITALOUTPUT_1000:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_INTERFACEKIT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1023_RFID_000:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_RFID:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1023_RFID_104:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_RFID:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1023_RFID_200:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_RFID:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1023_DIGITALOUTPUT_5V_200:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_RFID:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1023_DIGITALOUTPUT_LED_200:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_RFID:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1023_DIGITALOUTPUT_ONBOARD_LED_200:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_RFID:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1023_RFID_201:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_RFID:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1023_DIGITALOUTPUT_5V_201:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_RFID:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1023_DIGITALOUTPUT_LED_201:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_RFID:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1023_DIGITALOUTPUT_ONBOARD_LED_201:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_RFID:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1024_RFID_100:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_RFID:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1024_DIGITALOUTPUT_5V_100:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_RFID:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1024_DIGITALOUTPUT_LED_100:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_RFID:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1024_DIGITALOUTPUT_ONBOARD_LED_100:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_RFID:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1030_DIGITALOUTPUT_100:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_LED:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1031_DIGITALOUTPUT_100:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_LED:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1032_DIGITALOUTPUT_200:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_LED:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1040_GPS_000:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_GPS:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1041_ACCELEROMETER_200:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_SPATIAL:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1042_ACCELEROMETER_300:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_SPATIAL:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1042_GYROSCOPE_300:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_SPATIAL:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1042_MAGNETOMETER_300:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_SPATIAL:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1042_SPATIAL_300:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_SPATIAL:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1043_ACCELEROMETER_300:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_SPATIAL:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1044_ACCELEROMETER_400:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_SPATIAL:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1044_GYROSCOPE_400:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_SPATIAL:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1044_MAGNETOMETER_400:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_SPATIAL:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1044_SPATIAL_400:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_SPATIAL:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1044_ACCELEROMETER_500:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_SPATIAL:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1044_GYROSCOPE_500:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_SPATIAL:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1044_MAGNETOMETER_500:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_SPATIAL:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1044_SPATIAL_500:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_SPATIAL:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1044_ACCELEROMETER_510:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_SPATIAL:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1044_GYROSCOPE_510:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_SPATIAL:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1044_MAGNETOMETER_510:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_SPATIAL:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1044_SPATIAL_510:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_SPATIAL:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1045_TEMPERATURESENSOR_IR_100:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_TEMPERATURESENSOR:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1045_TEMPERATURESENSOR_IC_100:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_TEMPERATURESENSOR:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1046_VOLTAGERATIOINPUT_100:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_BRIDGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1046_VOLTAGERATIOINPUT_102:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_BRIDGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1047_ENCODER_100:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_ENCODER:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1047_DIGITALINPUT_100:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_ENCODER:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1047_ENCODER_200:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_ENCODER:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1047_DIGITALINPUT_200:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_ENCODER:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1048_TEMPERATURESENSOR_THERMOCOUPLE_100:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_TEMPERATURESENSOR:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1048_TEMPERATURESENSOR_IC_100:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_TEMPERATURESENSOR:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1048_VOLTAGEINPUT_100:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_TEMPERATURESENSOR:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1048_TEMPERATURESENSOR_THERMOCOUPLE_200:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_TEMPERATURESENSOR:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1048_TEMPERATURESENSOR_IC_200:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_TEMPERATURESENSOR:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1048_VOLTAGEINPUT_200:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_TEMPERATURESENSOR:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1049_ACCELEROMETER_000:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_SPATIAL:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1051_TEMPERATURESENSOR_THERMOCOUPLE_000:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_TEMPERATURESENSOR:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1051_TEMPERATURESENSOR_IC_000:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_TEMPERATURESENSOR:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1051_TEMPERATURESENSOR_THERMOCOUPLE_200:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_TEMPERATURESENSOR:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1051_TEMPERATURESENSOR_IC_200:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_TEMPERATURESENSOR:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1051_VOLTAGEINPUT_200:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_TEMPERATURESENSOR:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1051_TEMPERATURESENSOR_THERMOCOUPLE_300:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_TEMPERATURESENSOR:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1051_TEMPERATURESENSOR_IC_300:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_TEMPERATURESENSOR:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1051_VOLTAGEINPUT_300:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_TEMPERATURESENSOR:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1051_TEMPERATURESENSOR_THERMOCOUPLE_400:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_TEMPERATURESENSOR:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1051_TEMPERATURESENSOR_IC_400:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_TEMPERATURESENSOR:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1051_VOLTAGEINPUT_400:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_TEMPERATURESENSOR:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1052_ENCODER_000:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_ENCODER:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1052_DIGITALINPUT_000:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_ENCODER:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1052_ENCODER_101:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_ENCODER:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1052_DIGITALINPUT_101:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_ENCODER:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1052_ENCODER_110:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_ENCODER:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1052_DIGITALINPUT_110:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_ENCODER:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1053_ACCELEROMETER_300:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_ACCELEROMETER:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1054_FREQUENCYCOUNTER_000:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_FREQUENCYCOUNTER:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1055_IR_100:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_IR:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1055_IR_200_USB:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_IR:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1055_IR_200_VINT:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_IR:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1056_ACCELEROMETER_000:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_SPATIAL:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1056_GYROSCOPE_000:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_SPATIAL:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1056_MAGNETOMETER_000:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_SPATIAL:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1056_SPATIAL_000:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_SPATIAL:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1056_ACCELEROMETER_200:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_SPATIAL:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1056_GYROSCOPE_200:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_SPATIAL:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1056_MAGNETOMETER_200:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_SPATIAL:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1056_SPATIAL_200:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_SPATIAL:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1057_ENCODER_300:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_ENCODER:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1057_ENCODER_400:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_ENCODER:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1058_VOLTAGEINPUT_100:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_PHSENSOR:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1058_PHADAPTER_100:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_PHSENSOR:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1059_ACCELEROMETER_400:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_ACCELEROMETER:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1060_DCMOTOR_100:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_MOTORCONTROL:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1060_DIGITALINPUT_100:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_MOTORCONTROL:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1061_RCSERVO_100:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_ADVANCEDSERVO:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1061_CURRENTINPUT_100:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_ADVANCEDSERVO:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1061_RCSERVO_200:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_ADVANCEDSERVO:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1061_CURRENTINPUT_200:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_ADVANCEDSERVO:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1061_RCSERVO_300:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_ADVANCEDSERVO:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1061_CURRENTINPUT_300:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_ADVANCEDSERVO:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_RCC0004_RCSERVO_400:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_ADVANCEDSERVO:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1062_STEPPER_100:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_STEPPER:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1063_STEPPER_100:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_STEPPER:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1063_DIGITALINPUT_100:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_STEPPER:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1063_CURRENTINPUT_100:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_STEPPER:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1064_DCMOTOR_100:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_MOTORCONTROL:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1064_CURRENTINPUT_100:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_MOTORCONTROL:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1065_DCMOTOR_100:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_MOTORCONTROL:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1065_DIGITALINPUT_100:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_MOTORCONTROL:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1065_ENCODER_100:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_MOTORCONTROL:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1065_VOLTAGEINPUT_100:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_MOTORCONTROL:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1065_VOLTAGEINPUT_SUPPLY_100:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_MOTORCONTROL:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1065_VOLTAGERATIOINPUT_100:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_MOTORCONTROL:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1065_CURRENTINPUT_100:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_MOTORCONTROL:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1066_RCSERVO_100:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_ADVANCEDSERVO:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1066_CURRENTINPUT_100:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_ADVANCEDSERVO:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1067_STEPPER_200:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_STEPPER:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1067_STEPPER_300:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_STEPPER:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1202_VOLTAGEINPUT_000:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_INTERFACEKIT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1202_VOLTAGERATIOINPUT_000:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_INTERFACEKIT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1202_DIGITALINPUT_000:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_INTERFACEKIT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1202_DIGITALOUTPUT_000:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_INTERFACEKIT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1202_VOLTAGEINPUT_120:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_INTERFACEKIT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1202_VOLTAGERATIOINPUT_120:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_INTERFACEKIT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1202_DIGITALINPUT_120:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_INTERFACEKIT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1202_DIGITALOUTPUT_120:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_INTERFACEKIT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1202_TEXTLCD_000:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_TEXTLCD:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1202_VOLTAGEINPUT_300:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_INTERFACEKIT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1202_VOLTAGERATIOINPUT_300:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_INTERFACEKIT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1202_DIGITALINPUT_300:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_INTERFACEKIT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1202_DIGITALOUTPUT_300:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_INTERFACEKIT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1202_TEXTLCD_200:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_TEXTLCD:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1204_TEXTLCD_000:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_TEXTLCD:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1215_TEXTLCD_000:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_TEXTLCD:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1219_TEXTLCD_000:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_TEXTLCD:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1219_DIGITALINPUT_000:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_TEXTLCD:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1219_DIGITALOUTPUT_000:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_TEXTLCD:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_HUB_DIGITALINPUT_100:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_VINT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_HUB_DIGITALOUTPUT_100:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_VINT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_HUB_DIGITALOUTPUT_110:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_VINT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_HUB_VOLTAGEINPUT_100:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_VINT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_HUB_VOLTAGEINPUT_110:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_VINT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_HUB_VOLTAGERATIOINPUT_100:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_VINT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_ADP1000_PHSENSOR_100:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_VINT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_ADP1000_VOLTAGEINPUT_100:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_VINT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DAQ1000_VOLTAGERATIOINPUT_100:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_VINT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DAQ1000_VOLTAGEINPUT_100:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_VINT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DAQ1000_VOLTAGERATIOINPUT_110:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_VINT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DAQ1000_VOLTAGEINPUT_110:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_VINT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_OUT1000_VOLTAGEOUTPUT_100:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_VINT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_OUT1000_VOLTAGEOUTPUT_110:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_VINT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_OUT1001_VOLTAGEOUTPUT_100:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_VINT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_OUT1001_VOLTAGEOUTPUT_110:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_VINT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_OUT1002_VOLTAGEOUTPUT_100:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_VINT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_OUT1002_VOLTAGEOUTPUT_110:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_VINT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DAQ1200_DIGITALINPUT_100:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_VINT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_OUT1100_DIGITALOUTPUT_100:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_VINT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_OUT1100_DIGITALOUTPUT_110:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_VINT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_OUT1100_DIGITALOUTPUT_120:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_VINT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DAQ1300_DIGITALINPUT_100:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_VINT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DAQ1301_DIGITALINPUT_100:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_VINT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DAQ1400_VOLTAGEINPUT_100:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_VINT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DAQ1400_CURRENTINPUT_100:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_VINT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DAQ1400_DIGITALINPUT_100:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_VINT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DAQ1400_FREQUENCYCOUNTER_100:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_VINT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DAQ1400_VOLTAGEINPUT_120:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_VINT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DAQ1400_CURRENTINPUT_120:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_VINT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DAQ1400_DIGITALINPUT_120:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_VINT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DAQ1400_FREQUENCYCOUNTER_120:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_VINT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DAQ1500_VOLTAGERATIOINPUT_100:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_VINT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_VCP1100_CURRENTINPUT_100:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_VINT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_VCP1100_CURRENTINPUT_200:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_VINT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1000_DCMOTOR_100:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_VINT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1000_ENCODER_100:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_VINT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1000_VOLTAGERATIOINPUT_100:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_VINT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1000_TEMPERATURESENSOR_IC_100:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_VINT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1000_CURRENTINPUT_100:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_VINT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1000_DCMOTOR_200:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_VINT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1000_ENCODER_200:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_VINT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1000_VOLTAGERATIOINPUT_200:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_VINT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1000_TEMPERATURESENSOR_IC_200:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_VINT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1000_CURRENTINPUT_200:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_VINT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1000_MOTORPOSITIONCONTROLLER_200:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_VINT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1000_DCMOTOR_210:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_VINT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1000_ENCODER_210:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_VINT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1000_VOLTAGERATIOINPUT_210:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_VINT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1000_TEMPERATURESENSOR_IC_210:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_VINT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1000_CURRENTINPUT_210:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_VINT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1000_MOTORPOSITIONCONTROLLER_210:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_VINT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1001_DCMOTOR_100:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_VINT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1001_ENCODER_100:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_VINT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1001_MOTORPOSITIONCONTROLLER_100:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_VINT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1001_DCMOTOR_120:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_VINT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1001_ENCODER_120:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_VINT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1001_MOTORPOSITIONCONTROLLER_120:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_VINT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1002_DCMOTOR_100:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_VINT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1002_ENCODER_100:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_VINT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1002_MOTORPOSITIONCONTROLLER_100:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_VINT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1002_DCMOTOR_110:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_VINT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1002_ENCODER_110:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_VINT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1002_MOTORPOSITIONCONTROLLER_110:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_VINT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1003_DCMOTOR_100:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_VINT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1003_DCMOTOR_110:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_VINT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1100_BLDCMOTOR_100:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_VINT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1100_TEMPERATURESENSOR_IC_100:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_VINT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1100_MOTORPOSITIONCONTROLLER_100:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_VINT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1100_BLDCMOTOR_120:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_VINT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1100_TEMPERATURESENSOR_IC_120:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_VINT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1100_MOTORPOSITIONCONTROLLER_120:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_VINT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DST1000_DISTANCESENSOR_100:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_VINT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DST1001_DISTANCESENSOR_100:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_VINT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DST1002_DISTANCESENSOR_100:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_VINT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DST1200_DISTANCESENSOR_100:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_VINT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_ENC1000_ENCODER_100:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_VINT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_HIN1101_ENCODER_100:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_VINT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_HIN1101_DIGITALINPUT_100:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_VINT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_HIN1000_CAPACITIVETOUCH_100:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_VINT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_HIN1001_CAPACITIVETOUCH_BUTTONS_100:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_VINT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_HIN1001_CAPACITIVETOUCH_WHEEL_100:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_VINT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_HIN1100_VOLTAGERATIOINPUT_100:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_VINT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_HIN1100_DIGITALINPUT_100:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_VINT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_HUM1000_HUMIDITYSENSOR_100:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_VINT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_HUM1000_TEMPERATURESENSOR_IC_100:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_VINT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_HUM1001_HUMIDITYSENSOR_100:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_VINT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_HUM1001_TEMPERATURESENSOR_IC_100:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_VINT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_LCD1100_LCD_100:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_VINT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_LED1000_DIGITALOUTPUT_100:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_VINT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_LUX1000_LIGHTSENSOR_100:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_VINT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_HUM1100_VOLTAGERATIOINPUT_100:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_VINT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_MOT1100_ACCELEROMETER_200:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_VINT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_MOT1101_ACCELEROMETER_100:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_VINT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_MOT1101_GYROSCOPE_100:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_VINT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_MOT1101_MAGNETOMETER_100:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_VINT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_MOT1101_SPATIAL_100:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_VINT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_MOT1102_ACCELEROMETER_200:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_VINT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_MOT1102_GYROSCOPE_200:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_VINT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_MOT1102_MAGNETOMETER_200:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_VINT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_MOT1102_SPATIAL_200:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_VINT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_MOT0109_ACCELEROMETER_100:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_SPATIAL:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_MOT0109_GYROSCOPE_100:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_SPATIAL:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_MOT0109_MAGNETOMETER_100:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_SPATIAL:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_MOT0109_SPATIAL_100:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_SPATIAL:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_MOT0109_TEMPERATURESENSOR_100:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_SPATIAL:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_PRE1000_PRESSURESENSOR_100:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_VINT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_RCC1000_RCSERVO_100:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_VINT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_RCC1000_RCSERVO_110:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_VINT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_REL1000_DIGITALOUTPUT_100:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_VINT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_REL1000_DIGITALOUTPUT_110:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_VINT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_REL1100_DIGITALOUTPUT_100:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_VINT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_REL1100_DIGITALOUTPUT_110:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_VINT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_REL1100_DIGITALOUTPUT_120:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_VINT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_REL1101_DIGITALOUTPUT_100:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_VINT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_REL1101_DIGITALOUTPUT_110:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_VINT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_REL1101_1_DIGITALOUTPUT_FREQ_100:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_VINT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_REL1101_1_DIGITALOUTPUT_100:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_VINT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_SAF1000_POWERGUARD_100:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_VINT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_SAF1000_VOLTAGEINPUT_100:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_VINT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_SAF1000_TEMPERATURESENSOR_IC_100:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_VINT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_SAF1000_POWERGUARD_110:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_VINT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_SAF1000_VOLTAGEINPUT_110:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_VINT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_SAF1000_TEMPERATURESENSOR_IC_110:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_VINT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_SND1000_SOUNDSENSOR_100:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_VINT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_STC1000_STEPPER_100:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_VINT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_STC1000_STEPPER_110:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_VINT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_STC1001_STEPPER_100:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_VINT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_STC1001_STEPPER_110:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_VINT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_STC1002_STEPPER_100:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_VINT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_STC1002_STEPPER_110:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_VINT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_STC1003_STEPPER_100:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_VINT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_STC1003_STEPPER_110:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_VINT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_STC1003_STEPPER_200:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_VINT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_STC1003_VOLTAGEINPUT_200:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_VINT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_TMP1000_TEMPERATURESENSOR_IC_100:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_VINT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_TMP1100_TEMPERATURESENSOR_THERMOCOUPLE_100:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_VINT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_TMP1100_TEMPERATURESENSOR_IC_100:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_VINT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_TMP1100_VOLTAGEINPUT_100:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_VINT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_TMP1101_TEMPERATURESENSOR_THERMOCOUPLE_100:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_VINT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_TMP1101_TEMPERATURESENSOR_IC_100:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_VINT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_TMP1101_VOLTAGEINPUT_100:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_VINT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_TMP1101_TEMPERATURESENSOR_THERMOCOUPLE_200:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_VINT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_TMP1101_TEMPERATURESENSOR_IC_200:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_VINT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_TMP1101_VOLTAGEINPUT_200:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_VINT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_TMP1200_TEMPERATURESENSOR_RTD_100:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_VINT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_TMP1200_RESISTANCEINPUT_100:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_VINT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_VCP1000_VOLTAGEINPUT_100:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_VINT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_VCP1001_VOLTAGEINPUT_100:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_VINT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_VCP1001_VOLTAGEINPUT_110:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_VINT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_VCP1002_VOLTAGEINPUT_100:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_VINT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_VCP1002_VOLTAGEINPUT_110:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_VINT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_HUB0000_HUB_100:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_HUB:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_HUB0000_HUB_300:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_HUB:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_HUB0001_HUB_100:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_HUB:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_HUB0004_HUB_100:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_HUB:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_HUB0004_HUB_200:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_HUB:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_HUB5000_HUB_100:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_HUB:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_HUB5000_HUB_200:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_HUB:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_M3_USB_FIRMWARE_UPGRADE_000:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_FIRMWAREUPGRADE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_STM32_USB_FIRMWARE_UPGRADE_100:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_FIRMWAREUPGRADE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_STM32F0_FIRMWARE_UPGRADE_100:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_VINT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_STM32G0_FIRMWARE_UPGRADE_110:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_VINT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_STM8S_FIRMWARE_UPGRADE_100:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_VINT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_M3_SPI_FIRMWARE_UPGRADE_000:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_FIRMWAREUPGRADE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DICTIONARY:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_DICTIONARY:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_USB_UNKNOWN:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_GENERIC:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_VINT_UNKNOWN:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_VINT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_SPI_UNKNOWN:
		switch (val) {
		case PHIDCLASS_NOTHING:
		case PHIDCLASS_GENERIC:
			return (1);
		default:
			return (0);
		}
	default:
		return (0);
	}
}

int
supportedChannelClass(PhidgetChannelHandle ch, Phidget_ChannelClass val) {

	switch (ch->UCD->uid) {
	case PHIDCHUID_ifkit488_VOLTAGERATIOINPUT_000:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_VOLTAGERATIOINPUT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_ifkit488_DIGITALINPUT_000:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_DIGITALINPUT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_ifkit488_DIGITALOUTPUT_000:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_DIGITALOUTPUT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1000_RCSERVO_OLD1_200:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_RCSERVO:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1000_RCSERVO_OLD2_200:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_RCSERVO:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1000_RCSERVO_300:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_RCSERVO:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1000_RCSERVO_313:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_RCSERVO:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1001_RCSERVO_OLD1_200:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_RCSERVO:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1001_RCSERVO_OLD2_200:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_RCSERVO:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1001_RCSERVO_313:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_RCSERVO:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1001_RCSERVO_400:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_RCSERVO:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1002_VOLTAGEOUTPUT_100:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_VOLTAGEOUTPUT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1008_ACCELEROMETER_000:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_ACCELEROMETER:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1011_VOLTAGEINPUT_000:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_VOLTAGEINPUT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1011_VOLTAGERATIOINPUT_000:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_VOLTAGERATIOINPUT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1011_DIGITALINPUT_000:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_DIGITALINPUT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1011_DIGITALOUTPUT_000:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_DIGITALOUTPUT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1012_DIGITALINPUT_000:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_DIGITALINPUT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1012_DIGITALOUTPUT_000:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_DIGITALOUTPUT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1012_DIGITALINPUT_601:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_DIGITALINPUT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1012_DIGITALOUTPUT_601:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_DIGITALOUTPUT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1012_DIGITALINPUT_602:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_DIGITALINPUT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1012_DIGITALOUTPUT_602:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_DIGITALOUTPUT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1013_VOLTAGEINPUT_000:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_VOLTAGEINPUT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1013_VOLTAGERATIOINPUT_000:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_VOLTAGERATIOINPUT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1013_DIGITALINPUT_000:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_DIGITALINPUT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1013_DIGITALOUTPUT_000:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_DIGITALOUTPUT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1018_VOLTAGEINPUT_821:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_VOLTAGEINPUT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1018_VOLTAGERATIOINPUT_821:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_VOLTAGERATIOINPUT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1018_DIGITALINPUT_821:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_DIGITALINPUT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1018_DIGITALOUTPUT_821:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_DIGITALOUTPUT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1014_DIGITALOUTPUT_000:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_DIGITALOUTPUT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1014_DIGITALOUTPUT_704:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_DIGITALOUTPUT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1015_CAPACITIVETOUCH_000:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_CAPACITIVETOUCH:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1016_CAPACITIVETOUCH_000:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_CAPACITIVETOUCH:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1017_DIGITALOUTPUT_000:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_DIGITALOUTPUT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1018_VOLTAGEINPUT_900:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_VOLTAGEINPUT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1018_VOLTAGERATIOINPUT_900:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_VOLTAGERATIOINPUT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1018_DIGITALINPUT_900:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_DIGITALINPUT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1018_DIGITALOUTPUT_900:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_DIGITALOUTPUT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1018_VOLTAGEINPUT_1000:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_VOLTAGEINPUT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1018_VOLTAGERATIOINPUT_1000:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_VOLTAGERATIOINPUT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1018_DIGITALINPUT_1000:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_DIGITALINPUT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1018_DIGITALOUTPUT_1000:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_DIGITALOUTPUT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1023_RFID_000:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_RFID:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1023_RFID_104:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_RFID:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1023_RFID_200:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_RFID:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1023_DIGITALOUTPUT_5V_200:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_DIGITALOUTPUT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1023_DIGITALOUTPUT_LED_200:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_DIGITALOUTPUT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1023_DIGITALOUTPUT_ONBOARD_LED_200:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_DIGITALOUTPUT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1023_RFID_201:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_RFID:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1023_DIGITALOUTPUT_5V_201:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_DIGITALOUTPUT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1023_DIGITALOUTPUT_LED_201:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_DIGITALOUTPUT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1023_DIGITALOUTPUT_ONBOARD_LED_201:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_DIGITALOUTPUT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1024_RFID_100:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_RFID:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1024_DIGITALOUTPUT_5V_100:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_DIGITALOUTPUT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1024_DIGITALOUTPUT_LED_100:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_DIGITALOUTPUT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1024_DIGITALOUTPUT_ONBOARD_LED_100:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_DIGITALOUTPUT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1030_DIGITALOUTPUT_100:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_DIGITALOUTPUT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1031_DIGITALOUTPUT_100:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_DIGITALOUTPUT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1032_DIGITALOUTPUT_200:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_DIGITALOUTPUT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1040_GPS_000:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_GPS:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1041_ACCELEROMETER_200:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_ACCELEROMETER:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1042_ACCELEROMETER_300:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_ACCELEROMETER:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1042_GYROSCOPE_300:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_GYROSCOPE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1042_MAGNETOMETER_300:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_MAGNETOMETER:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1042_SPATIAL_300:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_SPATIAL:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1043_ACCELEROMETER_300:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_ACCELEROMETER:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1044_ACCELEROMETER_400:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_ACCELEROMETER:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1044_GYROSCOPE_400:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_GYROSCOPE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1044_MAGNETOMETER_400:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_MAGNETOMETER:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1044_SPATIAL_400:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_SPATIAL:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1044_ACCELEROMETER_500:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_ACCELEROMETER:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1044_GYROSCOPE_500:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_GYROSCOPE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1044_MAGNETOMETER_500:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_MAGNETOMETER:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1044_SPATIAL_500:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_SPATIAL:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1044_ACCELEROMETER_510:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_ACCELEROMETER:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1044_GYROSCOPE_510:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_GYROSCOPE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1044_MAGNETOMETER_510:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_MAGNETOMETER:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1044_SPATIAL_510:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_SPATIAL:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1045_TEMPERATURESENSOR_IR_100:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_TEMPERATURESENSOR:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1045_TEMPERATURESENSOR_IC_100:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_TEMPERATURESENSOR:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1046_VOLTAGERATIOINPUT_100:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_VOLTAGERATIOINPUT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1046_VOLTAGERATIOINPUT_102:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_VOLTAGERATIOINPUT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1047_ENCODER_100:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_ENCODER:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1047_DIGITALINPUT_100:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_DIGITALINPUT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1047_ENCODER_200:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_ENCODER:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1047_DIGITALINPUT_200:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_DIGITALINPUT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1048_TEMPERATURESENSOR_THERMOCOUPLE_100:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_TEMPERATURESENSOR:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1048_TEMPERATURESENSOR_IC_100:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_TEMPERATURESENSOR:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1048_VOLTAGEINPUT_100:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_VOLTAGEINPUT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1048_TEMPERATURESENSOR_THERMOCOUPLE_200:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_TEMPERATURESENSOR:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1048_TEMPERATURESENSOR_IC_200:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_TEMPERATURESENSOR:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1048_VOLTAGEINPUT_200:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_VOLTAGEINPUT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1049_ACCELEROMETER_000:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_ACCELEROMETER:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1051_TEMPERATURESENSOR_THERMOCOUPLE_000:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_TEMPERATURESENSOR:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1051_TEMPERATURESENSOR_IC_000:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_TEMPERATURESENSOR:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1051_TEMPERATURESENSOR_THERMOCOUPLE_200:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_TEMPERATURESENSOR:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1051_TEMPERATURESENSOR_IC_200:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_TEMPERATURESENSOR:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1051_VOLTAGEINPUT_200:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_VOLTAGEINPUT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1051_TEMPERATURESENSOR_THERMOCOUPLE_300:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_TEMPERATURESENSOR:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1051_TEMPERATURESENSOR_IC_300:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_TEMPERATURESENSOR:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1051_VOLTAGEINPUT_300:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_VOLTAGEINPUT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1051_TEMPERATURESENSOR_THERMOCOUPLE_400:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_TEMPERATURESENSOR:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1051_TEMPERATURESENSOR_IC_400:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_TEMPERATURESENSOR:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1051_VOLTAGEINPUT_400:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_VOLTAGEINPUT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1052_ENCODER_000:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_ENCODER:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1052_DIGITALINPUT_000:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_DIGITALINPUT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1052_ENCODER_101:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_ENCODER:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1052_DIGITALINPUT_101:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_DIGITALINPUT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1052_ENCODER_110:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_ENCODER:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1052_DIGITALINPUT_110:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_DIGITALINPUT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1053_ACCELEROMETER_300:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_ACCELEROMETER:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1054_FREQUENCYCOUNTER_000:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_FREQUENCYCOUNTER:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1055_IR_100:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_IR:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1055_IR_200_USB:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_IR:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1055_IR_200_VINT:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_IR:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1056_ACCELEROMETER_000:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_ACCELEROMETER:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1056_GYROSCOPE_000:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_GYROSCOPE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1056_MAGNETOMETER_000:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_MAGNETOMETER:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1056_SPATIAL_000:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_SPATIAL:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1056_ACCELEROMETER_200:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_ACCELEROMETER:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1056_GYROSCOPE_200:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_GYROSCOPE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1056_MAGNETOMETER_200:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_MAGNETOMETER:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1056_SPATIAL_200:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_SPATIAL:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1057_ENCODER_300:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_ENCODER:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1057_ENCODER_400:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_ENCODER:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1058_VOLTAGEINPUT_100:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_VOLTAGEINPUT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1058_PHADAPTER_100:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_PHSENSOR:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1059_ACCELEROMETER_400:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_ACCELEROMETER:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1060_DCMOTOR_100:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_DCMOTOR:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1060_DIGITALINPUT_100:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_DIGITALINPUT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1061_RCSERVO_100:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_RCSERVO:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1061_CURRENTINPUT_100:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_CURRENTINPUT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1061_RCSERVO_200:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_RCSERVO:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1061_CURRENTINPUT_200:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_CURRENTINPUT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1061_RCSERVO_300:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_RCSERVO:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1061_CURRENTINPUT_300:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_CURRENTINPUT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_RCC0004_RCSERVO_400:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_RCSERVO:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1062_STEPPER_100:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_STEPPER:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1063_STEPPER_100:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_STEPPER:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1063_DIGITALINPUT_100:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_DIGITALINPUT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1063_CURRENTINPUT_100:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_CURRENTINPUT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1064_DCMOTOR_100:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_DCMOTOR:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1064_CURRENTINPUT_100:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_CURRENTINPUT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1065_DCMOTOR_100:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_DCMOTOR:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1065_DIGITALINPUT_100:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_DIGITALINPUT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1065_ENCODER_100:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_ENCODER:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1065_VOLTAGEINPUT_100:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_VOLTAGEINPUT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1065_VOLTAGEINPUT_SUPPLY_100:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_VOLTAGEINPUT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1065_VOLTAGERATIOINPUT_100:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_VOLTAGERATIOINPUT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1065_CURRENTINPUT_100:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_CURRENTINPUT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1066_RCSERVO_100:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_RCSERVO:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1066_CURRENTINPUT_100:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_CURRENTINPUT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1067_STEPPER_200:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_STEPPER:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1067_STEPPER_300:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_STEPPER:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1202_VOLTAGEINPUT_000:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_VOLTAGEINPUT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1202_VOLTAGERATIOINPUT_000:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_VOLTAGERATIOINPUT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1202_DIGITALINPUT_000:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_DIGITALINPUT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1202_DIGITALOUTPUT_000:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_DIGITALOUTPUT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1202_VOLTAGEINPUT_120:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_VOLTAGEINPUT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1202_VOLTAGERATIOINPUT_120:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_VOLTAGERATIOINPUT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1202_DIGITALINPUT_120:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_DIGITALINPUT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1202_DIGITALOUTPUT_120:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_DIGITALOUTPUT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1202_TEXTLCD_000:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_LCD:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1202_VOLTAGEINPUT_300:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_VOLTAGEINPUT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1202_VOLTAGERATIOINPUT_300:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_VOLTAGERATIOINPUT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1202_DIGITALINPUT_300:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_DIGITALINPUT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1202_DIGITALOUTPUT_300:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_DIGITALOUTPUT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1202_TEXTLCD_200:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_LCD:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1204_TEXTLCD_000:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_LCD:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1215_TEXTLCD_000:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_LCD:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1219_TEXTLCD_000:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_LCD:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1219_DIGITALINPUT_000:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_DIGITALINPUT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1219_DIGITALOUTPUT_000:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_DIGITALOUTPUT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_HUB_DIGITALINPUT_100:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_DIGITALINPUT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_HUB_DIGITALOUTPUT_100:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_DIGITALOUTPUT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_HUB_DIGITALOUTPUT_110:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_DIGITALOUTPUT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_HUB_VOLTAGEINPUT_100:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_VOLTAGEINPUT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_HUB_VOLTAGEINPUT_110:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_VOLTAGEINPUT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_HUB_VOLTAGERATIOINPUT_100:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_VOLTAGERATIOINPUT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_ADP1000_PHSENSOR_100:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_PHSENSOR:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_ADP1000_VOLTAGEINPUT_100:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_VOLTAGEINPUT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DAQ1000_VOLTAGERATIOINPUT_100:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_VOLTAGERATIOINPUT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DAQ1000_VOLTAGEINPUT_100:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_VOLTAGEINPUT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DAQ1000_VOLTAGERATIOINPUT_110:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_VOLTAGERATIOINPUT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DAQ1000_VOLTAGEINPUT_110:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_VOLTAGEINPUT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_OUT1000_VOLTAGEOUTPUT_100:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_VOLTAGEOUTPUT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_OUT1000_VOLTAGEOUTPUT_110:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_VOLTAGEOUTPUT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_OUT1001_VOLTAGEOUTPUT_100:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_VOLTAGEOUTPUT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_OUT1001_VOLTAGEOUTPUT_110:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_VOLTAGEOUTPUT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_OUT1002_VOLTAGEOUTPUT_100:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_VOLTAGEOUTPUT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_OUT1002_VOLTAGEOUTPUT_110:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_VOLTAGEOUTPUT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DAQ1200_DIGITALINPUT_100:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_DIGITALINPUT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_OUT1100_DIGITALOUTPUT_100:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_DIGITALOUTPUT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_OUT1100_DIGITALOUTPUT_110:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_DIGITALOUTPUT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_OUT1100_DIGITALOUTPUT_120:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_DIGITALOUTPUT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DAQ1300_DIGITALINPUT_100:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_DIGITALINPUT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DAQ1301_DIGITALINPUT_100:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_DIGITALINPUT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DAQ1400_VOLTAGEINPUT_100:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_VOLTAGEINPUT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DAQ1400_CURRENTINPUT_100:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_CURRENTINPUT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DAQ1400_DIGITALINPUT_100:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_DIGITALINPUT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DAQ1400_FREQUENCYCOUNTER_100:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_FREQUENCYCOUNTER:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DAQ1400_VOLTAGEINPUT_120:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_VOLTAGEINPUT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DAQ1400_CURRENTINPUT_120:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_CURRENTINPUT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DAQ1400_DIGITALINPUT_120:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_DIGITALINPUT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DAQ1400_FREQUENCYCOUNTER_120:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_FREQUENCYCOUNTER:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DAQ1500_VOLTAGERATIOINPUT_100:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_VOLTAGERATIOINPUT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_VCP1100_CURRENTINPUT_100:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_CURRENTINPUT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_VCP1100_CURRENTINPUT_200:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_CURRENTINPUT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1000_DCMOTOR_100:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_DCMOTOR:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1000_ENCODER_100:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_ENCODER:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1000_VOLTAGERATIOINPUT_100:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_VOLTAGERATIOINPUT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1000_TEMPERATURESENSOR_IC_100:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_TEMPERATURESENSOR:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1000_CURRENTINPUT_100:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_CURRENTINPUT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1000_DCMOTOR_200:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_DCMOTOR:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1000_ENCODER_200:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_ENCODER:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1000_VOLTAGERATIOINPUT_200:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_VOLTAGERATIOINPUT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1000_TEMPERATURESENSOR_IC_200:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_TEMPERATURESENSOR:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1000_CURRENTINPUT_200:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_CURRENTINPUT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1000_MOTORPOSITIONCONTROLLER_200:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_MOTORPOSITIONCONTROLLER:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1000_DCMOTOR_210:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_DCMOTOR:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1000_ENCODER_210:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_ENCODER:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1000_VOLTAGERATIOINPUT_210:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_VOLTAGERATIOINPUT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1000_TEMPERATURESENSOR_IC_210:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_TEMPERATURESENSOR:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1000_CURRENTINPUT_210:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_CURRENTINPUT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1000_MOTORPOSITIONCONTROLLER_210:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_MOTORPOSITIONCONTROLLER:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1001_DCMOTOR_100:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_DCMOTOR:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1001_ENCODER_100:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_ENCODER:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1001_MOTORPOSITIONCONTROLLER_100:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_MOTORPOSITIONCONTROLLER:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1001_DCMOTOR_120:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_DCMOTOR:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1001_ENCODER_120:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_ENCODER:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1001_MOTORPOSITIONCONTROLLER_120:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_MOTORPOSITIONCONTROLLER:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1002_DCMOTOR_100:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_DCMOTOR:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1002_ENCODER_100:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_ENCODER:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1002_MOTORPOSITIONCONTROLLER_100:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_MOTORPOSITIONCONTROLLER:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1002_DCMOTOR_110:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_DCMOTOR:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1002_ENCODER_110:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_ENCODER:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1002_MOTORPOSITIONCONTROLLER_110:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_MOTORPOSITIONCONTROLLER:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1003_DCMOTOR_100:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_DCMOTOR:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1003_DCMOTOR_110:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_DCMOTOR:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1100_BLDCMOTOR_100:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_BLDCMOTOR:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1100_TEMPERATURESENSOR_IC_100:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_TEMPERATURESENSOR:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1100_MOTORPOSITIONCONTROLLER_100:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_MOTORPOSITIONCONTROLLER:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1100_BLDCMOTOR_120:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_BLDCMOTOR:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1100_TEMPERATURESENSOR_IC_120:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_TEMPERATURESENSOR:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1100_MOTORPOSITIONCONTROLLER_120:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_MOTORPOSITIONCONTROLLER:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DST1000_DISTANCESENSOR_100:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_DISTANCESENSOR:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DST1001_DISTANCESENSOR_100:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_DISTANCESENSOR:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DST1002_DISTANCESENSOR_100:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_DISTANCESENSOR:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DST1200_DISTANCESENSOR_100:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_DISTANCESENSOR:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_ENC1000_ENCODER_100:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_ENCODER:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_HIN1101_ENCODER_100:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_ENCODER:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_HIN1101_DIGITALINPUT_100:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_DIGITALINPUT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_HIN1000_CAPACITIVETOUCH_100:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_CAPACITIVETOUCH:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_HIN1001_CAPACITIVETOUCH_BUTTONS_100:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_CAPACITIVETOUCH:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_HIN1001_CAPACITIVETOUCH_WHEEL_100:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_CAPACITIVETOUCH:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_HIN1100_VOLTAGERATIOINPUT_100:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_VOLTAGERATIOINPUT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_HIN1100_DIGITALINPUT_100:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_DIGITALINPUT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_HUM1000_HUMIDITYSENSOR_100:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_HUMIDITYSENSOR:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_HUM1000_TEMPERATURESENSOR_IC_100:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_TEMPERATURESENSOR:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_HUM1001_HUMIDITYSENSOR_100:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_HUMIDITYSENSOR:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_HUM1001_TEMPERATURESENSOR_IC_100:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_TEMPERATURESENSOR:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_LCD1100_LCD_100:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_LCD:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_LED1000_DIGITALOUTPUT_100:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_DIGITALOUTPUT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_LUX1000_LIGHTSENSOR_100:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_LIGHTSENSOR:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_HUM1100_VOLTAGERATIOINPUT_100:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_VOLTAGERATIOINPUT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_MOT1100_ACCELEROMETER_200:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_ACCELEROMETER:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_MOT1101_ACCELEROMETER_100:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_ACCELEROMETER:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_MOT1101_GYROSCOPE_100:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_GYROSCOPE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_MOT1101_MAGNETOMETER_100:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_MAGNETOMETER:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_MOT1101_SPATIAL_100:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_SPATIAL:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_MOT1102_ACCELEROMETER_200:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_ACCELEROMETER:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_MOT1102_GYROSCOPE_200:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_GYROSCOPE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_MOT1102_MAGNETOMETER_200:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_MAGNETOMETER:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_MOT1102_SPATIAL_200:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_SPATIAL:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_MOT0109_ACCELEROMETER_100:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_ACCELEROMETER:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_MOT0109_GYROSCOPE_100:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_GYROSCOPE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_MOT0109_MAGNETOMETER_100:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_MAGNETOMETER:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_MOT0109_SPATIAL_100:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_SPATIAL:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_MOT0109_TEMPERATURESENSOR_100:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_TEMPERATURESENSOR:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_PRE1000_PRESSURESENSOR_100:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_PRESSURESENSOR:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_RCC1000_RCSERVO_100:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_RCSERVO:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_RCC1000_RCSERVO_110:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_RCSERVO:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_REL1000_DIGITALOUTPUT_100:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_DIGITALOUTPUT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_REL1000_DIGITALOUTPUT_110:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_DIGITALOUTPUT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_REL1100_DIGITALOUTPUT_100:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_DIGITALOUTPUT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_REL1100_DIGITALOUTPUT_110:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_DIGITALOUTPUT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_REL1100_DIGITALOUTPUT_120:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_DIGITALOUTPUT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_REL1101_DIGITALOUTPUT_100:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_DIGITALOUTPUT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_REL1101_DIGITALOUTPUT_110:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_DIGITALOUTPUT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_REL1101_1_DIGITALOUTPUT_FREQ_100:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_DIGITALOUTPUT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_REL1101_1_DIGITALOUTPUT_100:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_DIGITALOUTPUT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_SAF1000_POWERGUARD_100:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_POWERGUARD:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_SAF1000_VOLTAGEINPUT_100:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_VOLTAGEINPUT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_SAF1000_TEMPERATURESENSOR_IC_100:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_TEMPERATURESENSOR:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_SAF1000_POWERGUARD_110:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_POWERGUARD:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_SAF1000_VOLTAGEINPUT_110:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_VOLTAGEINPUT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_SAF1000_TEMPERATURESENSOR_IC_110:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_TEMPERATURESENSOR:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_SND1000_SOUNDSENSOR_100:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_SOUNDSENSOR:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_STC1000_STEPPER_100:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_STEPPER:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_STC1000_STEPPER_110:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_STEPPER:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_STC1001_STEPPER_100:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_STEPPER:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_STC1001_STEPPER_110:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_STEPPER:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_STC1002_STEPPER_100:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_STEPPER:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_STC1002_STEPPER_110:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_STEPPER:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_STC1003_STEPPER_100:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_STEPPER:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_STC1003_STEPPER_110:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_STEPPER:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_STC1003_STEPPER_200:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_STEPPER:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_STC1003_VOLTAGEINPUT_200:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_VOLTAGEINPUT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_TMP1000_TEMPERATURESENSOR_IC_100:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_TEMPERATURESENSOR:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_TMP1100_TEMPERATURESENSOR_THERMOCOUPLE_100:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_TEMPERATURESENSOR:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_TMP1100_TEMPERATURESENSOR_IC_100:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_TEMPERATURESENSOR:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_TMP1100_VOLTAGEINPUT_100:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_VOLTAGEINPUT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_TMP1101_TEMPERATURESENSOR_THERMOCOUPLE_100:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_TEMPERATURESENSOR:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_TMP1101_TEMPERATURESENSOR_IC_100:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_TEMPERATURESENSOR:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_TMP1101_VOLTAGEINPUT_100:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_VOLTAGEINPUT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_TMP1101_TEMPERATURESENSOR_THERMOCOUPLE_200:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_TEMPERATURESENSOR:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_TMP1101_TEMPERATURESENSOR_IC_200:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_TEMPERATURESENSOR:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_TMP1101_VOLTAGEINPUT_200:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_VOLTAGEINPUT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_TMP1200_TEMPERATURESENSOR_RTD_100:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_TEMPERATURESENSOR:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_TMP1200_RESISTANCEINPUT_100:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_RESISTANCEINPUT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_VCP1000_VOLTAGEINPUT_100:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_VOLTAGEINPUT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_VCP1001_VOLTAGEINPUT_100:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_VOLTAGEINPUT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_VCP1001_VOLTAGEINPUT_110:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_VOLTAGEINPUT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_VCP1002_VOLTAGEINPUT_100:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_VOLTAGEINPUT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_VCP1002_VOLTAGEINPUT_110:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_VOLTAGEINPUT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_HUB0000_HUB_100:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_HUB:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_HUB0000_HUB_300:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_HUB:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_HUB0001_HUB_100:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_HUB:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_HUB0004_HUB_100:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_HUB:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_HUB0004_HUB_200:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_HUB:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_HUB5000_HUB_100:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_HUB:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_HUB5000_HUB_200:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_HUB:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_M3_USB_FIRMWARE_UPGRADE_000:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_FIRMWAREUPGRADE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_STM32_USB_FIRMWARE_UPGRADE_100:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_FIRMWAREUPGRADE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_STM32F0_FIRMWARE_UPGRADE_100:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_FIRMWAREUPGRADE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_STM32G0_FIRMWARE_UPGRADE_110:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_FIRMWAREUPGRADE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_STM8S_FIRMWARE_UPGRADE_100:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_FIRMWAREUPGRADE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_M3_SPI_FIRMWARE_UPGRADE_000:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_FIRMWAREUPGRADE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DICTIONARY:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_DICTIONARY:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_USB_UNKNOWN:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_GENERIC:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_VINT_UNKNOWN:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_GENERIC:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_SPI_UNKNOWN:
		switch (val) {
		case PHIDCHCLASS_NOTHING:
		case PHIDCHCLASS_GENERIC:
			return (1);
		default:
			return (0);
		}
	default:
		return (0);
	}
}

int
supportedChannelSubclass(PhidgetChannelHandle ch, Phidget_ChannelSubclass val) {

	switch (ch->UCD->uid) {
	case PHIDCHUID_ifkit488_VOLTAGERATIOINPUT_000:
		switch (val) {
		case PHIDCHSUBCLASS_VOLTAGERATIOINPUT_SENSOR_PORT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_ifkit488_DIGITALINPUT_000:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_ifkit488_DIGITALOUTPUT_000:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1000_RCSERVO_OLD1_200:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1000_RCSERVO_OLD2_200:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1000_RCSERVO_300:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1000_RCSERVO_313:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1001_RCSERVO_OLD1_200:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1001_RCSERVO_OLD2_200:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1001_RCSERVO_313:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1001_RCSERVO_400:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1002_VOLTAGEOUTPUT_100:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1008_ACCELEROMETER_000:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1011_VOLTAGEINPUT_000:
		switch (val) {
		case PHIDCHSUBCLASS_VOLTAGEINPUT_SENSOR_PORT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1011_VOLTAGERATIOINPUT_000:
		switch (val) {
		case PHIDCHSUBCLASS_VOLTAGERATIOINPUT_SENSOR_PORT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1011_DIGITALINPUT_000:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1011_DIGITALOUTPUT_000:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1012_DIGITALINPUT_000:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1012_DIGITALOUTPUT_000:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1012_DIGITALINPUT_601:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1012_DIGITALOUTPUT_601:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1012_DIGITALINPUT_602:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1012_DIGITALOUTPUT_602:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1013_VOLTAGEINPUT_000:
		switch (val) {
		case PHIDCHSUBCLASS_VOLTAGEINPUT_SENSOR_PORT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1013_VOLTAGERATIOINPUT_000:
		switch (val) {
		case PHIDCHSUBCLASS_VOLTAGERATIOINPUT_SENSOR_PORT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1013_DIGITALINPUT_000:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1013_DIGITALOUTPUT_000:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1018_VOLTAGEINPUT_821:
		switch (val) {
		case PHIDCHSUBCLASS_VOLTAGEINPUT_SENSOR_PORT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1018_VOLTAGERATIOINPUT_821:
		switch (val) {
		case PHIDCHSUBCLASS_VOLTAGERATIOINPUT_SENSOR_PORT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1018_DIGITALINPUT_821:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1018_DIGITALOUTPUT_821:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1014_DIGITALOUTPUT_000:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1014_DIGITALOUTPUT_704:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1015_CAPACITIVETOUCH_000:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1016_CAPACITIVETOUCH_000:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1017_DIGITALOUTPUT_000:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1018_VOLTAGEINPUT_900:
		switch (val) {
		case PHIDCHSUBCLASS_VOLTAGEINPUT_SENSOR_PORT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1018_VOLTAGERATIOINPUT_900:
		switch (val) {
		case PHIDCHSUBCLASS_VOLTAGERATIOINPUT_SENSOR_PORT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1018_DIGITALINPUT_900:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1018_DIGITALOUTPUT_900:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1018_VOLTAGEINPUT_1000:
		switch (val) {
		case PHIDCHSUBCLASS_VOLTAGEINPUT_SENSOR_PORT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1018_VOLTAGERATIOINPUT_1000:
		switch (val) {
		case PHIDCHSUBCLASS_VOLTAGERATIOINPUT_SENSOR_PORT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1018_DIGITALINPUT_1000:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1018_DIGITALOUTPUT_1000:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1023_RFID_000:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1023_RFID_104:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1023_RFID_200:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1023_DIGITALOUTPUT_5V_200:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1023_DIGITALOUTPUT_LED_200:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1023_DIGITALOUTPUT_ONBOARD_LED_200:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1023_RFID_201:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1023_DIGITALOUTPUT_5V_201:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1023_DIGITALOUTPUT_LED_201:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1023_DIGITALOUTPUT_ONBOARD_LED_201:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1024_RFID_100:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1024_DIGITALOUTPUT_5V_100:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1024_DIGITALOUTPUT_LED_100:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1024_DIGITALOUTPUT_ONBOARD_LED_100:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1030_DIGITALOUTPUT_100:
		switch (val) {
		case PHIDCHSUBCLASS_DIGITALOUTPUT_LED_DRIVER:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1031_DIGITALOUTPUT_100:
		switch (val) {
		case PHIDCHSUBCLASS_DIGITALOUTPUT_LED_DRIVER:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1032_DIGITALOUTPUT_200:
		switch (val) {
		case PHIDCHSUBCLASS_DIGITALOUTPUT_LED_DRIVER:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1040_GPS_000:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1041_ACCELEROMETER_200:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1042_ACCELEROMETER_300:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1042_GYROSCOPE_300:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1042_MAGNETOMETER_300:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1042_SPATIAL_300:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1043_ACCELEROMETER_300:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1044_ACCELEROMETER_400:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1044_GYROSCOPE_400:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1044_MAGNETOMETER_400:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1044_SPATIAL_400:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1044_ACCELEROMETER_500:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1044_GYROSCOPE_500:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1044_MAGNETOMETER_500:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1044_SPATIAL_500:
		switch (val) {
		case PHIDCHSUBCLASS_SPATIAL_AHRS:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1044_ACCELEROMETER_510:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1044_GYROSCOPE_510:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1044_MAGNETOMETER_510:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1044_SPATIAL_510:
		switch (val) {
		case PHIDCHSUBCLASS_SPATIAL_AHRS:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1045_TEMPERATURESENSOR_IR_100:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1045_TEMPERATURESENSOR_IC_100:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1046_VOLTAGERATIOINPUT_100:
		switch (val) {
		case PHIDCHSUBCLASS_VOLTAGERATIOINPUT_BRIDGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1046_VOLTAGERATIOINPUT_102:
		switch (val) {
		case PHIDCHSUBCLASS_VOLTAGERATIOINPUT_BRIDGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1047_ENCODER_100:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1047_DIGITALINPUT_100:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1047_ENCODER_200:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1047_DIGITALINPUT_200:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1048_TEMPERATURESENSOR_THERMOCOUPLE_100:
		switch (val) {
		case PHIDCHSUBCLASS_TEMPERATURESENSOR_THERMOCOUPLE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1048_TEMPERATURESENSOR_IC_100:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1048_VOLTAGEINPUT_100:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1048_TEMPERATURESENSOR_THERMOCOUPLE_200:
		switch (val) {
		case PHIDCHSUBCLASS_TEMPERATURESENSOR_THERMOCOUPLE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1048_TEMPERATURESENSOR_IC_200:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1048_VOLTAGEINPUT_200:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1049_ACCELEROMETER_000:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1051_TEMPERATURESENSOR_THERMOCOUPLE_000:
		switch (val) {
		case PHIDCHSUBCLASS_TEMPERATURESENSOR_THERMOCOUPLE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1051_TEMPERATURESENSOR_IC_000:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1051_TEMPERATURESENSOR_THERMOCOUPLE_200:
		switch (val) {
		case PHIDCHSUBCLASS_TEMPERATURESENSOR_THERMOCOUPLE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1051_TEMPERATURESENSOR_IC_200:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1051_VOLTAGEINPUT_200:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1051_TEMPERATURESENSOR_THERMOCOUPLE_300:
		switch (val) {
		case PHIDCHSUBCLASS_TEMPERATURESENSOR_THERMOCOUPLE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1051_TEMPERATURESENSOR_IC_300:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1051_VOLTAGEINPUT_300:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1051_TEMPERATURESENSOR_THERMOCOUPLE_400:
		switch (val) {
		case PHIDCHSUBCLASS_TEMPERATURESENSOR_THERMOCOUPLE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1051_TEMPERATURESENSOR_IC_400:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1051_VOLTAGEINPUT_400:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1052_ENCODER_000:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1052_DIGITALINPUT_000:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1052_ENCODER_101:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1052_DIGITALINPUT_101:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1052_ENCODER_110:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1052_DIGITALINPUT_110:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1053_ACCELEROMETER_300:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1054_FREQUENCYCOUNTER_000:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1055_IR_100:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1055_IR_200_USB:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1055_IR_200_VINT:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1056_ACCELEROMETER_000:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1056_GYROSCOPE_000:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1056_MAGNETOMETER_000:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1056_SPATIAL_000:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1056_ACCELEROMETER_200:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1056_GYROSCOPE_200:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1056_MAGNETOMETER_200:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1056_SPATIAL_200:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1057_ENCODER_300:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1057_ENCODER_400:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1058_VOLTAGEINPUT_100:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1058_PHADAPTER_100:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1059_ACCELEROMETER_400:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1060_DCMOTOR_100:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1060_DIGITALINPUT_100:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1061_RCSERVO_100:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1061_CURRENTINPUT_100:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1061_RCSERVO_200:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1061_CURRENTINPUT_200:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1061_RCSERVO_300:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1061_CURRENTINPUT_300:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_RCC0004_RCSERVO_400:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1062_STEPPER_100:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1063_STEPPER_100:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1063_DIGITALINPUT_100:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1063_CURRENTINPUT_100:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1064_DCMOTOR_100:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1064_CURRENTINPUT_100:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1065_DCMOTOR_100:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1065_DIGITALINPUT_100:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1065_ENCODER_100:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1065_VOLTAGEINPUT_100:
		switch (val) {
		case PHIDCHSUBCLASS_VOLTAGEINPUT_SENSOR_PORT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1065_VOLTAGEINPUT_SUPPLY_100:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1065_VOLTAGERATIOINPUT_100:
		switch (val) {
		case PHIDCHSUBCLASS_VOLTAGERATIOINPUT_SENSOR_PORT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1065_CURRENTINPUT_100:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1066_RCSERVO_100:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1066_CURRENTINPUT_100:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1067_STEPPER_200:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1067_STEPPER_300:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1202_VOLTAGEINPUT_000:
		switch (val) {
		case PHIDCHSUBCLASS_VOLTAGEINPUT_SENSOR_PORT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1202_VOLTAGERATIOINPUT_000:
		switch (val) {
		case PHIDCHSUBCLASS_VOLTAGERATIOINPUT_SENSOR_PORT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1202_DIGITALINPUT_000:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1202_DIGITALOUTPUT_000:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1202_VOLTAGEINPUT_120:
		switch (val) {
		case PHIDCHSUBCLASS_VOLTAGEINPUT_SENSOR_PORT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1202_VOLTAGERATIOINPUT_120:
		switch (val) {
		case PHIDCHSUBCLASS_VOLTAGERATIOINPUT_SENSOR_PORT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1202_DIGITALINPUT_120:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1202_DIGITALOUTPUT_120:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1202_TEXTLCD_000:
		switch (val) {
		case PHIDCHSUBCLASS_LCD_TEXT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1202_VOLTAGEINPUT_300:
		switch (val) {
		case PHIDCHSUBCLASS_VOLTAGEINPUT_SENSOR_PORT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1202_VOLTAGERATIOINPUT_300:
		switch (val) {
		case PHIDCHSUBCLASS_VOLTAGERATIOINPUT_SENSOR_PORT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1202_DIGITALINPUT_300:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1202_DIGITALOUTPUT_300:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1202_TEXTLCD_200:
		switch (val) {
		case PHIDCHSUBCLASS_LCD_TEXT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1204_TEXTLCD_000:
		switch (val) {
		case PHIDCHSUBCLASS_LCD_TEXT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1215_TEXTLCD_000:
		switch (val) {
		case PHIDCHSUBCLASS_LCD_TEXT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1219_TEXTLCD_000:
		switch (val) {
		case PHIDCHSUBCLASS_LCD_TEXT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1219_DIGITALINPUT_000:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1219_DIGITALOUTPUT_000:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_HUB_DIGITALINPUT_100:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_HUB_DIGITALOUTPUT_100:
		switch (val) {
		case PHIDCHSUBCLASS_DIGITALOUTPUT_DUTY_CYCLE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_HUB_DIGITALOUTPUT_110:
		switch (val) {
		case PHIDCHSUBCLASS_DIGITALOUTPUT_DUTY_CYCLE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_HUB_VOLTAGEINPUT_100:
		switch (val) {
		case PHIDCHSUBCLASS_VOLTAGEINPUT_SENSOR_PORT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_HUB_VOLTAGEINPUT_110:
		switch (val) {
		case PHIDCHSUBCLASS_VOLTAGEINPUT_SENSOR_PORT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_HUB_VOLTAGERATIOINPUT_100:
		switch (val) {
		case PHIDCHSUBCLASS_VOLTAGERATIOINPUT_SENSOR_PORT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_ADP1000_PHSENSOR_100:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_ADP1000_VOLTAGEINPUT_100:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DAQ1000_VOLTAGERATIOINPUT_100:
		switch (val) {
		case PHIDCHSUBCLASS_VOLTAGERATIOINPUT_SENSOR_PORT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DAQ1000_VOLTAGEINPUT_100:
		switch (val) {
		case PHIDCHSUBCLASS_VOLTAGEINPUT_SENSOR_PORT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DAQ1000_VOLTAGERATIOINPUT_110:
		switch (val) {
		case PHIDCHSUBCLASS_VOLTAGERATIOINPUT_SENSOR_PORT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DAQ1000_VOLTAGEINPUT_110:
		switch (val) {
		case PHIDCHSUBCLASS_VOLTAGEINPUT_SENSOR_PORT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_OUT1000_VOLTAGEOUTPUT_100:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_OUT1000_VOLTAGEOUTPUT_110:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_OUT1001_VOLTAGEOUTPUT_100:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_OUT1001_VOLTAGEOUTPUT_110:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_OUT1002_VOLTAGEOUTPUT_100:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_OUT1002_VOLTAGEOUTPUT_110:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DAQ1200_DIGITALINPUT_100:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_OUT1100_DIGITALOUTPUT_100:
		switch (val) {
		case PHIDCHSUBCLASS_DIGITALOUTPUT_DUTY_CYCLE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_OUT1100_DIGITALOUTPUT_110:
		switch (val) {
		case PHIDCHSUBCLASS_DIGITALOUTPUT_DUTY_CYCLE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_OUT1100_DIGITALOUTPUT_120:
		switch (val) {
		case PHIDCHSUBCLASS_DIGITALOUTPUT_FREQUENCY:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DAQ1300_DIGITALINPUT_100:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DAQ1301_DIGITALINPUT_100:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DAQ1400_VOLTAGEINPUT_100:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DAQ1400_CURRENTINPUT_100:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DAQ1400_DIGITALINPUT_100:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DAQ1400_FREQUENCYCOUNTER_100:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DAQ1400_VOLTAGEINPUT_120:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DAQ1400_CURRENTINPUT_120:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DAQ1400_DIGITALINPUT_120:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DAQ1400_FREQUENCYCOUNTER_120:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DAQ1500_VOLTAGERATIOINPUT_100:
		switch (val) {
		case PHIDCHSUBCLASS_VOLTAGERATIOINPUT_BRIDGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_VCP1100_CURRENTINPUT_100:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_VCP1100_CURRENTINPUT_200:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1000_DCMOTOR_100:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1000_ENCODER_100:
		switch (val) {
		case PHIDCHSUBCLASS_ENCODER_MODE_SETTABLE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1000_VOLTAGERATIOINPUT_100:
		switch (val) {
		case PHIDCHSUBCLASS_VOLTAGERATIOINPUT_SENSOR_PORT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1000_TEMPERATURESENSOR_IC_100:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1000_CURRENTINPUT_100:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1000_DCMOTOR_200:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1000_ENCODER_200:
		switch (val) {
		case PHIDCHSUBCLASS_ENCODER_MODE_SETTABLE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1000_VOLTAGERATIOINPUT_200:
		switch (val) {
		case PHIDCHSUBCLASS_VOLTAGERATIOINPUT_SENSOR_PORT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1000_TEMPERATURESENSOR_IC_200:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1000_CURRENTINPUT_200:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1000_MOTORPOSITIONCONTROLLER_200:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1000_DCMOTOR_210:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1000_ENCODER_210:
		switch (val) {
		case PHIDCHSUBCLASS_ENCODER_MODE_SETTABLE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1000_VOLTAGERATIOINPUT_210:
		switch (val) {
		case PHIDCHSUBCLASS_VOLTAGERATIOINPUT_SENSOR_PORT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1000_TEMPERATURESENSOR_IC_210:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1000_CURRENTINPUT_210:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1000_MOTORPOSITIONCONTROLLER_210:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1001_DCMOTOR_100:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1001_ENCODER_100:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1001_MOTORPOSITIONCONTROLLER_100:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1001_DCMOTOR_120:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1001_ENCODER_120:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1001_MOTORPOSITIONCONTROLLER_120:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1002_DCMOTOR_100:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1002_ENCODER_100:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1002_MOTORPOSITIONCONTROLLER_100:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1002_DCMOTOR_110:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1002_ENCODER_110:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1002_MOTORPOSITIONCONTROLLER_110:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1003_DCMOTOR_100:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1003_DCMOTOR_110:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1100_BLDCMOTOR_100:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1100_TEMPERATURESENSOR_IC_100:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1100_MOTORPOSITIONCONTROLLER_100:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1100_BLDCMOTOR_120:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1100_TEMPERATURESENSOR_IC_120:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1100_MOTORPOSITIONCONTROLLER_120:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DST1000_DISTANCESENSOR_100:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DST1001_DISTANCESENSOR_100:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DST1002_DISTANCESENSOR_100:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DST1200_DISTANCESENSOR_100:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_ENC1000_ENCODER_100:
		switch (val) {
		case PHIDCHSUBCLASS_ENCODER_MODE_SETTABLE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_HIN1101_ENCODER_100:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_HIN1101_DIGITALINPUT_100:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_HIN1000_CAPACITIVETOUCH_100:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_HIN1001_CAPACITIVETOUCH_BUTTONS_100:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_HIN1001_CAPACITIVETOUCH_WHEEL_100:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_HIN1100_VOLTAGERATIOINPUT_100:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_HIN1100_DIGITALINPUT_100:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_HUM1000_HUMIDITYSENSOR_100:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_HUM1000_TEMPERATURESENSOR_IC_100:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_HUM1001_HUMIDITYSENSOR_100:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_HUM1001_TEMPERATURESENSOR_IC_100:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_LCD1100_LCD_100:
		switch (val) {
		case PHIDCHSUBCLASS_LCD_GRAPHIC:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_LED1000_DIGITALOUTPUT_100:
		switch (val) {
		case PHIDCHSUBCLASS_DIGITALOUTPUT_LED_DRIVER:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_LUX1000_LIGHTSENSOR_100:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_HUM1100_VOLTAGERATIOINPUT_100:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_MOT1100_ACCELEROMETER_200:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_MOT1101_ACCELEROMETER_100:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_MOT1101_GYROSCOPE_100:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_MOT1101_MAGNETOMETER_100:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_MOT1101_SPATIAL_100:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_MOT1102_ACCELEROMETER_200:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_MOT1102_GYROSCOPE_200:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_MOT1102_MAGNETOMETER_200:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_MOT1102_SPATIAL_200:
		switch (val) {
		case PHIDCHSUBCLASS_SPATIAL_AHRS:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_MOT0109_ACCELEROMETER_100:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_MOT0109_GYROSCOPE_100:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_MOT0109_MAGNETOMETER_100:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_MOT0109_SPATIAL_100:
		switch (val) {
		case PHIDCHSUBCLASS_SPATIAL_AHRS:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_MOT0109_TEMPERATURESENSOR_100:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_PRE1000_PRESSURESENSOR_100:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_RCC1000_RCSERVO_100:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_RCC1000_RCSERVO_110:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_REL1000_DIGITALOUTPUT_100:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_REL1000_DIGITALOUTPUT_110:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_REL1100_DIGITALOUTPUT_100:
		switch (val) {
		case PHIDCHSUBCLASS_DIGITALOUTPUT_DUTY_CYCLE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_REL1100_DIGITALOUTPUT_110:
		switch (val) {
		case PHIDCHSUBCLASS_DIGITALOUTPUT_DUTY_CYCLE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_REL1100_DIGITALOUTPUT_120:
		switch (val) {
		case PHIDCHSUBCLASS_DIGITALOUTPUT_FREQUENCY:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_REL1101_DIGITALOUTPUT_100:
		switch (val) {
		case PHIDCHSUBCLASS_DIGITALOUTPUT_DUTY_CYCLE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_REL1101_DIGITALOUTPUT_110:
		switch (val) {
		case PHIDCHSUBCLASS_DIGITALOUTPUT_DUTY_CYCLE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_REL1101_1_DIGITALOUTPUT_FREQ_100:
		switch (val) {
		case PHIDCHSUBCLASS_DIGITALOUTPUT_FREQUENCY:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_REL1101_1_DIGITALOUTPUT_100:
		switch (val) {
		case PHIDCHSUBCLASS_DIGITALOUTPUT_DUTY_CYCLE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_SAF1000_POWERGUARD_100:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_SAF1000_VOLTAGEINPUT_100:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_SAF1000_TEMPERATURESENSOR_IC_100:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_SAF1000_POWERGUARD_110:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_SAF1000_VOLTAGEINPUT_110:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_SAF1000_TEMPERATURESENSOR_IC_110:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_SND1000_SOUNDSENSOR_100:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_STC1000_STEPPER_100:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_STC1000_STEPPER_110:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_STC1001_STEPPER_100:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_STC1001_STEPPER_110:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_STC1002_STEPPER_100:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_STC1002_STEPPER_110:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_STC1003_STEPPER_100:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_STC1003_STEPPER_110:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_STC1003_STEPPER_200:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_STC1003_VOLTAGEINPUT_200:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_TMP1000_TEMPERATURESENSOR_IC_100:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_TMP1100_TEMPERATURESENSOR_THERMOCOUPLE_100:
		switch (val) {
		case PHIDCHSUBCLASS_TEMPERATURESENSOR_THERMOCOUPLE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_TMP1100_TEMPERATURESENSOR_IC_100:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_TMP1100_VOLTAGEINPUT_100:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_TMP1101_TEMPERATURESENSOR_THERMOCOUPLE_100:
		switch (val) {
		case PHIDCHSUBCLASS_TEMPERATURESENSOR_THERMOCOUPLE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_TMP1101_TEMPERATURESENSOR_IC_100:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_TMP1101_VOLTAGEINPUT_100:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_TMP1101_TEMPERATURESENSOR_THERMOCOUPLE_200:
		switch (val) {
		case PHIDCHSUBCLASS_TEMPERATURESENSOR_THERMOCOUPLE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_TMP1101_TEMPERATURESENSOR_IC_200:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_TMP1101_VOLTAGEINPUT_200:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_TMP1200_TEMPERATURESENSOR_RTD_100:
		switch (val) {
		case PHIDCHSUBCLASS_TEMPERATURESENSOR_RTD:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_TMP1200_RESISTANCEINPUT_100:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_VCP1000_VOLTAGEINPUT_100:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_VCP1001_VOLTAGEINPUT_100:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_VCP1001_VOLTAGEINPUT_110:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_VCP1002_VOLTAGEINPUT_100:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_VCP1002_VOLTAGEINPUT_110:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_HUB0000_HUB_100:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_HUB0000_HUB_300:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_HUB0001_HUB_100:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_HUB0004_HUB_100:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_HUB0004_HUB_200:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_HUB5000_HUB_100:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_HUB5000_HUB_200:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_M3_USB_FIRMWARE_UPGRADE_000:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_STM32_USB_FIRMWARE_UPGRADE_100:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_STM32F0_FIRMWARE_UPGRADE_100:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_STM32G0_FIRMWARE_UPGRADE_110:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_STM8S_FIRMWARE_UPGRADE_100:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_M3_SPI_FIRMWARE_UPGRADE_000:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DICTIONARY:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_USB_UNKNOWN:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_VINT_UNKNOWN:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_SPI_UNKNOWN:
		switch (val) {
		case PHIDCHSUBCLASS_NONE:
			return (1);
		default:
			return (0);
		}
	default:
		return (0);
	}
}

int
supportedMeshMode(PhidgetChannelHandle ch, Phidget_MeshMode val) {

	switch (ch->UCD->uid) {
	default:
		return (0);
	}
}

int
supportedPowerSupply(PhidgetChannelHandle ch, Phidget_PowerSupply val) {

	switch (ch->UCD->uid) {
	case PHIDCHUID_DAQ1400_VOLTAGEINPUT_100:
		switch (val) {
		case POWER_SUPPLY_OFF:
		case POWER_SUPPLY_12V:
		case POWER_SUPPLY_24V:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DAQ1400_CURRENTINPUT_100:
		switch (val) {
		case POWER_SUPPLY_OFF:
		case POWER_SUPPLY_12V:
		case POWER_SUPPLY_24V:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DAQ1400_DIGITALINPUT_100:
		switch (val) {
		case POWER_SUPPLY_OFF:
		case POWER_SUPPLY_12V:
		case POWER_SUPPLY_24V:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DAQ1400_FREQUENCYCOUNTER_100:
		switch (val) {
		case POWER_SUPPLY_OFF:
		case POWER_SUPPLY_12V:
		case POWER_SUPPLY_24V:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DAQ1400_VOLTAGEINPUT_120:
		switch (val) {
		case POWER_SUPPLY_OFF:
		case POWER_SUPPLY_12V:
		case POWER_SUPPLY_24V:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DAQ1400_CURRENTINPUT_120:
		switch (val) {
		case POWER_SUPPLY_OFF:
		case POWER_SUPPLY_12V:
		case POWER_SUPPLY_24V:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DAQ1400_DIGITALINPUT_120:
		switch (val) {
		case POWER_SUPPLY_OFF:
		case POWER_SUPPLY_12V:
		case POWER_SUPPLY_24V:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DAQ1400_FREQUENCYCOUNTER_120:
		switch (val) {
		case POWER_SUPPLY_OFF:
		case POWER_SUPPLY_12V:
		case POWER_SUPPLY_24V:
			return (1);
		default:
			return (0);
		}
	default:
		return (0);
	}
}

int
supportedRTDWireSetup(PhidgetChannelHandle ch, Phidget_RTDWireSetup val) {

	switch (ch->UCD->uid) {
	case PHIDCHUID_TMP1200_TEMPERATURESENSOR_RTD_100:
		switch (val) {
		case RTD_WIRE_SETUP_2WIRE:
		case RTD_WIRE_SETUP_3WIRE:
		case RTD_WIRE_SETUP_4WIRE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_TMP1200_RESISTANCEINPUT_100:
		switch (val) {
		case RTD_WIRE_SETUP_2WIRE:
		case RTD_WIRE_SETUP_3WIRE:
		case RTD_WIRE_SETUP_4WIRE:
			return (1);
		default:
			return (0);
		}
	default:
		return (0);
	}
}

int
supportedInputMode(PhidgetChannelHandle ch, Phidget_InputMode val) {

	switch (ch->UCD->uid) {
	case PHIDCHUID_DAQ1400_DIGITALINPUT_100:
		switch (val) {
		case INPUT_MODE_NPN:
		case INPUT_MODE_PNP:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DAQ1400_FREQUENCYCOUNTER_100:
		switch (val) {
		case INPUT_MODE_NPN:
		case INPUT_MODE_PNP:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DAQ1400_DIGITALINPUT_120:
		switch (val) {
		case INPUT_MODE_NPN:
		case INPUT_MODE_PNP:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DAQ1400_FREQUENCYCOUNTER_120:
		switch (val) {
		case INPUT_MODE_NPN:
		case INPUT_MODE_PNP:
			return (1);
		default:
			return (0);
		}
	default:
		return (0);
	}
}

int
supportedFanMode(PhidgetChannelHandle ch, Phidget_FanMode val) {

	switch (ch->UCD->uid) {
	case PHIDCHUID_DCC1000_DCMOTOR_100:
		switch (val) {
		case FAN_MODE_OFF:
		case FAN_MODE_ON:
		case FAN_MODE_AUTO:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1000_DCMOTOR_200:
		switch (val) {
		case FAN_MODE_OFF:
		case FAN_MODE_ON:
		case FAN_MODE_AUTO:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1000_MOTORPOSITIONCONTROLLER_200:
		switch (val) {
		case FAN_MODE_OFF:
		case FAN_MODE_ON:
		case FAN_MODE_AUTO:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1000_DCMOTOR_210:
		switch (val) {
		case FAN_MODE_OFF:
		case FAN_MODE_ON:
		case FAN_MODE_AUTO:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1000_MOTORPOSITIONCONTROLLER_210:
		switch (val) {
		case FAN_MODE_OFF:
		case FAN_MODE_ON:
		case FAN_MODE_AUTO:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_SAF1000_POWERGUARD_100:
		switch (val) {
		case FAN_MODE_OFF:
		case FAN_MODE_ON:
		case FAN_MODE_AUTO:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_SAF1000_POWERGUARD_110:
		switch (val) {
		case FAN_MODE_OFF:
		case FAN_MODE_ON:
		case FAN_MODE_AUTO:
			return (1);
		default:
			return (0);
		}
	default:
		return (0);
	}
}

int
supportedSpatialPrecision(PhidgetChannelHandle ch, Phidget_SpatialPrecision val) {

	switch (ch->UCD->uid) {
	case PHIDCHUID_1041_ACCELEROMETER_200:
		switch (val) {
		case SPATIAL_PRECISION_HYBRID:
		case SPATIAL_PRECISION_HIGH:
		case SPATIAL_PRECISION_LOW:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1042_ACCELEROMETER_300:
		switch (val) {
		case SPATIAL_PRECISION_HYBRID:
		case SPATIAL_PRECISION_HIGH:
		case SPATIAL_PRECISION_LOW:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1042_GYROSCOPE_300:
		switch (val) {
		case SPATIAL_PRECISION_HYBRID:
		case SPATIAL_PRECISION_HIGH:
		case SPATIAL_PRECISION_LOW:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1042_SPATIAL_300:
		switch (val) {
		case SPATIAL_PRECISION_HYBRID:
		case SPATIAL_PRECISION_HIGH:
		case SPATIAL_PRECISION_LOW:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1043_ACCELEROMETER_300:
		switch (val) {
		case SPATIAL_PRECISION_HYBRID:
		case SPATIAL_PRECISION_HIGH:
		case SPATIAL_PRECISION_LOW:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1044_ACCELEROMETER_400:
		switch (val) {
		case SPATIAL_PRECISION_HYBRID:
		case SPATIAL_PRECISION_HIGH:
		case SPATIAL_PRECISION_LOW:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1044_GYROSCOPE_400:
		switch (val) {
		case SPATIAL_PRECISION_HYBRID:
		case SPATIAL_PRECISION_HIGH:
		case SPATIAL_PRECISION_LOW:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1044_SPATIAL_400:
		switch (val) {
		case SPATIAL_PRECISION_HYBRID:
		case SPATIAL_PRECISION_HIGH:
		case SPATIAL_PRECISION_LOW:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1044_ACCELEROMETER_500:
		switch (val) {
		case SPATIAL_PRECISION_HYBRID:
		case SPATIAL_PRECISION_HIGH:
		case SPATIAL_PRECISION_LOW:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1044_GYROSCOPE_500:
		switch (val) {
		case SPATIAL_PRECISION_HYBRID:
		case SPATIAL_PRECISION_HIGH:
		case SPATIAL_PRECISION_LOW:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1044_SPATIAL_500:
		switch (val) {
		case SPATIAL_PRECISION_HYBRID:
		case SPATIAL_PRECISION_HIGH:
		case SPATIAL_PRECISION_LOW:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1044_ACCELEROMETER_510:
		switch (val) {
		case SPATIAL_PRECISION_HYBRID:
		case SPATIAL_PRECISION_HIGH:
		case SPATIAL_PRECISION_LOW:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1044_GYROSCOPE_510:
		switch (val) {
		case SPATIAL_PRECISION_HYBRID:
		case SPATIAL_PRECISION_HIGH:
		case SPATIAL_PRECISION_LOW:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1044_SPATIAL_510:
		switch (val) {
		case SPATIAL_PRECISION_HYBRID:
		case SPATIAL_PRECISION_HIGH:
		case SPATIAL_PRECISION_LOW:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1056_ACCELEROMETER_000:
		switch (val) {
		case SPATIAL_PRECISION_HYBRID:
		case SPATIAL_PRECISION_HIGH:
		case SPATIAL_PRECISION_LOW:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1056_GYROSCOPE_000:
		switch (val) {
		case SPATIAL_PRECISION_HYBRID:
		case SPATIAL_PRECISION_HIGH:
		case SPATIAL_PRECISION_LOW:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1056_SPATIAL_000:
		switch (val) {
		case SPATIAL_PRECISION_HYBRID:
		case SPATIAL_PRECISION_HIGH:
		case SPATIAL_PRECISION_LOW:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1056_ACCELEROMETER_200:
		switch (val) {
		case SPATIAL_PRECISION_HYBRID:
		case SPATIAL_PRECISION_HIGH:
		case SPATIAL_PRECISION_LOW:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1056_GYROSCOPE_200:
		switch (val) {
		case SPATIAL_PRECISION_HYBRID:
		case SPATIAL_PRECISION_HIGH:
		case SPATIAL_PRECISION_LOW:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1056_SPATIAL_200:
		switch (val) {
		case SPATIAL_PRECISION_HYBRID:
		case SPATIAL_PRECISION_HIGH:
		case SPATIAL_PRECISION_LOW:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_MOT1101_ACCELEROMETER_100:
		switch (val) {
		case SPATIAL_PRECISION_HYBRID:
		case SPATIAL_PRECISION_HIGH:
		case SPATIAL_PRECISION_LOW:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_MOT1101_GYROSCOPE_100:
		switch (val) {
		case SPATIAL_PRECISION_HYBRID:
		case SPATIAL_PRECISION_HIGH:
		case SPATIAL_PRECISION_LOW:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_MOT1101_SPATIAL_100:
		switch (val) {
		case SPATIAL_PRECISION_HYBRID:
		case SPATIAL_PRECISION_HIGH:
		case SPATIAL_PRECISION_LOW:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_MOT1102_ACCELEROMETER_200:
		switch (val) {
		case SPATIAL_PRECISION_HYBRID:
		case SPATIAL_PRECISION_HIGH:
		case SPATIAL_PRECISION_LOW:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_MOT1102_GYROSCOPE_200:
		switch (val) {
		case SPATIAL_PRECISION_HYBRID:
		case SPATIAL_PRECISION_HIGH:
		case SPATIAL_PRECISION_LOW:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_MOT1102_SPATIAL_200:
		switch (val) {
		case SPATIAL_PRECISION_HYBRID:
		case SPATIAL_PRECISION_HIGH:
		case SPATIAL_PRECISION_LOW:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_MOT0109_ACCELEROMETER_100:
		switch (val) {
		case SPATIAL_PRECISION_HYBRID:
		case SPATIAL_PRECISION_HIGH:
		case SPATIAL_PRECISION_LOW:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_MOT0109_GYROSCOPE_100:
		switch (val) {
		case SPATIAL_PRECISION_HYBRID:
		case SPATIAL_PRECISION_HIGH:
		case SPATIAL_PRECISION_LOW:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_MOT0109_SPATIAL_100:
		switch (val) {
		case SPATIAL_PRECISION_HYBRID:
		case SPATIAL_PRECISION_HIGH:
		case SPATIAL_PRECISION_LOW:
			return (1);
		default:
			return (0);
		}
	default:
		return (0);
	}
}

int
supportedUnit(PhidgetChannelHandle ch, Phidget_Unit val) {

	switch (ch->UCD->uid) {
	default:
		return (0);
	}
}

int
supportedBridgeGain(PhidgetChannelHandle ch, PhidgetVoltageRatioInput_BridgeGain val) {

	switch (ch->UCD->uid) {
	case PHIDCHUID_1046_VOLTAGERATIOINPUT_100:
		switch (val) {
		case BRIDGE_GAIN_1:
		case BRIDGE_GAIN_8:
		case BRIDGE_GAIN_16:
		case BRIDGE_GAIN_32:
		case BRIDGE_GAIN_64:
		case BRIDGE_GAIN_128:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1046_VOLTAGERATIOINPUT_102:
		switch (val) {
		case BRIDGE_GAIN_1:
		case BRIDGE_GAIN_8:
		case BRIDGE_GAIN_16:
		case BRIDGE_GAIN_32:
		case BRIDGE_GAIN_64:
		case BRIDGE_GAIN_128:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DAQ1500_VOLTAGERATIOINPUT_100:
		switch (val) {
		case BRIDGE_GAIN_1:
		case BRIDGE_GAIN_2:
		case BRIDGE_GAIN_64:
		case BRIDGE_GAIN_128:
			return (1);
		default:
			return (0);
		}
	default:
		return (0);
	}
}

int
supportedVoltageRatioSensorType(PhidgetChannelHandle ch, PhidgetVoltageRatioInput_SensorType val) {

	switch (ch->UCD->uid) {
	case PHIDCHUID_ifkit488_VOLTAGERATIOINPUT_000:
		switch (val) {
		case SENSOR_TYPE_VOLTAGERATIO:
		case SENSOR_TYPE_3522:
		case SENSOR_TYPE_1101_SHARP_2Y0A21:
		case SENSOR_TYPE_1101_SHARP_2Y0A02:
		case SENSOR_TYPE_1102:
		case SENSOR_TYPE_1103:
		case SENSOR_TYPE_1104:
		case SENSOR_TYPE_1105:
		case SENSOR_TYPE_1106:
		case SENSOR_TYPE_1107:
		case SENSOR_TYPE_1108:
		case SENSOR_TYPE_1109:
		case SENSOR_TYPE_1110:
		case SENSOR_TYPE_1111:
		case SENSOR_TYPE_1112:
		case SENSOR_TYPE_1113:
		case SENSOR_TYPE_1115:
		case SENSOR_TYPE_1116:
		case SENSOR_TYPE_1118_AC:
		case SENSOR_TYPE_1118_DC:
		case SENSOR_TYPE_1119_AC:
		case SENSOR_TYPE_1119_DC:
		case SENSOR_TYPE_1120:
		case SENSOR_TYPE_1121:
		case SENSOR_TYPE_1101_SHARP_2D120X:
		case SENSOR_TYPE_1122_DC:
		case SENSOR_TYPE_1124:
		case SENSOR_TYPE_1125_HUMIDITY:
		case SENSOR_TYPE_1125_TEMPERATURE:
		case SENSOR_TYPE_1126:
		case SENSOR_TYPE_1128:
		case SENSOR_TYPE_1129:
		case SENSOR_TYPE_1131:
		case SENSOR_TYPE_1134:
		case SENSOR_TYPE_1136:
		case SENSOR_TYPE_1137:
		case SENSOR_TYPE_1138:
		case SENSOR_TYPE_1139:
		case SENSOR_TYPE_1140:
		case SENSOR_TYPE_1141:
		case SENSOR_TYPE_1146:
		case SENSOR_TYPE_3120:
		case SENSOR_TYPE_3121:
		case SENSOR_TYPE_3122:
		case SENSOR_TYPE_3123:
		case SENSOR_TYPE_3130:
		case SENSOR_TYPE_3520:
		case SENSOR_TYPE_3521:
		case SENSOR_TYPE_1122_AC:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1011_VOLTAGERATIOINPUT_000:
		switch (val) {
		case SENSOR_TYPE_VOLTAGERATIO:
		case SENSOR_TYPE_3522:
		case SENSOR_TYPE_1101_SHARP_2Y0A21:
		case SENSOR_TYPE_1101_SHARP_2Y0A02:
		case SENSOR_TYPE_1102:
		case SENSOR_TYPE_1103:
		case SENSOR_TYPE_1104:
		case SENSOR_TYPE_1105:
		case SENSOR_TYPE_1106:
		case SENSOR_TYPE_1107:
		case SENSOR_TYPE_1108:
		case SENSOR_TYPE_1109:
		case SENSOR_TYPE_1110:
		case SENSOR_TYPE_1111:
		case SENSOR_TYPE_1112:
		case SENSOR_TYPE_1113:
		case SENSOR_TYPE_1115:
		case SENSOR_TYPE_1116:
		case SENSOR_TYPE_1118_AC:
		case SENSOR_TYPE_1118_DC:
		case SENSOR_TYPE_1119_AC:
		case SENSOR_TYPE_1119_DC:
		case SENSOR_TYPE_1120:
		case SENSOR_TYPE_1121:
		case SENSOR_TYPE_1101_SHARP_2D120X:
		case SENSOR_TYPE_1122_DC:
		case SENSOR_TYPE_1124:
		case SENSOR_TYPE_1125_HUMIDITY:
		case SENSOR_TYPE_1125_TEMPERATURE:
		case SENSOR_TYPE_1126:
		case SENSOR_TYPE_1128:
		case SENSOR_TYPE_1129:
		case SENSOR_TYPE_1131:
		case SENSOR_TYPE_1134:
		case SENSOR_TYPE_1136:
		case SENSOR_TYPE_1137:
		case SENSOR_TYPE_1138:
		case SENSOR_TYPE_1139:
		case SENSOR_TYPE_1140:
		case SENSOR_TYPE_1141:
		case SENSOR_TYPE_1146:
		case SENSOR_TYPE_3120:
		case SENSOR_TYPE_3121:
		case SENSOR_TYPE_3122:
		case SENSOR_TYPE_3123:
		case SENSOR_TYPE_3130:
		case SENSOR_TYPE_3520:
		case SENSOR_TYPE_3521:
		case SENSOR_TYPE_1122_AC:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1013_VOLTAGERATIOINPUT_000:
		switch (val) {
		case SENSOR_TYPE_VOLTAGERATIO:
		case SENSOR_TYPE_3522:
		case SENSOR_TYPE_1101_SHARP_2Y0A21:
		case SENSOR_TYPE_1101_SHARP_2Y0A02:
		case SENSOR_TYPE_1102:
		case SENSOR_TYPE_1103:
		case SENSOR_TYPE_1104:
		case SENSOR_TYPE_1105:
		case SENSOR_TYPE_1106:
		case SENSOR_TYPE_1107:
		case SENSOR_TYPE_1108:
		case SENSOR_TYPE_1109:
		case SENSOR_TYPE_1110:
		case SENSOR_TYPE_1111:
		case SENSOR_TYPE_1112:
		case SENSOR_TYPE_1113:
		case SENSOR_TYPE_1115:
		case SENSOR_TYPE_1116:
		case SENSOR_TYPE_1118_AC:
		case SENSOR_TYPE_1118_DC:
		case SENSOR_TYPE_1119_AC:
		case SENSOR_TYPE_1119_DC:
		case SENSOR_TYPE_1120:
		case SENSOR_TYPE_1121:
		case SENSOR_TYPE_1101_SHARP_2D120X:
		case SENSOR_TYPE_1122_DC:
		case SENSOR_TYPE_1124:
		case SENSOR_TYPE_1125_HUMIDITY:
		case SENSOR_TYPE_1125_TEMPERATURE:
		case SENSOR_TYPE_1126:
		case SENSOR_TYPE_1128:
		case SENSOR_TYPE_1129:
		case SENSOR_TYPE_1131:
		case SENSOR_TYPE_1134:
		case SENSOR_TYPE_1136:
		case SENSOR_TYPE_1137:
		case SENSOR_TYPE_1138:
		case SENSOR_TYPE_1139:
		case SENSOR_TYPE_1140:
		case SENSOR_TYPE_1141:
		case SENSOR_TYPE_1146:
		case SENSOR_TYPE_3120:
		case SENSOR_TYPE_3121:
		case SENSOR_TYPE_3122:
		case SENSOR_TYPE_3123:
		case SENSOR_TYPE_3130:
		case SENSOR_TYPE_3520:
		case SENSOR_TYPE_3521:
		case SENSOR_TYPE_1122_AC:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1018_VOLTAGERATIOINPUT_821:
		switch (val) {
		case SENSOR_TYPE_VOLTAGERATIO:
		case SENSOR_TYPE_3522:
		case SENSOR_TYPE_1101_SHARP_2Y0A21:
		case SENSOR_TYPE_1101_SHARP_2Y0A02:
		case SENSOR_TYPE_1102:
		case SENSOR_TYPE_1103:
		case SENSOR_TYPE_1104:
		case SENSOR_TYPE_1105:
		case SENSOR_TYPE_1106:
		case SENSOR_TYPE_1107:
		case SENSOR_TYPE_1108:
		case SENSOR_TYPE_1109:
		case SENSOR_TYPE_1110:
		case SENSOR_TYPE_1111:
		case SENSOR_TYPE_1112:
		case SENSOR_TYPE_1113:
		case SENSOR_TYPE_1115:
		case SENSOR_TYPE_1116:
		case SENSOR_TYPE_1118_AC:
		case SENSOR_TYPE_1118_DC:
		case SENSOR_TYPE_1119_AC:
		case SENSOR_TYPE_1119_DC:
		case SENSOR_TYPE_1120:
		case SENSOR_TYPE_1121:
		case SENSOR_TYPE_1101_SHARP_2D120X:
		case SENSOR_TYPE_1122_DC:
		case SENSOR_TYPE_1124:
		case SENSOR_TYPE_1125_HUMIDITY:
		case SENSOR_TYPE_1125_TEMPERATURE:
		case SENSOR_TYPE_1126:
		case SENSOR_TYPE_1128:
		case SENSOR_TYPE_1129:
		case SENSOR_TYPE_1131:
		case SENSOR_TYPE_1134:
		case SENSOR_TYPE_1136:
		case SENSOR_TYPE_1137:
		case SENSOR_TYPE_1138:
		case SENSOR_TYPE_1139:
		case SENSOR_TYPE_1140:
		case SENSOR_TYPE_1141:
		case SENSOR_TYPE_1146:
		case SENSOR_TYPE_3120:
		case SENSOR_TYPE_3121:
		case SENSOR_TYPE_3122:
		case SENSOR_TYPE_3123:
		case SENSOR_TYPE_3130:
		case SENSOR_TYPE_3520:
		case SENSOR_TYPE_3521:
		case SENSOR_TYPE_1122_AC:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1018_VOLTAGERATIOINPUT_900:
		switch (val) {
		case SENSOR_TYPE_VOLTAGERATIO:
		case SENSOR_TYPE_3522:
		case SENSOR_TYPE_1101_SHARP_2Y0A21:
		case SENSOR_TYPE_1101_SHARP_2Y0A02:
		case SENSOR_TYPE_1102:
		case SENSOR_TYPE_1103:
		case SENSOR_TYPE_1104:
		case SENSOR_TYPE_1105:
		case SENSOR_TYPE_1106:
		case SENSOR_TYPE_1107:
		case SENSOR_TYPE_1108:
		case SENSOR_TYPE_1109:
		case SENSOR_TYPE_1110:
		case SENSOR_TYPE_1111:
		case SENSOR_TYPE_1112:
		case SENSOR_TYPE_1113:
		case SENSOR_TYPE_1115:
		case SENSOR_TYPE_1116:
		case SENSOR_TYPE_1118_AC:
		case SENSOR_TYPE_1118_DC:
		case SENSOR_TYPE_1119_AC:
		case SENSOR_TYPE_1119_DC:
		case SENSOR_TYPE_1120:
		case SENSOR_TYPE_1121:
		case SENSOR_TYPE_1101_SHARP_2D120X:
		case SENSOR_TYPE_1122_DC:
		case SENSOR_TYPE_1124:
		case SENSOR_TYPE_1125_HUMIDITY:
		case SENSOR_TYPE_1125_TEMPERATURE:
		case SENSOR_TYPE_1126:
		case SENSOR_TYPE_1128:
		case SENSOR_TYPE_1129:
		case SENSOR_TYPE_1131:
		case SENSOR_TYPE_1134:
		case SENSOR_TYPE_1136:
		case SENSOR_TYPE_1137:
		case SENSOR_TYPE_1138:
		case SENSOR_TYPE_1139:
		case SENSOR_TYPE_1140:
		case SENSOR_TYPE_1141:
		case SENSOR_TYPE_1146:
		case SENSOR_TYPE_3120:
		case SENSOR_TYPE_3121:
		case SENSOR_TYPE_3122:
		case SENSOR_TYPE_3123:
		case SENSOR_TYPE_3130:
		case SENSOR_TYPE_3520:
		case SENSOR_TYPE_3521:
		case SENSOR_TYPE_1122_AC:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1018_VOLTAGERATIOINPUT_1000:
		switch (val) {
		case SENSOR_TYPE_VOLTAGERATIO:
		case SENSOR_TYPE_3522:
		case SENSOR_TYPE_1101_SHARP_2Y0A21:
		case SENSOR_TYPE_1101_SHARP_2Y0A02:
		case SENSOR_TYPE_1102:
		case SENSOR_TYPE_1103:
		case SENSOR_TYPE_1104:
		case SENSOR_TYPE_1105:
		case SENSOR_TYPE_1106:
		case SENSOR_TYPE_1107:
		case SENSOR_TYPE_1108:
		case SENSOR_TYPE_1109:
		case SENSOR_TYPE_1110:
		case SENSOR_TYPE_1111:
		case SENSOR_TYPE_1112:
		case SENSOR_TYPE_1113:
		case SENSOR_TYPE_1115:
		case SENSOR_TYPE_1116:
		case SENSOR_TYPE_1118_AC:
		case SENSOR_TYPE_1118_DC:
		case SENSOR_TYPE_1119_AC:
		case SENSOR_TYPE_1119_DC:
		case SENSOR_TYPE_1120:
		case SENSOR_TYPE_1121:
		case SENSOR_TYPE_1101_SHARP_2D120X:
		case SENSOR_TYPE_1122_DC:
		case SENSOR_TYPE_1124:
		case SENSOR_TYPE_1125_HUMIDITY:
		case SENSOR_TYPE_1125_TEMPERATURE:
		case SENSOR_TYPE_1126:
		case SENSOR_TYPE_1128:
		case SENSOR_TYPE_1129:
		case SENSOR_TYPE_1131:
		case SENSOR_TYPE_1134:
		case SENSOR_TYPE_1136:
		case SENSOR_TYPE_1137:
		case SENSOR_TYPE_1138:
		case SENSOR_TYPE_1139:
		case SENSOR_TYPE_1140:
		case SENSOR_TYPE_1141:
		case SENSOR_TYPE_1146:
		case SENSOR_TYPE_3120:
		case SENSOR_TYPE_3121:
		case SENSOR_TYPE_3122:
		case SENSOR_TYPE_3123:
		case SENSOR_TYPE_3130:
		case SENSOR_TYPE_3520:
		case SENSOR_TYPE_3521:
		case SENSOR_TYPE_1122_AC:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1065_VOLTAGERATIOINPUT_100:
		switch (val) {
		case SENSOR_TYPE_VOLTAGERATIO:
		case SENSOR_TYPE_3522:
		case SENSOR_TYPE_1101_SHARP_2Y0A21:
		case SENSOR_TYPE_1101_SHARP_2Y0A02:
		case SENSOR_TYPE_1102:
		case SENSOR_TYPE_1103:
		case SENSOR_TYPE_1104:
		case SENSOR_TYPE_1105:
		case SENSOR_TYPE_1106:
		case SENSOR_TYPE_1107:
		case SENSOR_TYPE_1108:
		case SENSOR_TYPE_1109:
		case SENSOR_TYPE_1110:
		case SENSOR_TYPE_1111:
		case SENSOR_TYPE_1112:
		case SENSOR_TYPE_1113:
		case SENSOR_TYPE_1115:
		case SENSOR_TYPE_1116:
		case SENSOR_TYPE_1118_AC:
		case SENSOR_TYPE_1118_DC:
		case SENSOR_TYPE_1119_AC:
		case SENSOR_TYPE_1119_DC:
		case SENSOR_TYPE_1120:
		case SENSOR_TYPE_1121:
		case SENSOR_TYPE_1101_SHARP_2D120X:
		case SENSOR_TYPE_1122_DC:
		case SENSOR_TYPE_1124:
		case SENSOR_TYPE_1125_HUMIDITY:
		case SENSOR_TYPE_1125_TEMPERATURE:
		case SENSOR_TYPE_1126:
		case SENSOR_TYPE_1128:
		case SENSOR_TYPE_1129:
		case SENSOR_TYPE_1131:
		case SENSOR_TYPE_1134:
		case SENSOR_TYPE_1136:
		case SENSOR_TYPE_1137:
		case SENSOR_TYPE_1138:
		case SENSOR_TYPE_1139:
		case SENSOR_TYPE_1140:
		case SENSOR_TYPE_1141:
		case SENSOR_TYPE_1146:
		case SENSOR_TYPE_3120:
		case SENSOR_TYPE_3121:
		case SENSOR_TYPE_3122:
		case SENSOR_TYPE_3123:
		case SENSOR_TYPE_3130:
		case SENSOR_TYPE_3520:
		case SENSOR_TYPE_3521:
		case SENSOR_TYPE_1122_AC:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1202_VOLTAGERATIOINPUT_000:
		switch (val) {
		case SENSOR_TYPE_VOLTAGERATIO:
		case SENSOR_TYPE_3522:
		case SENSOR_TYPE_1101_SHARP_2Y0A21:
		case SENSOR_TYPE_1101_SHARP_2Y0A02:
		case SENSOR_TYPE_1102:
		case SENSOR_TYPE_1103:
		case SENSOR_TYPE_1104:
		case SENSOR_TYPE_1105:
		case SENSOR_TYPE_1106:
		case SENSOR_TYPE_1107:
		case SENSOR_TYPE_1108:
		case SENSOR_TYPE_1109:
		case SENSOR_TYPE_1110:
		case SENSOR_TYPE_1111:
		case SENSOR_TYPE_1112:
		case SENSOR_TYPE_1113:
		case SENSOR_TYPE_1115:
		case SENSOR_TYPE_1116:
		case SENSOR_TYPE_1118_AC:
		case SENSOR_TYPE_1118_DC:
		case SENSOR_TYPE_1119_AC:
		case SENSOR_TYPE_1119_DC:
		case SENSOR_TYPE_1120:
		case SENSOR_TYPE_1121:
		case SENSOR_TYPE_1101_SHARP_2D120X:
		case SENSOR_TYPE_1122_DC:
		case SENSOR_TYPE_1124:
		case SENSOR_TYPE_1125_HUMIDITY:
		case SENSOR_TYPE_1125_TEMPERATURE:
		case SENSOR_TYPE_1126:
		case SENSOR_TYPE_1128:
		case SENSOR_TYPE_1129:
		case SENSOR_TYPE_1131:
		case SENSOR_TYPE_1134:
		case SENSOR_TYPE_1136:
		case SENSOR_TYPE_1137:
		case SENSOR_TYPE_1138:
		case SENSOR_TYPE_1139:
		case SENSOR_TYPE_1140:
		case SENSOR_TYPE_1141:
		case SENSOR_TYPE_1146:
		case SENSOR_TYPE_3120:
		case SENSOR_TYPE_3121:
		case SENSOR_TYPE_3122:
		case SENSOR_TYPE_3123:
		case SENSOR_TYPE_3130:
		case SENSOR_TYPE_3520:
		case SENSOR_TYPE_3521:
		case SENSOR_TYPE_1122_AC:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1202_VOLTAGERATIOINPUT_120:
		switch (val) {
		case SENSOR_TYPE_VOLTAGERATIO:
		case SENSOR_TYPE_3522:
		case SENSOR_TYPE_1101_SHARP_2Y0A21:
		case SENSOR_TYPE_1101_SHARP_2Y0A02:
		case SENSOR_TYPE_1102:
		case SENSOR_TYPE_1103:
		case SENSOR_TYPE_1104:
		case SENSOR_TYPE_1105:
		case SENSOR_TYPE_1106:
		case SENSOR_TYPE_1107:
		case SENSOR_TYPE_1108:
		case SENSOR_TYPE_1109:
		case SENSOR_TYPE_1110:
		case SENSOR_TYPE_1111:
		case SENSOR_TYPE_1112:
		case SENSOR_TYPE_1113:
		case SENSOR_TYPE_1115:
		case SENSOR_TYPE_1116:
		case SENSOR_TYPE_1118_AC:
		case SENSOR_TYPE_1118_DC:
		case SENSOR_TYPE_1119_AC:
		case SENSOR_TYPE_1119_DC:
		case SENSOR_TYPE_1120:
		case SENSOR_TYPE_1121:
		case SENSOR_TYPE_1101_SHARP_2D120X:
		case SENSOR_TYPE_1122_DC:
		case SENSOR_TYPE_1124:
		case SENSOR_TYPE_1125_HUMIDITY:
		case SENSOR_TYPE_1125_TEMPERATURE:
		case SENSOR_TYPE_1126:
		case SENSOR_TYPE_1128:
		case SENSOR_TYPE_1129:
		case SENSOR_TYPE_1131:
		case SENSOR_TYPE_1134:
		case SENSOR_TYPE_1136:
		case SENSOR_TYPE_1137:
		case SENSOR_TYPE_1138:
		case SENSOR_TYPE_1139:
		case SENSOR_TYPE_1140:
		case SENSOR_TYPE_1141:
		case SENSOR_TYPE_1146:
		case SENSOR_TYPE_3120:
		case SENSOR_TYPE_3121:
		case SENSOR_TYPE_3122:
		case SENSOR_TYPE_3123:
		case SENSOR_TYPE_3130:
		case SENSOR_TYPE_3520:
		case SENSOR_TYPE_3521:
		case SENSOR_TYPE_1122_AC:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1202_VOLTAGERATIOINPUT_300:
		switch (val) {
		case SENSOR_TYPE_VOLTAGERATIO:
		case SENSOR_TYPE_3522:
		case SENSOR_TYPE_1101_SHARP_2Y0A21:
		case SENSOR_TYPE_1101_SHARP_2Y0A02:
		case SENSOR_TYPE_1102:
		case SENSOR_TYPE_1103:
		case SENSOR_TYPE_1104:
		case SENSOR_TYPE_1105:
		case SENSOR_TYPE_1106:
		case SENSOR_TYPE_1107:
		case SENSOR_TYPE_1108:
		case SENSOR_TYPE_1109:
		case SENSOR_TYPE_1110:
		case SENSOR_TYPE_1111:
		case SENSOR_TYPE_1112:
		case SENSOR_TYPE_1113:
		case SENSOR_TYPE_1115:
		case SENSOR_TYPE_1116:
		case SENSOR_TYPE_1118_AC:
		case SENSOR_TYPE_1118_DC:
		case SENSOR_TYPE_1119_AC:
		case SENSOR_TYPE_1119_DC:
		case SENSOR_TYPE_1120:
		case SENSOR_TYPE_1121:
		case SENSOR_TYPE_1101_SHARP_2D120X:
		case SENSOR_TYPE_1122_DC:
		case SENSOR_TYPE_1124:
		case SENSOR_TYPE_1125_HUMIDITY:
		case SENSOR_TYPE_1125_TEMPERATURE:
		case SENSOR_TYPE_1126:
		case SENSOR_TYPE_1128:
		case SENSOR_TYPE_1129:
		case SENSOR_TYPE_1131:
		case SENSOR_TYPE_1134:
		case SENSOR_TYPE_1136:
		case SENSOR_TYPE_1137:
		case SENSOR_TYPE_1138:
		case SENSOR_TYPE_1139:
		case SENSOR_TYPE_1140:
		case SENSOR_TYPE_1141:
		case SENSOR_TYPE_1146:
		case SENSOR_TYPE_3120:
		case SENSOR_TYPE_3121:
		case SENSOR_TYPE_3122:
		case SENSOR_TYPE_3123:
		case SENSOR_TYPE_3130:
		case SENSOR_TYPE_3520:
		case SENSOR_TYPE_3521:
		case SENSOR_TYPE_1122_AC:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_HUB_VOLTAGERATIOINPUT_100:
		switch (val) {
		case SENSOR_TYPE_VOLTAGERATIO:
		case SENSOR_TYPE_3522:
		case SENSOR_TYPE_1101_SHARP_2Y0A21:
		case SENSOR_TYPE_1101_SHARP_2Y0A02:
		case SENSOR_TYPE_1102:
		case SENSOR_TYPE_1103:
		case SENSOR_TYPE_1104:
		case SENSOR_TYPE_1105:
		case SENSOR_TYPE_1106:
		case SENSOR_TYPE_1107:
		case SENSOR_TYPE_1108:
		case SENSOR_TYPE_1109:
		case SENSOR_TYPE_1110:
		case SENSOR_TYPE_1111:
		case SENSOR_TYPE_1112:
		case SENSOR_TYPE_1113:
		case SENSOR_TYPE_1115:
		case SENSOR_TYPE_1116:
		case SENSOR_TYPE_1118_AC:
		case SENSOR_TYPE_1118_DC:
		case SENSOR_TYPE_1119_AC:
		case SENSOR_TYPE_1119_DC:
		case SENSOR_TYPE_1120:
		case SENSOR_TYPE_1121:
		case SENSOR_TYPE_1101_SHARP_2D120X:
		case SENSOR_TYPE_1122_DC:
		case SENSOR_TYPE_1124:
		case SENSOR_TYPE_1125_HUMIDITY:
		case SENSOR_TYPE_1125_TEMPERATURE:
		case SENSOR_TYPE_1126:
		case SENSOR_TYPE_1128:
		case SENSOR_TYPE_1129:
		case SENSOR_TYPE_1131:
		case SENSOR_TYPE_1134:
		case SENSOR_TYPE_1136:
		case SENSOR_TYPE_1137:
		case SENSOR_TYPE_1138:
		case SENSOR_TYPE_1139:
		case SENSOR_TYPE_1140:
		case SENSOR_TYPE_1141:
		case SENSOR_TYPE_1146:
		case SENSOR_TYPE_3120:
		case SENSOR_TYPE_3121:
		case SENSOR_TYPE_3122:
		case SENSOR_TYPE_3123:
		case SENSOR_TYPE_3130:
		case SENSOR_TYPE_3520:
		case SENSOR_TYPE_3521:
		case SENSOR_TYPE_1122_AC:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DAQ1000_VOLTAGERATIOINPUT_100:
		switch (val) {
		case SENSOR_TYPE_VOLTAGERATIO:
		case SENSOR_TYPE_3522:
		case SENSOR_TYPE_1101_SHARP_2Y0A21:
		case SENSOR_TYPE_1101_SHARP_2Y0A02:
		case SENSOR_TYPE_1102:
		case SENSOR_TYPE_1103:
		case SENSOR_TYPE_1104:
		case SENSOR_TYPE_1105:
		case SENSOR_TYPE_1106:
		case SENSOR_TYPE_1107:
		case SENSOR_TYPE_1108:
		case SENSOR_TYPE_1109:
		case SENSOR_TYPE_1110:
		case SENSOR_TYPE_1111:
		case SENSOR_TYPE_1112:
		case SENSOR_TYPE_1113:
		case SENSOR_TYPE_1115:
		case SENSOR_TYPE_1116:
		case SENSOR_TYPE_1118_AC:
		case SENSOR_TYPE_1118_DC:
		case SENSOR_TYPE_1119_AC:
		case SENSOR_TYPE_1119_DC:
		case SENSOR_TYPE_1120:
		case SENSOR_TYPE_1121:
		case SENSOR_TYPE_1101_SHARP_2D120X:
		case SENSOR_TYPE_1122_DC:
		case SENSOR_TYPE_1124:
		case SENSOR_TYPE_1125_HUMIDITY:
		case SENSOR_TYPE_1125_TEMPERATURE:
		case SENSOR_TYPE_1126:
		case SENSOR_TYPE_1128:
		case SENSOR_TYPE_1129:
		case SENSOR_TYPE_1131:
		case SENSOR_TYPE_1134:
		case SENSOR_TYPE_1136:
		case SENSOR_TYPE_1137:
		case SENSOR_TYPE_1138:
		case SENSOR_TYPE_1139:
		case SENSOR_TYPE_1140:
		case SENSOR_TYPE_1141:
		case SENSOR_TYPE_1146:
		case SENSOR_TYPE_3120:
		case SENSOR_TYPE_3121:
		case SENSOR_TYPE_3122:
		case SENSOR_TYPE_3123:
		case SENSOR_TYPE_3130:
		case SENSOR_TYPE_3520:
		case SENSOR_TYPE_3521:
		case SENSOR_TYPE_1122_AC:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DAQ1000_VOLTAGERATIOINPUT_110:
		switch (val) {
		case SENSOR_TYPE_VOLTAGERATIO:
		case SENSOR_TYPE_3522:
		case SENSOR_TYPE_1101_SHARP_2Y0A21:
		case SENSOR_TYPE_1101_SHARP_2Y0A02:
		case SENSOR_TYPE_1102:
		case SENSOR_TYPE_1103:
		case SENSOR_TYPE_1104:
		case SENSOR_TYPE_1105:
		case SENSOR_TYPE_1106:
		case SENSOR_TYPE_1107:
		case SENSOR_TYPE_1108:
		case SENSOR_TYPE_1109:
		case SENSOR_TYPE_1110:
		case SENSOR_TYPE_1111:
		case SENSOR_TYPE_1112:
		case SENSOR_TYPE_1113:
		case SENSOR_TYPE_1115:
		case SENSOR_TYPE_1116:
		case SENSOR_TYPE_1118_AC:
		case SENSOR_TYPE_1118_DC:
		case SENSOR_TYPE_1119_AC:
		case SENSOR_TYPE_1119_DC:
		case SENSOR_TYPE_1120:
		case SENSOR_TYPE_1121:
		case SENSOR_TYPE_1101_SHARP_2D120X:
		case SENSOR_TYPE_1122_DC:
		case SENSOR_TYPE_1124:
		case SENSOR_TYPE_1125_HUMIDITY:
		case SENSOR_TYPE_1125_TEMPERATURE:
		case SENSOR_TYPE_1126:
		case SENSOR_TYPE_1128:
		case SENSOR_TYPE_1129:
		case SENSOR_TYPE_1131:
		case SENSOR_TYPE_1134:
		case SENSOR_TYPE_1136:
		case SENSOR_TYPE_1137:
		case SENSOR_TYPE_1138:
		case SENSOR_TYPE_1139:
		case SENSOR_TYPE_1140:
		case SENSOR_TYPE_1141:
		case SENSOR_TYPE_1146:
		case SENSOR_TYPE_3120:
		case SENSOR_TYPE_3121:
		case SENSOR_TYPE_3122:
		case SENSOR_TYPE_3123:
		case SENSOR_TYPE_3130:
		case SENSOR_TYPE_3520:
		case SENSOR_TYPE_3521:
		case SENSOR_TYPE_1122_AC:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1000_VOLTAGERATIOINPUT_100:
		switch (val) {
		case SENSOR_TYPE_VOLTAGERATIO:
		case SENSOR_TYPE_3522:
		case SENSOR_TYPE_1101_SHARP_2Y0A21:
		case SENSOR_TYPE_1101_SHARP_2Y0A02:
		case SENSOR_TYPE_1102:
		case SENSOR_TYPE_1103:
		case SENSOR_TYPE_1104:
		case SENSOR_TYPE_1105:
		case SENSOR_TYPE_1106:
		case SENSOR_TYPE_1107:
		case SENSOR_TYPE_1108:
		case SENSOR_TYPE_1109:
		case SENSOR_TYPE_1110:
		case SENSOR_TYPE_1111:
		case SENSOR_TYPE_1112:
		case SENSOR_TYPE_1113:
		case SENSOR_TYPE_1115:
		case SENSOR_TYPE_1116:
		case SENSOR_TYPE_1118_AC:
		case SENSOR_TYPE_1118_DC:
		case SENSOR_TYPE_1119_AC:
		case SENSOR_TYPE_1119_DC:
		case SENSOR_TYPE_1120:
		case SENSOR_TYPE_1121:
		case SENSOR_TYPE_1101_SHARP_2D120X:
		case SENSOR_TYPE_1122_DC:
		case SENSOR_TYPE_1124:
		case SENSOR_TYPE_1125_HUMIDITY:
		case SENSOR_TYPE_1125_TEMPERATURE:
		case SENSOR_TYPE_1126:
		case SENSOR_TYPE_1128:
		case SENSOR_TYPE_1129:
		case SENSOR_TYPE_1131:
		case SENSOR_TYPE_1134:
		case SENSOR_TYPE_1136:
		case SENSOR_TYPE_1137:
		case SENSOR_TYPE_1138:
		case SENSOR_TYPE_1139:
		case SENSOR_TYPE_1140:
		case SENSOR_TYPE_1141:
		case SENSOR_TYPE_1146:
		case SENSOR_TYPE_3120:
		case SENSOR_TYPE_3121:
		case SENSOR_TYPE_3122:
		case SENSOR_TYPE_3123:
		case SENSOR_TYPE_3130:
		case SENSOR_TYPE_3520:
		case SENSOR_TYPE_3521:
		case SENSOR_TYPE_1122_AC:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1000_VOLTAGERATIOINPUT_200:
		switch (val) {
		case SENSOR_TYPE_VOLTAGERATIO:
		case SENSOR_TYPE_3522:
		case SENSOR_TYPE_1101_SHARP_2Y0A21:
		case SENSOR_TYPE_1101_SHARP_2Y0A02:
		case SENSOR_TYPE_1102:
		case SENSOR_TYPE_1103:
		case SENSOR_TYPE_1104:
		case SENSOR_TYPE_1105:
		case SENSOR_TYPE_1106:
		case SENSOR_TYPE_1107:
		case SENSOR_TYPE_1108:
		case SENSOR_TYPE_1109:
		case SENSOR_TYPE_1110:
		case SENSOR_TYPE_1111:
		case SENSOR_TYPE_1112:
		case SENSOR_TYPE_1113:
		case SENSOR_TYPE_1115:
		case SENSOR_TYPE_1116:
		case SENSOR_TYPE_1118_AC:
		case SENSOR_TYPE_1118_DC:
		case SENSOR_TYPE_1119_AC:
		case SENSOR_TYPE_1119_DC:
		case SENSOR_TYPE_1120:
		case SENSOR_TYPE_1121:
		case SENSOR_TYPE_1101_SHARP_2D120X:
		case SENSOR_TYPE_1122_DC:
		case SENSOR_TYPE_1124:
		case SENSOR_TYPE_1125_HUMIDITY:
		case SENSOR_TYPE_1125_TEMPERATURE:
		case SENSOR_TYPE_1126:
		case SENSOR_TYPE_1128:
		case SENSOR_TYPE_1129:
		case SENSOR_TYPE_1131:
		case SENSOR_TYPE_1134:
		case SENSOR_TYPE_1136:
		case SENSOR_TYPE_1137:
		case SENSOR_TYPE_1138:
		case SENSOR_TYPE_1139:
		case SENSOR_TYPE_1140:
		case SENSOR_TYPE_1141:
		case SENSOR_TYPE_1146:
		case SENSOR_TYPE_3120:
		case SENSOR_TYPE_3121:
		case SENSOR_TYPE_3122:
		case SENSOR_TYPE_3123:
		case SENSOR_TYPE_3130:
		case SENSOR_TYPE_3520:
		case SENSOR_TYPE_3521:
		case SENSOR_TYPE_1122_AC:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DCC1000_VOLTAGERATIOINPUT_210:
		switch (val) {
		case SENSOR_TYPE_VOLTAGERATIO:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_HIN1100_VOLTAGERATIOINPUT_100:
		switch (val) {
		case SENSOR_TYPE_VOLTAGERATIO:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_HUM1100_VOLTAGERATIOINPUT_100:
		switch (val) {
		case SENSOR_TYPE_VOLTAGERATIO:
			return (1);
		default:
			return (0);
		}
	default:
		return (0);
	}
}

int
supportedLEDForwardVoltage(PhidgetChannelHandle ch, PhidgetDigitalOutput_LEDForwardVoltage val) {

	switch (ch->UCD->uid) {
	case PHIDCHUID_1030_DIGITALOUTPUT_100:
		switch (val) {
		case LED_FORWARD_VOLTAGE_3_2V:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1031_DIGITALOUTPUT_100:
		switch (val) {
		case LED_FORWARD_VOLTAGE_1_7V:
		case LED_FORWARD_VOLTAGE_2_75V:
		case LED_FORWARD_VOLTAGE_3_9V:
		case LED_FORWARD_VOLTAGE_5_0V:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1032_DIGITALOUTPUT_200:
		switch (val) {
		case LED_FORWARD_VOLTAGE_1_7V:
		case LED_FORWARD_VOLTAGE_2_75V:
		case LED_FORWARD_VOLTAGE_3_9V:
		case LED_FORWARD_VOLTAGE_5_0V:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_LED1000_DIGITALOUTPUT_100:
		switch (val) {
		case LED_FORWARD_VOLTAGE_3_2V:
		case LED_FORWARD_VOLTAGE_4_0V:
		case LED_FORWARD_VOLTAGE_4_8V:
		case LED_FORWARD_VOLTAGE_5_6V:
			return (1);
		default:
			return (0);
		}
	default:
		return (0);
	}
}

int
supportedRCServoVoltage(PhidgetChannelHandle ch, PhidgetRCServo_Voltage val) {

	switch (ch->UCD->uid) {
	case PHIDCHUID_1000_RCSERVO_OLD1_200:
		switch (val) {
		case RCSERVO_VOLTAGE_5V:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1000_RCSERVO_OLD2_200:
		switch (val) {
		case RCSERVO_VOLTAGE_5V:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1000_RCSERVO_300:
		switch (val) {
		case RCSERVO_VOLTAGE_5V:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1000_RCSERVO_313:
		switch (val) {
		case RCSERVO_VOLTAGE_5V:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1001_RCSERVO_OLD1_200:
		switch (val) {
		case RCSERVO_VOLTAGE_5V:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1001_RCSERVO_OLD2_200:
		switch (val) {
		case RCSERVO_VOLTAGE_5V:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1001_RCSERVO_313:
		switch (val) {
		case RCSERVO_VOLTAGE_5V:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1001_RCSERVO_400:
		switch (val) {
		case RCSERVO_VOLTAGE_5V:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1061_RCSERVO_100:
		switch (val) {
		case RCSERVO_VOLTAGE_5V:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1061_RCSERVO_200:
		switch (val) {
		case RCSERVO_VOLTAGE_5V:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1061_RCSERVO_300:
		switch (val) {
		case RCSERVO_VOLTAGE_5V:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_RCC0004_RCSERVO_400:
		switch (val) {
		case RCSERVO_VOLTAGE_5V:
		case RCSERVO_VOLTAGE_6V:
		case RCSERVO_VOLTAGE_7_4V:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1066_RCSERVO_100:
		switch (val) {
		case RCSERVO_VOLTAGE_5V:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_RCC1000_RCSERVO_100:
		switch (val) {
		case RCSERVO_VOLTAGE_5V:
		case RCSERVO_VOLTAGE_6V:
		case RCSERVO_VOLTAGE_7_4V:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_RCC1000_RCSERVO_110:
		switch (val) {
		case RCSERVO_VOLTAGE_5V:
		case RCSERVO_VOLTAGE_6V:
		case RCSERVO_VOLTAGE_7_4V:
			return (1);
		default:
			return (0);
		}
	default:
		return (0);
	}
}

int
supportedVoltageOutputRange(PhidgetChannelHandle ch, PhidgetVoltageOutput_VoltageOutputRange val) {

	switch (ch->UCD->uid) {
	case PHIDCHUID_1002_VOLTAGEOUTPUT_100:
		switch (val) {
		case VOLTAGE_OUTPUT_RANGE_10V:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_OUT1001_VOLTAGEOUTPUT_100:
		switch (val) {
		case VOLTAGE_OUTPUT_RANGE_10V:
		case VOLTAGE_OUTPUT_RANGE_5V:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_OUT1001_VOLTAGEOUTPUT_110:
		switch (val) {
		case VOLTAGE_OUTPUT_RANGE_10V:
		case VOLTAGE_OUTPUT_RANGE_5V:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_OUT1002_VOLTAGEOUTPUT_100:
		switch (val) {
		case VOLTAGE_OUTPUT_RANGE_10V:
		case VOLTAGE_OUTPUT_RANGE_5V:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_OUT1002_VOLTAGEOUTPUT_110:
		switch (val) {
		case VOLTAGE_OUTPUT_RANGE_10V:
		case VOLTAGE_OUTPUT_RANGE_5V:
			return (1);
		default:
			return (0);
		}
	default:
		return (0);
	}
}

int
supportedVoltageRange(PhidgetChannelHandle ch, PhidgetVoltageInput_VoltageRange val) {

	switch (ch->UCD->uid) {
	case PHIDCHUID_ADP1000_VOLTAGEINPUT_100:
		switch (val) {
		case VOLTAGE_RANGE_400mV:
		case VOLTAGE_RANGE_2V:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_VCP1000_VOLTAGEINPUT_100:
		switch (val) {
		case VOLTAGE_RANGE_312_5mV:
		case VOLTAGE_RANGE_40V:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_VCP1001_VOLTAGEINPUT_100:
		switch (val) {
		case VOLTAGE_RANGE_AUTO:
		case VOLTAGE_RANGE_5V:
		case VOLTAGE_RANGE_15V:
		case VOLTAGE_RANGE_40V:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_VCP1001_VOLTAGEINPUT_110:
		switch (val) {
		case VOLTAGE_RANGE_AUTO:
		case VOLTAGE_RANGE_5V:
		case VOLTAGE_RANGE_15V:
		case VOLTAGE_RANGE_40V:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_VCP1002_VOLTAGEINPUT_100:
		switch (val) {
		case VOLTAGE_RANGE_10mV:
		case VOLTAGE_RANGE_AUTO:
		case VOLTAGE_RANGE_200mV:
		case VOLTAGE_RANGE_40mV:
		case VOLTAGE_RANGE_1000mV:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_VCP1002_VOLTAGEINPUT_110:
		switch (val) {
		case VOLTAGE_RANGE_10mV:
		case VOLTAGE_RANGE_AUTO:
		case VOLTAGE_RANGE_200mV:
		case VOLTAGE_RANGE_40mV:
		case VOLTAGE_RANGE_1000mV:
			return (1);
		default:
			return (0);
		}
	default:
		return (0);
	}
}

int
supportedVoltageSensorType(PhidgetChannelHandle ch, PhidgetVoltageInput_SensorType val) {

	switch (ch->UCD->uid) {
	case PHIDCHUID_1011_VOLTAGEINPUT_000:
		switch (val) {
		case SENSOR_TYPE_VOLTAGE:
		case SENSOR_TYPE_VCP4114:
		case SENSOR_TYPE_1117:
		case SENSOR_TYPE_1123:
		case SENSOR_TYPE_1127:
		case SENSOR_TYPE_1130_PH:
		case SENSOR_TYPE_1130_ORP:
		case SENSOR_TYPE_1132:
		case SENSOR_TYPE_1133:
		case SENSOR_TYPE_1135:
		case SENSOR_TYPE_1142:
		case SENSOR_TYPE_1143:
		case SENSOR_TYPE_3500:
		case SENSOR_TYPE_3501:
		case SENSOR_TYPE_3502:
		case SENSOR_TYPE_3503:
		case SENSOR_TYPE_3507:
		case SENSOR_TYPE_3508:
		case SENSOR_TYPE_3509:
		case SENSOR_TYPE_1114:
		case SENSOR_TYPE_3511:
		case SENSOR_TYPE_3512:
		case SENSOR_TYPE_3513:
		case SENSOR_TYPE_3514:
		case SENSOR_TYPE_3515:
		case SENSOR_TYPE_3516:
		case SENSOR_TYPE_3517:
		case SENSOR_TYPE_3518:
		case SENSOR_TYPE_3519:
		case SENSOR_TYPE_3584:
		case SENSOR_TYPE_3585:
		case SENSOR_TYPE_3586:
		case SENSOR_TYPE_3587:
		case SENSOR_TYPE_3588:
		case SENSOR_TYPE_3589:
		case SENSOR_TYPE_MOT2002_LOW:
		case SENSOR_TYPE_MOT2002_MED:
		case SENSOR_TYPE_MOT2002_HIGH:
		case SENSOR_TYPE_3510:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1013_VOLTAGEINPUT_000:
		switch (val) {
		case SENSOR_TYPE_VOLTAGE:
		case SENSOR_TYPE_VCP4114:
		case SENSOR_TYPE_1117:
		case SENSOR_TYPE_1123:
		case SENSOR_TYPE_1127:
		case SENSOR_TYPE_1130_PH:
		case SENSOR_TYPE_1130_ORP:
		case SENSOR_TYPE_1132:
		case SENSOR_TYPE_1133:
		case SENSOR_TYPE_1135:
		case SENSOR_TYPE_1142:
		case SENSOR_TYPE_1143:
		case SENSOR_TYPE_3500:
		case SENSOR_TYPE_3501:
		case SENSOR_TYPE_3502:
		case SENSOR_TYPE_3503:
		case SENSOR_TYPE_3507:
		case SENSOR_TYPE_3508:
		case SENSOR_TYPE_3509:
		case SENSOR_TYPE_1114:
		case SENSOR_TYPE_3511:
		case SENSOR_TYPE_3512:
		case SENSOR_TYPE_3513:
		case SENSOR_TYPE_3514:
		case SENSOR_TYPE_3515:
		case SENSOR_TYPE_3516:
		case SENSOR_TYPE_3517:
		case SENSOR_TYPE_3518:
		case SENSOR_TYPE_3519:
		case SENSOR_TYPE_3584:
		case SENSOR_TYPE_3585:
		case SENSOR_TYPE_3586:
		case SENSOR_TYPE_3587:
		case SENSOR_TYPE_3588:
		case SENSOR_TYPE_3589:
		case SENSOR_TYPE_MOT2002_LOW:
		case SENSOR_TYPE_MOT2002_MED:
		case SENSOR_TYPE_MOT2002_HIGH:
		case SENSOR_TYPE_3510:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1018_VOLTAGEINPUT_821:
		switch (val) {
		case SENSOR_TYPE_VOLTAGE:
		case SENSOR_TYPE_VCP4114:
		case SENSOR_TYPE_1117:
		case SENSOR_TYPE_1123:
		case SENSOR_TYPE_1127:
		case SENSOR_TYPE_1130_PH:
		case SENSOR_TYPE_1130_ORP:
		case SENSOR_TYPE_1132:
		case SENSOR_TYPE_1133:
		case SENSOR_TYPE_1135:
		case SENSOR_TYPE_1142:
		case SENSOR_TYPE_1143:
		case SENSOR_TYPE_3500:
		case SENSOR_TYPE_3501:
		case SENSOR_TYPE_3502:
		case SENSOR_TYPE_3503:
		case SENSOR_TYPE_3507:
		case SENSOR_TYPE_3508:
		case SENSOR_TYPE_3509:
		case SENSOR_TYPE_1114:
		case SENSOR_TYPE_3511:
		case SENSOR_TYPE_3512:
		case SENSOR_TYPE_3513:
		case SENSOR_TYPE_3514:
		case SENSOR_TYPE_3515:
		case SENSOR_TYPE_3516:
		case SENSOR_TYPE_3517:
		case SENSOR_TYPE_3518:
		case SENSOR_TYPE_3519:
		case SENSOR_TYPE_3584:
		case SENSOR_TYPE_3585:
		case SENSOR_TYPE_3586:
		case SENSOR_TYPE_3587:
		case SENSOR_TYPE_3588:
		case SENSOR_TYPE_3589:
		case SENSOR_TYPE_MOT2002_LOW:
		case SENSOR_TYPE_MOT2002_MED:
		case SENSOR_TYPE_MOT2002_HIGH:
		case SENSOR_TYPE_3510:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1018_VOLTAGEINPUT_900:
		switch (val) {
		case SENSOR_TYPE_VOLTAGE:
		case SENSOR_TYPE_VCP4114:
		case SENSOR_TYPE_1117:
		case SENSOR_TYPE_1123:
		case SENSOR_TYPE_1127:
		case SENSOR_TYPE_1130_PH:
		case SENSOR_TYPE_1130_ORP:
		case SENSOR_TYPE_1132:
		case SENSOR_TYPE_1133:
		case SENSOR_TYPE_1135:
		case SENSOR_TYPE_1142:
		case SENSOR_TYPE_1143:
		case SENSOR_TYPE_3500:
		case SENSOR_TYPE_3501:
		case SENSOR_TYPE_3502:
		case SENSOR_TYPE_3503:
		case SENSOR_TYPE_3507:
		case SENSOR_TYPE_3508:
		case SENSOR_TYPE_3509:
		case SENSOR_TYPE_1114:
		case SENSOR_TYPE_3511:
		case SENSOR_TYPE_3512:
		case SENSOR_TYPE_3513:
		case SENSOR_TYPE_3514:
		case SENSOR_TYPE_3515:
		case SENSOR_TYPE_3516:
		case SENSOR_TYPE_3517:
		case SENSOR_TYPE_3518:
		case SENSOR_TYPE_3519:
		case SENSOR_TYPE_3584:
		case SENSOR_TYPE_3585:
		case SENSOR_TYPE_3586:
		case SENSOR_TYPE_3587:
		case SENSOR_TYPE_3588:
		case SENSOR_TYPE_3589:
		case SENSOR_TYPE_MOT2002_LOW:
		case SENSOR_TYPE_MOT2002_MED:
		case SENSOR_TYPE_MOT2002_HIGH:
		case SENSOR_TYPE_3510:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1018_VOLTAGEINPUT_1000:
		switch (val) {
		case SENSOR_TYPE_VOLTAGE:
		case SENSOR_TYPE_VCP4114:
		case SENSOR_TYPE_1117:
		case SENSOR_TYPE_1123:
		case SENSOR_TYPE_1127:
		case SENSOR_TYPE_1130_PH:
		case SENSOR_TYPE_1130_ORP:
		case SENSOR_TYPE_1132:
		case SENSOR_TYPE_1133:
		case SENSOR_TYPE_1135:
		case SENSOR_TYPE_1142:
		case SENSOR_TYPE_1143:
		case SENSOR_TYPE_3500:
		case SENSOR_TYPE_3501:
		case SENSOR_TYPE_3502:
		case SENSOR_TYPE_3503:
		case SENSOR_TYPE_3507:
		case SENSOR_TYPE_3508:
		case SENSOR_TYPE_3509:
		case SENSOR_TYPE_1114:
		case SENSOR_TYPE_3511:
		case SENSOR_TYPE_3512:
		case SENSOR_TYPE_3513:
		case SENSOR_TYPE_3514:
		case SENSOR_TYPE_3515:
		case SENSOR_TYPE_3516:
		case SENSOR_TYPE_3517:
		case SENSOR_TYPE_3518:
		case SENSOR_TYPE_3519:
		case SENSOR_TYPE_3584:
		case SENSOR_TYPE_3585:
		case SENSOR_TYPE_3586:
		case SENSOR_TYPE_3587:
		case SENSOR_TYPE_3588:
		case SENSOR_TYPE_3589:
		case SENSOR_TYPE_MOT2002_LOW:
		case SENSOR_TYPE_MOT2002_MED:
		case SENSOR_TYPE_MOT2002_HIGH:
		case SENSOR_TYPE_3510:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1065_VOLTAGEINPUT_100:
		switch (val) {
		case SENSOR_TYPE_VOLTAGE:
		case SENSOR_TYPE_VCP4114:
		case SENSOR_TYPE_1117:
		case SENSOR_TYPE_1123:
		case SENSOR_TYPE_1127:
		case SENSOR_TYPE_1130_PH:
		case SENSOR_TYPE_1130_ORP:
		case SENSOR_TYPE_1132:
		case SENSOR_TYPE_1133:
		case SENSOR_TYPE_1135:
		case SENSOR_TYPE_1142:
		case SENSOR_TYPE_1143:
		case SENSOR_TYPE_3500:
		case SENSOR_TYPE_3501:
		case SENSOR_TYPE_3502:
		case SENSOR_TYPE_3503:
		case SENSOR_TYPE_3507:
		case SENSOR_TYPE_3508:
		case SENSOR_TYPE_3509:
		case SENSOR_TYPE_1114:
		case SENSOR_TYPE_3511:
		case SENSOR_TYPE_3512:
		case SENSOR_TYPE_3513:
		case SENSOR_TYPE_3514:
		case SENSOR_TYPE_3515:
		case SENSOR_TYPE_3516:
		case SENSOR_TYPE_3517:
		case SENSOR_TYPE_3518:
		case SENSOR_TYPE_3519:
		case SENSOR_TYPE_3584:
		case SENSOR_TYPE_3585:
		case SENSOR_TYPE_3586:
		case SENSOR_TYPE_3587:
		case SENSOR_TYPE_3588:
		case SENSOR_TYPE_3589:
		case SENSOR_TYPE_MOT2002_LOW:
		case SENSOR_TYPE_MOT2002_MED:
		case SENSOR_TYPE_MOT2002_HIGH:
		case SENSOR_TYPE_3510:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1202_VOLTAGEINPUT_000:
		switch (val) {
		case SENSOR_TYPE_VOLTAGE:
		case SENSOR_TYPE_VCP4114:
		case SENSOR_TYPE_1117:
		case SENSOR_TYPE_1123:
		case SENSOR_TYPE_1127:
		case SENSOR_TYPE_1130_PH:
		case SENSOR_TYPE_1130_ORP:
		case SENSOR_TYPE_1132:
		case SENSOR_TYPE_1133:
		case SENSOR_TYPE_1135:
		case SENSOR_TYPE_1142:
		case SENSOR_TYPE_1143:
		case SENSOR_TYPE_3500:
		case SENSOR_TYPE_3501:
		case SENSOR_TYPE_3502:
		case SENSOR_TYPE_3503:
		case SENSOR_TYPE_3507:
		case SENSOR_TYPE_3508:
		case SENSOR_TYPE_3509:
		case SENSOR_TYPE_1114:
		case SENSOR_TYPE_3511:
		case SENSOR_TYPE_3512:
		case SENSOR_TYPE_3513:
		case SENSOR_TYPE_3514:
		case SENSOR_TYPE_3515:
		case SENSOR_TYPE_3516:
		case SENSOR_TYPE_3517:
		case SENSOR_TYPE_3518:
		case SENSOR_TYPE_3519:
		case SENSOR_TYPE_3584:
		case SENSOR_TYPE_3585:
		case SENSOR_TYPE_3586:
		case SENSOR_TYPE_3587:
		case SENSOR_TYPE_3588:
		case SENSOR_TYPE_3589:
		case SENSOR_TYPE_MOT2002_LOW:
		case SENSOR_TYPE_MOT2002_MED:
		case SENSOR_TYPE_MOT2002_HIGH:
		case SENSOR_TYPE_3510:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1202_VOLTAGEINPUT_120:
		switch (val) {
		case SENSOR_TYPE_VOLTAGE:
		case SENSOR_TYPE_VCP4114:
		case SENSOR_TYPE_1117:
		case SENSOR_TYPE_1123:
		case SENSOR_TYPE_1127:
		case SENSOR_TYPE_1130_PH:
		case SENSOR_TYPE_1130_ORP:
		case SENSOR_TYPE_1132:
		case SENSOR_TYPE_1133:
		case SENSOR_TYPE_1135:
		case SENSOR_TYPE_1142:
		case SENSOR_TYPE_1143:
		case SENSOR_TYPE_3500:
		case SENSOR_TYPE_3501:
		case SENSOR_TYPE_3502:
		case SENSOR_TYPE_3503:
		case SENSOR_TYPE_3507:
		case SENSOR_TYPE_3508:
		case SENSOR_TYPE_3509:
		case SENSOR_TYPE_1114:
		case SENSOR_TYPE_3511:
		case SENSOR_TYPE_3512:
		case SENSOR_TYPE_3513:
		case SENSOR_TYPE_3514:
		case SENSOR_TYPE_3515:
		case SENSOR_TYPE_3516:
		case SENSOR_TYPE_3517:
		case SENSOR_TYPE_3518:
		case SENSOR_TYPE_3519:
		case SENSOR_TYPE_3584:
		case SENSOR_TYPE_3585:
		case SENSOR_TYPE_3586:
		case SENSOR_TYPE_3587:
		case SENSOR_TYPE_3588:
		case SENSOR_TYPE_3589:
		case SENSOR_TYPE_MOT2002_LOW:
		case SENSOR_TYPE_MOT2002_MED:
		case SENSOR_TYPE_MOT2002_HIGH:
		case SENSOR_TYPE_3510:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1202_VOLTAGEINPUT_300:
		switch (val) {
		case SENSOR_TYPE_VOLTAGE:
		case SENSOR_TYPE_VCP4114:
		case SENSOR_TYPE_1117:
		case SENSOR_TYPE_1123:
		case SENSOR_TYPE_1127:
		case SENSOR_TYPE_1130_PH:
		case SENSOR_TYPE_1130_ORP:
		case SENSOR_TYPE_1132:
		case SENSOR_TYPE_1133:
		case SENSOR_TYPE_1135:
		case SENSOR_TYPE_1142:
		case SENSOR_TYPE_1143:
		case SENSOR_TYPE_3500:
		case SENSOR_TYPE_3501:
		case SENSOR_TYPE_3502:
		case SENSOR_TYPE_3503:
		case SENSOR_TYPE_3507:
		case SENSOR_TYPE_3508:
		case SENSOR_TYPE_3509:
		case SENSOR_TYPE_1114:
		case SENSOR_TYPE_3511:
		case SENSOR_TYPE_3512:
		case SENSOR_TYPE_3513:
		case SENSOR_TYPE_3514:
		case SENSOR_TYPE_3515:
		case SENSOR_TYPE_3516:
		case SENSOR_TYPE_3517:
		case SENSOR_TYPE_3518:
		case SENSOR_TYPE_3519:
		case SENSOR_TYPE_3584:
		case SENSOR_TYPE_3585:
		case SENSOR_TYPE_3586:
		case SENSOR_TYPE_3587:
		case SENSOR_TYPE_3588:
		case SENSOR_TYPE_3589:
		case SENSOR_TYPE_MOT2002_LOW:
		case SENSOR_TYPE_MOT2002_MED:
		case SENSOR_TYPE_MOT2002_HIGH:
		case SENSOR_TYPE_3510:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_HUB_VOLTAGEINPUT_100:
		switch (val) {
		case SENSOR_TYPE_VOLTAGE:
		case SENSOR_TYPE_VCP4114:
		case SENSOR_TYPE_1117:
		case SENSOR_TYPE_1123:
		case SENSOR_TYPE_1127:
		case SENSOR_TYPE_1130_PH:
		case SENSOR_TYPE_1130_ORP:
		case SENSOR_TYPE_1132:
		case SENSOR_TYPE_1133:
		case SENSOR_TYPE_1135:
		case SENSOR_TYPE_1142:
		case SENSOR_TYPE_1143:
		case SENSOR_TYPE_3500:
		case SENSOR_TYPE_3501:
		case SENSOR_TYPE_3502:
		case SENSOR_TYPE_3503:
		case SENSOR_TYPE_3507:
		case SENSOR_TYPE_3508:
		case SENSOR_TYPE_3509:
		case SENSOR_TYPE_1114:
		case SENSOR_TYPE_3511:
		case SENSOR_TYPE_3512:
		case SENSOR_TYPE_3513:
		case SENSOR_TYPE_3514:
		case SENSOR_TYPE_3515:
		case SENSOR_TYPE_3516:
		case SENSOR_TYPE_3517:
		case SENSOR_TYPE_3518:
		case SENSOR_TYPE_3519:
		case SENSOR_TYPE_3584:
		case SENSOR_TYPE_3585:
		case SENSOR_TYPE_3586:
		case SENSOR_TYPE_3587:
		case SENSOR_TYPE_3588:
		case SENSOR_TYPE_3589:
		case SENSOR_TYPE_MOT2002_LOW:
		case SENSOR_TYPE_MOT2002_MED:
		case SENSOR_TYPE_MOT2002_HIGH:
		case SENSOR_TYPE_3510:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_HUB_VOLTAGEINPUT_110:
		switch (val) {
		case SENSOR_TYPE_VOLTAGE:
		case SENSOR_TYPE_VCP4114:
		case SENSOR_TYPE_1117:
		case SENSOR_TYPE_1123:
		case SENSOR_TYPE_1127:
		case SENSOR_TYPE_1130_PH:
		case SENSOR_TYPE_1130_ORP:
		case SENSOR_TYPE_1132:
		case SENSOR_TYPE_1133:
		case SENSOR_TYPE_1135:
		case SENSOR_TYPE_1142:
		case SENSOR_TYPE_1143:
		case SENSOR_TYPE_3500:
		case SENSOR_TYPE_3501:
		case SENSOR_TYPE_3502:
		case SENSOR_TYPE_3503:
		case SENSOR_TYPE_3507:
		case SENSOR_TYPE_3508:
		case SENSOR_TYPE_3509:
		case SENSOR_TYPE_1114:
		case SENSOR_TYPE_3511:
		case SENSOR_TYPE_3512:
		case SENSOR_TYPE_3513:
		case SENSOR_TYPE_3514:
		case SENSOR_TYPE_3515:
		case SENSOR_TYPE_3516:
		case SENSOR_TYPE_3517:
		case SENSOR_TYPE_3518:
		case SENSOR_TYPE_3519:
		case SENSOR_TYPE_3584:
		case SENSOR_TYPE_3585:
		case SENSOR_TYPE_3586:
		case SENSOR_TYPE_3587:
		case SENSOR_TYPE_3588:
		case SENSOR_TYPE_3589:
		case SENSOR_TYPE_MOT2002_LOW:
		case SENSOR_TYPE_MOT2002_MED:
		case SENSOR_TYPE_MOT2002_HIGH:
		case SENSOR_TYPE_3510:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_ADP1000_VOLTAGEINPUT_100:
		switch (val) {
		case SENSOR_TYPE_VOLTAGE:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DAQ1000_VOLTAGEINPUT_100:
		switch (val) {
		case SENSOR_TYPE_VOLTAGE:
		case SENSOR_TYPE_VCP4114:
		case SENSOR_TYPE_1117:
		case SENSOR_TYPE_1123:
		case SENSOR_TYPE_1127:
		case SENSOR_TYPE_1130_PH:
		case SENSOR_TYPE_1130_ORP:
		case SENSOR_TYPE_1132:
		case SENSOR_TYPE_1133:
		case SENSOR_TYPE_1135:
		case SENSOR_TYPE_1142:
		case SENSOR_TYPE_1143:
		case SENSOR_TYPE_3500:
		case SENSOR_TYPE_3501:
		case SENSOR_TYPE_3502:
		case SENSOR_TYPE_3503:
		case SENSOR_TYPE_3507:
		case SENSOR_TYPE_3508:
		case SENSOR_TYPE_3509:
		case SENSOR_TYPE_1114:
		case SENSOR_TYPE_3511:
		case SENSOR_TYPE_3512:
		case SENSOR_TYPE_3513:
		case SENSOR_TYPE_3514:
		case SENSOR_TYPE_3515:
		case SENSOR_TYPE_3516:
		case SENSOR_TYPE_3517:
		case SENSOR_TYPE_3518:
		case SENSOR_TYPE_3519:
		case SENSOR_TYPE_3584:
		case SENSOR_TYPE_3585:
		case SENSOR_TYPE_3586:
		case SENSOR_TYPE_3587:
		case SENSOR_TYPE_3588:
		case SENSOR_TYPE_3589:
		case SENSOR_TYPE_MOT2002_LOW:
		case SENSOR_TYPE_MOT2002_MED:
		case SENSOR_TYPE_MOT2002_HIGH:
		case SENSOR_TYPE_3510:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DAQ1000_VOLTAGEINPUT_110:
		switch (val) {
		case SENSOR_TYPE_VOLTAGE:
		case SENSOR_TYPE_VCP4114:
		case SENSOR_TYPE_1117:
		case SENSOR_TYPE_1123:
		case SENSOR_TYPE_1127:
		case SENSOR_TYPE_1130_PH:
		case SENSOR_TYPE_1130_ORP:
		case SENSOR_TYPE_1132:
		case SENSOR_TYPE_1133:
		case SENSOR_TYPE_1135:
		case SENSOR_TYPE_1142:
		case SENSOR_TYPE_1143:
		case SENSOR_TYPE_3500:
		case SENSOR_TYPE_3501:
		case SENSOR_TYPE_3502:
		case SENSOR_TYPE_3503:
		case SENSOR_TYPE_3507:
		case SENSOR_TYPE_3508:
		case SENSOR_TYPE_3509:
		case SENSOR_TYPE_1114:
		case SENSOR_TYPE_3511:
		case SENSOR_TYPE_3512:
		case SENSOR_TYPE_3513:
		case SENSOR_TYPE_3514:
		case SENSOR_TYPE_3515:
		case SENSOR_TYPE_3516:
		case SENSOR_TYPE_3517:
		case SENSOR_TYPE_3518:
		case SENSOR_TYPE_3519:
		case SENSOR_TYPE_3584:
		case SENSOR_TYPE_3585:
		case SENSOR_TYPE_3586:
		case SENSOR_TYPE_3587:
		case SENSOR_TYPE_3588:
		case SENSOR_TYPE_3589:
		case SENSOR_TYPE_MOT2002_LOW:
		case SENSOR_TYPE_MOT2002_MED:
		case SENSOR_TYPE_MOT2002_HIGH:
		case SENSOR_TYPE_3510:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DAQ1400_VOLTAGEINPUT_100:
		switch (val) {
		case SENSOR_TYPE_VOLTAGE:
		case SENSOR_TYPE_3507:
		case SENSOR_TYPE_3508:
		case SENSOR_TYPE_3509:
		case SENSOR_TYPE_3511:
		case SENSOR_TYPE_3512:
		case SENSOR_TYPE_3513:
		case SENSOR_TYPE_3514:
		case SENSOR_TYPE_3515:
		case SENSOR_TYPE_3516:
		case SENSOR_TYPE_3517:
		case SENSOR_TYPE_3518:
		case SENSOR_TYPE_3519:
		case SENSOR_TYPE_3584:
		case SENSOR_TYPE_3510:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DAQ1400_VOLTAGEINPUT_120:
		switch (val) {
		case SENSOR_TYPE_VOLTAGE:
		case SENSOR_TYPE_3507:
		case SENSOR_TYPE_3508:
		case SENSOR_TYPE_3509:
		case SENSOR_TYPE_3511:
		case SENSOR_TYPE_3512:
		case SENSOR_TYPE_3513:
		case SENSOR_TYPE_3514:
		case SENSOR_TYPE_3515:
		case SENSOR_TYPE_3516:
		case SENSOR_TYPE_3517:
		case SENSOR_TYPE_3518:
		case SENSOR_TYPE_3519:
		case SENSOR_TYPE_3584:
		case SENSOR_TYPE_3510:
			return (1);
		default:
			return (0);
		}
	default:
		return (0);
	}
}

int
supportedRFIDProtocol(PhidgetChannelHandle ch, PhidgetRFID_Protocol val) {

	switch (ch->UCD->uid) {
	case PHIDCHUID_1023_RFID_000:
		switch (val) {
		case PROTOCOL_EM4100:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1023_RFID_104:
		switch (val) {
		case PROTOCOL_EM4100:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1023_RFID_200:
		switch (val) {
		case PROTOCOL_EM4100:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1023_RFID_201:
		switch (val) {
		case PROTOCOL_EM4100:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1024_RFID_100:
		switch (val) {
		case PROTOCOL_EM4100:
		case PROTOCOL_ISO11785_FDX_B:
		case PROTOCOL_PHIDGETS:
			return (1);
		default:
			return (0);
		}
	default:
		return (0);
	}
}

int
supportedSpatialAlgorithm(PhidgetChannelHandle ch, Phidget_SpatialAlgorithm val) {

	switch (ch->UCD->uid) {
	case PHIDCHUID_1042_SPATIAL_300:
		switch (val) {
		case SPATIAL_ALGORITHM_NONE:
		case SPATIAL_ALGORITHM_AHRS:
		case SPATIAL_ALGORITHM_IMU:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1044_SPATIAL_400:
		switch (val) {
		case SPATIAL_ALGORITHM_NONE:
		case SPATIAL_ALGORITHM_AHRS:
		case SPATIAL_ALGORITHM_IMU:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1044_SPATIAL_500:
		switch (val) {
		case SPATIAL_ALGORITHM_NONE:
		case SPATIAL_ALGORITHM_AHRS:
		case SPATIAL_ALGORITHM_IMU:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1044_SPATIAL_510:
		switch (val) {
		case SPATIAL_ALGORITHM_NONE:
		case SPATIAL_ALGORITHM_AHRS:
		case SPATIAL_ALGORITHM_IMU:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1056_SPATIAL_000:
		switch (val) {
		case SPATIAL_ALGORITHM_NONE:
		case SPATIAL_ALGORITHM_AHRS:
		case SPATIAL_ALGORITHM_IMU:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1056_SPATIAL_200:
		switch (val) {
		case SPATIAL_ALGORITHM_NONE:
		case SPATIAL_ALGORITHM_AHRS:
		case SPATIAL_ALGORITHM_IMU:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_MOT1101_SPATIAL_100:
		switch (val) {
		case SPATIAL_ALGORITHM_NONE:
		case SPATIAL_ALGORITHM_AHRS:
		case SPATIAL_ALGORITHM_IMU:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_MOT1102_SPATIAL_200:
		switch (val) {
		case SPATIAL_ALGORITHM_NONE:
		case SPATIAL_ALGORITHM_AHRS:
		case SPATIAL_ALGORITHM_IMU:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_MOT0109_SPATIAL_100:
		switch (val) {
		case SPATIAL_ALGORITHM_NONE:
		case SPATIAL_ALGORITHM_AHRS:
		case SPATIAL_ALGORITHM_IMU:
			return (1);
		default:
			return (0);
		}
	default:
		return (0);
	}
}

int
supportedRTDType(PhidgetChannelHandle ch, PhidgetTemperatureSensor_RTDType val) {

	switch (ch->UCD->uid) {
	case PHIDCHUID_TMP1200_TEMPERATURESENSOR_RTD_100:
		switch (val) {
		case RTD_TYPE_PT100_3850:
		case RTD_TYPE_PT1000_3850:
		case RTD_TYPE_PT100_3920:
		case RTD_TYPE_PT1000_3920:
			return (1);
		default:
			return (0);
		}
	default:
		return (0);
	}
}

int
supportedThermocoupleType(PhidgetChannelHandle ch, PhidgetTemperatureSensor_ThermocoupleType val) {

	switch (ch->UCD->uid) {
	case PHIDCHUID_1048_TEMPERATURESENSOR_THERMOCOUPLE_100:
		switch (val) {
		case THERMOCOUPLE_TYPE_J:
		case THERMOCOUPLE_TYPE_K:
		case THERMOCOUPLE_TYPE_E:
		case THERMOCOUPLE_TYPE_T:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1048_TEMPERATURESENSOR_THERMOCOUPLE_200:
		switch (val) {
		case THERMOCOUPLE_TYPE_J:
		case THERMOCOUPLE_TYPE_K:
		case THERMOCOUPLE_TYPE_E:
		case THERMOCOUPLE_TYPE_T:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1051_TEMPERATURESENSOR_THERMOCOUPLE_000:
		switch (val) {
		case THERMOCOUPLE_TYPE_K:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1051_TEMPERATURESENSOR_THERMOCOUPLE_200:
		switch (val) {
		case THERMOCOUPLE_TYPE_J:
		case THERMOCOUPLE_TYPE_K:
		case THERMOCOUPLE_TYPE_E:
		case THERMOCOUPLE_TYPE_T:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1051_TEMPERATURESENSOR_THERMOCOUPLE_300:
		switch (val) {
		case THERMOCOUPLE_TYPE_J:
		case THERMOCOUPLE_TYPE_K:
		case THERMOCOUPLE_TYPE_E:
		case THERMOCOUPLE_TYPE_T:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1051_TEMPERATURESENSOR_THERMOCOUPLE_400:
		switch (val) {
		case THERMOCOUPLE_TYPE_J:
		case THERMOCOUPLE_TYPE_K:
		case THERMOCOUPLE_TYPE_E:
		case THERMOCOUPLE_TYPE_T:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_TMP1100_TEMPERATURESENSOR_THERMOCOUPLE_100:
		switch (val) {
		case THERMOCOUPLE_TYPE_J:
		case THERMOCOUPLE_TYPE_K:
		case THERMOCOUPLE_TYPE_E:
		case THERMOCOUPLE_TYPE_T:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_TMP1101_TEMPERATURESENSOR_THERMOCOUPLE_100:
		switch (val) {
		case THERMOCOUPLE_TYPE_J:
		case THERMOCOUPLE_TYPE_K:
		case THERMOCOUPLE_TYPE_E:
		case THERMOCOUPLE_TYPE_T:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_TMP1101_TEMPERATURESENSOR_THERMOCOUPLE_200:
		switch (val) {
		case THERMOCOUPLE_TYPE_J:
		case THERMOCOUPLE_TYPE_K:
		case THERMOCOUPLE_TYPE_E:
		case THERMOCOUPLE_TYPE_T:
			return (1);
		default:
			return (0);
		}
	default:
		return (0);
	}
}

int
supportedFilterType(PhidgetChannelHandle ch, PhidgetFrequencyCounter_FilterType val) {

	switch (ch->UCD->uid) {
	case PHIDCHUID_1054_FREQUENCYCOUNTER_000:
		switch (val) {
		case FILTER_TYPE_ZERO_CROSSING:
		case FILTER_TYPE_LOGIC_LEVEL:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DAQ1400_FREQUENCYCOUNTER_100:
		switch (val) {
		case FILTER_TYPE_LOGIC_LEVEL:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_DAQ1400_FREQUENCYCOUNTER_120:
		switch (val) {
		case FILTER_TYPE_LOGIC_LEVEL:
			return (1);
		default:
			return (0);
		}
	default:
		return (0);
	}
}

int
supportedEncoding(PhidgetChannelHandle ch, PhidgetIR_Encoding val) {

	switch (ch->UCD->uid) {
	default:
		return (0);
	}
}

int
supportedLength(PhidgetChannelHandle ch, PhidgetIR_Length val) {

	switch (ch->UCD->uid) {
	default:
		return (0);
	}
}

int
supportedControlMode(PhidgetChannelHandle ch, PhidgetStepper_ControlMode val) {

	switch (ch->UCD->uid) {
	case PHIDCHUID_1062_STEPPER_100:
		switch (val) {
		case CONTROL_MODE_STEP:
		case CONTROL_MODE_RUN:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1063_STEPPER_100:
		switch (val) {
		case CONTROL_MODE_STEP:
		case CONTROL_MODE_RUN:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1067_STEPPER_200:
		switch (val) {
		case CONTROL_MODE_STEP:
		case CONTROL_MODE_RUN:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1067_STEPPER_300:
		switch (val) {
		case CONTROL_MODE_STEP:
		case CONTROL_MODE_RUN:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_STC1000_STEPPER_100:
		switch (val) {
		case CONTROL_MODE_STEP:
		case CONTROL_MODE_RUN:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_STC1000_STEPPER_110:
		switch (val) {
		case CONTROL_MODE_STEP:
		case CONTROL_MODE_RUN:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_STC1001_STEPPER_100:
		switch (val) {
		case CONTROL_MODE_STEP:
		case CONTROL_MODE_RUN:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_STC1001_STEPPER_110:
		switch (val) {
		case CONTROL_MODE_STEP:
		case CONTROL_MODE_RUN:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_STC1002_STEPPER_100:
		switch (val) {
		case CONTROL_MODE_STEP:
		case CONTROL_MODE_RUN:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_STC1002_STEPPER_110:
		switch (val) {
		case CONTROL_MODE_STEP:
		case CONTROL_MODE_RUN:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_STC1003_STEPPER_100:
		switch (val) {
		case CONTROL_MODE_STEP:
		case CONTROL_MODE_RUN:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_STC1003_STEPPER_110:
		switch (val) {
		case CONTROL_MODE_STEP:
		case CONTROL_MODE_RUN:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_STC1003_STEPPER_200:
		switch (val) {
		case CONTROL_MODE_STEP:
		case CONTROL_MODE_RUN:
			return (1);
		default:
			return (0);
		}
	default:
		return (0);
	}
}

int
supportedLCDFont(PhidgetChannelHandle ch, PhidgetLCD_Font val) {

	switch (ch->UCD->uid) {
	case PHIDCHUID_1202_TEXTLCD_000:
		switch (val) {
		case FONT_5x8:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1202_TEXTLCD_200:
		switch (val) {
		case FONT_5x8:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1204_TEXTLCD_000:
		switch (val) {
		case FONT_5x8:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1215_TEXTLCD_000:
		switch (val) {
		case FONT_5x8:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1219_TEXTLCD_000:
		switch (val) {
		case FONT_5x8:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_LCD1100_LCD_100:
		switch (val) {
		case FONT_User1:
		case FONT_User2:
		case FONT_6x10:
		case FONT_5x8:
		case FONT_6x12:
			return (1);
		default:
			return (0);
		}
	default:
		return (0);
	}
}

int
supportedLCDScreenSize(PhidgetChannelHandle ch, PhidgetLCD_ScreenSize val) {

	switch (ch->UCD->uid) {
	case PHIDCHUID_1202_TEXTLCD_000:
		switch (val) {
		case SCREEN_SIZE_2x20:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1202_TEXTLCD_200:
		switch (val) {
		case SCREEN_SIZE_2x20:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1204_TEXTLCD_000:
		switch (val) {
		case SCREEN_SIZE_NONE:
		case SCREEN_SIZE_2x8:
		case SCREEN_SIZE_1x16:
		case SCREEN_SIZE_2x16:
		case SCREEN_SIZE_4x16:
		case SCREEN_SIZE_1x8:
		case SCREEN_SIZE_4x20:
		case SCREEN_SIZE_2x24:
		case SCREEN_SIZE_1x40:
		case SCREEN_SIZE_2x40:
		case SCREEN_SIZE_4x40:
		case SCREEN_SIZE_2x20:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1215_TEXTLCD_000:
		switch (val) {
		case SCREEN_SIZE_2x20:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_1219_TEXTLCD_000:
		switch (val) {
		case SCREEN_SIZE_2x20:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_LCD1100_LCD_100:
		switch (val) {
		case SCREEN_SIZE_64x128:
			return (1);
		default:
			return (0);
		}
	default:
		return (0);
	}
}

int
supportedLCDPixelState(PhidgetChannelHandle ch, PhidgetLCD_PixelState val) {

	switch (ch->UCD->uid) {
	case PHIDCHUID_LCD1100_LCD_100:
		switch (val) {
		case PIXEL_STATE_OFF:
		case PIXEL_STATE_ON:
		case PIXEL_STATE_INVERT:
			return (1);
		default:
			return (0);
		}
	default:
		return (0);
	}
}

int
supportedDataAdapterParity(PhidgetChannelHandle ch, PhidgetDataAdapter_Parity val) {

	switch (ch->UCD->uid) {
	default:
		return (0);
	}
}

int
supportedDataAdapterStopBits(PhidgetChannelHandle ch, PhidgetDataAdapter_StopBits val) {

	switch (ch->UCD->uid) {
	default:
		return (0);
	}
}

int
supportedDataAdapterHandshakeMode(PhidgetChannelHandle ch, PhidgetDataAdapter_HandshakeMode val) {

	switch (ch->UCD->uid) {
	default:
		return (0);
	}
}

int
supportedDataAdapterProtocol(PhidgetChannelHandle ch, PhidgetDataAdapter_Protocol val) {

	switch (ch->UCD->uid) {
	default:
		return (0);
	}
}

int
supportedDataAdapterSPIMode(PhidgetChannelHandle ch, PhidgetDataAdapter_SPIMode val) {

	switch (ch->UCD->uid) {
	default:
		return (0);
	}
}

int
supportedDataAdapterEndianness(PhidgetChannelHandle ch, PhidgetDataAdapter_Endianness val) {

	switch (ch->UCD->uid) {
	default:
		return (0);
	}
}

int
supportedDataAdapterIOVoltage(PhidgetChannelHandle ch, PhidgetDataAdapter_IOVoltage val) {

	switch (ch->UCD->uid) {
	default:
		return (0);
	}
}

int
supportedPacketErrorCode(PhidgetChannelHandle ch, PhidgetDataAdapter_PacketErrorCode val) {

	switch (ch->UCD->uid) {
	default:
		return (0);
	}
}

int
supportedSPLRange(PhidgetChannelHandle ch, PhidgetSoundSensor_SPLRange val) {

	switch (ch->UCD->uid) {
	case PHIDCHUID_SND1000_SOUNDSENSOR_100:
		switch (val) {
		case SPL_RANGE_102dB:
		case SPL_RANGE_82dB:
			return (1);
		default:
			return (0);
		}
	default:
		return (0);
	}
}

int
supportedPortMode(PhidgetChannelHandle ch, PhidgetHub_PortMode val) {

	switch (ch->UCD->uid) {
	case PHIDCHUID_HUB0000_HUB_100:
		switch (val) {
		case PORT_MODE_VINT_PORT:
		case PORT_MODE_DIGITAL_INPUT:
		case PORT_MODE_DIGITAL_OUTPUT:
		case PORT_MODE_VOLTAGE_INPUT:
		case PORT_MODE_VOLTAGE_RATIO_INPUT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_HUB0000_HUB_300:
		switch (val) {
		case PORT_MODE_VINT_PORT:
		case PORT_MODE_DIGITAL_INPUT:
		case PORT_MODE_DIGITAL_OUTPUT:
		case PORT_MODE_VOLTAGE_INPUT:
		case PORT_MODE_VOLTAGE_RATIO_INPUT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_HUB0001_HUB_100:
		switch (val) {
		case PORT_MODE_VINT_PORT:
		case PORT_MODE_DIGITAL_INPUT:
		case PORT_MODE_DIGITAL_OUTPUT:
		case PORT_MODE_VOLTAGE_INPUT:
		case PORT_MODE_VOLTAGE_RATIO_INPUT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_HUB0004_HUB_100:
		switch (val) {
		case PORT_MODE_VINT_PORT:
		case PORT_MODE_DIGITAL_INPUT:
		case PORT_MODE_DIGITAL_OUTPUT:
		case PORT_MODE_VOLTAGE_INPUT:
		case PORT_MODE_VOLTAGE_RATIO_INPUT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_HUB0004_HUB_200:
		switch (val) {
		case PORT_MODE_VINT_PORT:
		case PORT_MODE_DIGITAL_INPUT:
		case PORT_MODE_DIGITAL_OUTPUT:
		case PORT_MODE_VOLTAGE_INPUT:
		case PORT_MODE_VOLTAGE_RATIO_INPUT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_HUB5000_HUB_100:
		switch (val) {
		case PORT_MODE_VINT_PORT:
		case PORT_MODE_DIGITAL_INPUT:
		case PORT_MODE_DIGITAL_OUTPUT:
		case PORT_MODE_VOLTAGE_INPUT:
		case PORT_MODE_VOLTAGE_RATIO_INPUT:
			return (1);
		default:
			return (0);
		}
	case PHIDCHUID_HUB5000_HUB_200:
		switch (val) {
		case PORT_MODE_VINT_PORT:
		case PORT_MODE_DIGITAL_INPUT:
		case PORT_MODE_DIGITAL_OUTPUT:
		case PORT_MODE_VOLTAGE_INPUT:
		case PORT_MODE_VOLTAGE_RATIO_INPUT:
			return (1);
		default:
			return (0);
		}
	default:
		return (0);
	}
}
