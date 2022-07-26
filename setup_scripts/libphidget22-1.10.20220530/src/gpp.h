#ifndef EXTERNALPROTO
/*
* This file is part of libphidget22
*
* Copyright 2015 Phidgets Inc <patrick@phidgets.com>
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
#endif

#ifndef __PHIDGETGPP_H
#define __PHIDGETGPP_H
#include "phidget.h"

//Some constants and function for new M3 Phidgets (General Packet Protocol)

//Bit 7 (MSB)
#define PHID_GENERAL_PACKET_FLAG 0x80

//Bit 6 (used for return value on IN stream)
#define PHID_GENERAL_PACKET_SUCCESS	0x00
#define PHID_GENERAL_PACKET_FAIL	0x40

//Bits 0-5 (0x00 - 0x1F) (up to 31 types)
#define PHID_GENERAL_PACKET_IGNORE 0x00
#define PHID_GENERAL_PACKET_REBOOT_FIRMWARE_UPGRADE	0x01
#define PHID_GENERAL_PACKET_REBOOT_ISP 0x02
#define PHID_GENERAL_PACKET_CONTINUATION 0x03	//This is more data for when we need to send more then will fit in a single packet.
#define PHID_GENERAL_PACKET_ZERO_CONFIG 0x04
#define PHID_GENERAL_PACKET_WRITE_FLASH 0x05
#define PHID_GENERAL_PACKET_FIRMWARE_UPGRADE_WRITE_SECTOR 0x06
#define PHID_GENERAL_PACKET_SET_DS_TABLE 0x07
#define PHID_GENERAL_PACKET_SET_DW_TABLE 0x08
#define PHID_GENERAL_PACKET_FIRMWARE_UPGRADE_ERASE 0x09
#define PHID_GENERAL_PACKET_ERASE_CONFIG 0x0A

// PHIDUSB device only - these can extend to 0xFF
#define PHID_GENERAL_PACKET_OPEN_RESET 0x20
#define PHID_GENERAL_PACKET_CLOSE_RESET 0x21
#define PHID_GENERAL_PACKET_FIRMWARE_UPGRADE_WRITE	0x22
#define PHID_GENERAL_PACKET_FIRMWARE_UPGRADE_DONE	0x23
#define PHID_GENERAL_PACKET_WRITE_LABEL				0x24

//Internal API
PhidgetReturnCode PhidgetGPP_reboot_firmwareUpgrade(mosiop_t iop, PhidgetChannelHandle channel);
PhidgetReturnCode PhidgetGPP_setLabel(mosiop_t iop, PhidgetChannelHandle channel, const char *buffer);
PhidgetReturnCode PhidgetGPP_upgradeFirmware(mosiop_t iop, PhidgetChannelHandle channel, const unsigned char *data, size_t length);
PhidgetReturnCode PhidgetGPP_eraseFirmware(mosiop_t iop, PhidgetChannelHandle channel);
PhidgetReturnCode PhidgetGPP_dataInput(PhidgetDeviceHandle device, unsigned char *buffer, size_t length);
BOOL deviceSupportsGeneralPacketProtocol(PhidgetDeviceHandle device);
BOOL deviceSupportsGeneralPacketProtocolDataInput(PhidgetDeviceHandle device);
PhidgetReturnCode GPP_reboot_firmwareUpgrade(mosiop_t iop, PhidgetDeviceHandle device);
PhidgetReturnCode GPP_reboot_ISP(mosiop_t iop, PhidgetDeviceHandle device);
PhidgetReturnCode GPP_setLabel(mosiop_t iop, PhidgetDeviceHandle device, const char *buffer);
PhidgetReturnCode GPP_setDeviceSpecificConfigTable(mosiop_t iop, PhidgetDeviceHandle device, const unsigned char *data, size_t length, int index);
PhidgetReturnCode GPP_setDeviceWideConfigTable(mosiop_t iop, PhidgetDeviceHandle device, const unsigned char *data, size_t length, int index);
PhidgetReturnCode GPP_upgradeFirmware(mosiop_t iop, PhidgetDeviceHandle device, const unsigned char *data, size_t length, PhidgetChannelHandle channel);
PhidgetReturnCode GPP_eraseFirmware(mosiop_t iop, PhidgetDeviceHandle device);
PhidgetReturnCode GPP_eraseConfig(mosiop_t iop, PhidgetDeviceHandle device);
PhidgetReturnCode GPP_writeFlash(mosiop_t iop, PhidgetDeviceHandle device);
PhidgetReturnCode GPP_open_reset(mosiop_t iop, PhidgetDeviceHandle device);
PhidgetReturnCode GPP_close_reset(mosiop_t iop, PhidgetDeviceHandle device);
#endif
