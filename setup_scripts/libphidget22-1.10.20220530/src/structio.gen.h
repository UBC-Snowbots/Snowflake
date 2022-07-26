PhidgetReturnCode readUnitInfo(BridgePacket *bp, Phidget_UnitInfoHandle dst);
PhidgetReturnCode writeUnitInfo(const Phidget_UnitInfoHandle src, BridgePacket *bp);
PhidgetReturnCode readPhidgetServer(BridgePacket *bp, PhidgetServerHandle dst);
PhidgetReturnCode writePhidgetServer(const PhidgetServerHandle src, BridgePacket *bp);
PhidgetReturnCode readGPSTime(BridgePacket *bp, PhidgetGPS_TimeHandle dst);
PhidgetReturnCode writeGPSTime(const PhidgetGPS_TimeHandle src, BridgePacket *bp);
PhidgetReturnCode readGPSDate(BridgePacket *bp, PhidgetGPS_DateHandle dst);
PhidgetReturnCode writeGPSDate(const PhidgetGPS_DateHandle src, BridgePacket *bp);
PhidgetReturnCode readGPGGA(BridgePacket *bp, PhidgetGPS_GPGGAHandle dst);
PhidgetReturnCode writeGPGGA(const PhidgetGPS_GPGGAHandle src, BridgePacket *bp);
PhidgetReturnCode readGPGSA(BridgePacket *bp, PhidgetGPS_GPGSAHandle dst);
PhidgetReturnCode writeGPGSA(const PhidgetGPS_GPGSAHandle src, BridgePacket *bp);
PhidgetReturnCode readGPRMC(BridgePacket *bp, PhidgetGPS_GPRMCHandle dst);
PhidgetReturnCode writeGPRMC(const PhidgetGPS_GPRMCHandle src, BridgePacket *bp);
PhidgetReturnCode readGPVTG(BridgePacket *bp, PhidgetGPS_GPVTGHandle dst);
PhidgetReturnCode writeGPVTG(const PhidgetGPS_GPVTGHandle src, BridgePacket *bp);
PhidgetReturnCode readNMEAData(BridgePacket *bp, PhidgetGPS_NMEADataHandle dst);
PhidgetReturnCode writeNMEAData(const PhidgetGPS_NMEADataHandle src, BridgePacket *bp);
PhidgetReturnCode readSpatialQuaternion(BridgePacket *bp, PhidgetSpatial_SpatialQuaternionHandle dst);
PhidgetReturnCode writeSpatialQuaternion(const PhidgetSpatial_SpatialQuaternionHandle src,
  BridgePacket *bp);
PhidgetReturnCode readSpatialEulerAngles(BridgePacket *bp, PhidgetSpatial_SpatialEulerAnglesHandle dst);
PhidgetReturnCode writeSpatialEulerAngles(const PhidgetSpatial_SpatialEulerAnglesHandle src,
  BridgePacket *bp);
PhidgetReturnCode readCodeInfo(BridgePacket *bp, PhidgetIR_CodeInfoHandle dst);
PhidgetReturnCode writeCodeInfo(const PhidgetIR_CodeInfoHandle src, BridgePacket *bp);
