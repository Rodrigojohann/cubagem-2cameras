//
// Copyright note: Redistribution and use in source, with or without modification, are permitted.
// 
// Created: August 2017
// 
// @author:  Patrick Ebner
// SICK AG, Waldkirch
// email: TechSupport0905@sick.de
// 
// Last commit: $Date: 2017-12-06 17:17:50 +0100 (Mi, 06 Dez 2017) $
// Last editor: $Author: richean $
// 
// Version "$Revision: 15145 $"
//

#pragma once

#include <cstring>
#include <string>
#include "Network.h"

class VisionaryAutoIPScan : public Network
{
public:
  struct DeviceInfo
  {
    std::string DeviceName;
    std::string MacAddress;
    std::string IpAddress;
    std::string SubNet;
    std::string Port;
  };
  VisionaryAutoIPScan();
  VisionaryAutoIPScan(unsigned short port);
  VisionaryAutoIPScan(unsigned long ipAddress, unsigned short port);

  ~VisionaryAutoIPScan();

  /// <summary>
  /// Runs an autoIP scan and returns a list of devices
  /// </summary>
  /// <returns>A list of devices.</returns>
  std::vector<DeviceInfo> doScan(int timeOut);

private:
  DeviceInfo parseAutoIPXml(std::stringstream& rStringStream);
  static const short DEFAULT_PORT = 30718;
};
