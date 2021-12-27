//
// Copyright note: Redistribution and use in source, with or without modification, are permitted.
// 
// Created: August 2017
// 
// @author:  Andreas Richert
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
#include "CoLaBCommand.h"
#include "CoLaUserLevel.h"

class VisionaryControl : public Network
{
public:
  VisionaryControl();
  VisionaryControl(unsigned long ipAddress);
  VisionaryControl(unsigned long ipAddress, unsigned short port);
  ~VisionaryControl();

  /// <summary>Login to the device.</summary>
  /// <param name="userLevel">The user level to login as.</param>
  /// <param name="password">Password for the selected user level.</param>
  /// <returns>True if login was successful, false otherwise.</returns>
  bool login(CoLaUserLevel::Enum userLevel, const char* password);

  /// <summary>Logout from the device.</summary>
  /// <returns>True if logout was successful, false otherwise.</returns>
  bool logout();

  /// <summary>
  /// Start streaming the data by calling the "PLAYSTART" method on the device. Works only when acquisition is stopped.
  /// </summary>
  /// <returns>True if successful, false otherwise.</returns>
  bool startAcquisition();

  /// <summary>
  /// Trigger a single image on the device. Works only when acquisition is stopped.
  /// </summary>
  /// <returns>True if successful, false otherwise.</returns>
  bool stepAcquisition();

  /// <summary>
  /// Stops the data stream. Works always, also when acquisition is already stopped before.
  /// </summary>
  bool stopAcquisition();

  /// <summary>
  /// Tells the device that there is a streaming channel by invoking a method named GetBlobClientConfig.
  /// </summary>
  /// <returns>True if successful, false otherwise.</returns>
  bool getDataStreamConfig();

  /// <summary>Send a <see cref="CoLaBCommand" /> to the device and waits for the result.</summary>
  /// <param name="command">Command to send</param>
  /// <returns>The response.</returns>
  CoLaBCommand sendCommand(CoLaBCommand& command);
  
private:
  std::string receiveCoLaResponse();
  CoLaBCommand receiveCoLaBCommand();

  // Network byte ordered short value for default configuration port: 2114
  static const short DEFAULT_PORT = 0x4008;
};
