//
// Copyright note: Redistribution and use in source, with or without modification, are permitted.
// 
// Created: August 2017
// 
// @author:  Andreas Richert
// SICK AG, Waldkirch
// email: TechSupport0905@sick.de
// 
// Last commit: $Date: 2017-12-06 16:56:03 +0100 (Mi, 06 Dez 2017) $
// Last editor: $Author: richean $
// 
// Version "$Revision: 15144 $"
//

#pragma once

#include <memory>
#include <boost/shared_ptr.hpp>
#include "VisionaryData.h"
#include "Network.h"

class VisionaryDataStream: public Network
{
public:
  VisionaryDataStream(boost::shared_ptr<VisionaryData> dataHandler);
  VisionaryDataStream(boost::shared_ptr<VisionaryData> dataHandler, unsigned long ipAddress);
  VisionaryDataStream(boost::shared_ptr<VisionaryData> dataHandler, unsigned long ipAddress, unsigned short port);
  ~VisionaryDataStream();

  //-----------------------------------------------
  // Receive a single blob from the connected device and store it in buffer.
  // Returns true when valid frame completely received.
  bool getNextFrame();
private:
  // Network byte ordered long value for default data stream port: 2114
  static const long DEFAULT_PORT = 0x4208;


  boost::shared_ptr<VisionaryData> m_dataHandler;

  // Parse the Segment-Binary-Data (Blob data without protocol version and packet type).
  // Returns true when parsing was successful.
  bool parseSegmentBinaryData(const std::vector<char>::iterator itBuf);
};
