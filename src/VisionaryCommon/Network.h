//
// Copyright note: Redistribution and use in source, with or without modification, are permitted.
// 
// Created: September 2017
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

#include <stdint.h>
#include <vector>

// Include socket
#ifdef _WIN32    // Windows specific
#include <winsock2.h>
#include <Ws2tcpip.h>
// to use with other compiler than Visual C++ need to set Linker flag -lws2_32
#ifdef _MSC_VER
#pragma comment(lib,"ws2_32.lib")
#endif
#else        // Linux specific
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

typedef int SOCKET;
#define INVALID_SOCKET  ((SOCKET)(~0))
#define SOCKET_ERROR    (-1)
#endif

class Network
{
public:
  Network(unsigned short port);
  Network(unsigned long ipAddress, unsigned short port);
  ~Network();

  void setIpAddress(unsigned long ipAddress);
  void setIpAddress(char* ipAddress);
  unsigned long getIpAddress();
  void setPort(unsigned short port);
  unsigned short getPort();

  //-----------------------------------------------
  // Functions to handle connection with device
  bool openConnection();
  bool openUDPSocket();
  void closeConnection();

protected:
  // Send data on socket to device
  int write(const std::vector<char> &buffer);
  int write(const std::vector<uint8_t> &buffer);
  bool syncCoLa() const;
  bool receiveData(uint32_t maxBytesToReceive, uint32_t& receivedBytes, char* pBuffer);
  bool receiveData(uint32_t bytesToReceive, std::vector<char>& buffer);
  bool receiveData(uint32_t bytesToReceive, std::vector<uint8_t>& buffer);
  bool receiveData(uint32_t bytesToReceive, char* pBuffer);
  void printLastErrorMessage() const;
private:
  //-----------------------------------------------
  // Connection handling
  static const unsigned long DEFAULT_IP = 0x0A01A8C0;    // Network byte ordered long value for IP address: 192.168.1.10

  unsigned short m_port;
  unsigned long m_ipAddress;

  bool         m_isUDP;

  SOCKET m_recvSocket;

  struct sockaddr_in m_udpAddr;
};
