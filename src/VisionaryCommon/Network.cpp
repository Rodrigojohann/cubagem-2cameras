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

#include <errno.h>
#include <stdio.h>
#include <string.h>
#include "Network.h"

Network::Network(unsigned short port) :
    m_ipAddress(DEFAULT_IP)
    , m_port(port)
{
  memset(&m_udpAddr, 0, sizeof(m_udpAddr));
}

Network::Network(unsigned long ipAddress, unsigned short port):
    m_ipAddress(ipAddress)
    , m_port(port)
{
  memset(&m_udpAddr, 0, sizeof(m_udpAddr));
}

Network::~Network()
{
}

void Network::setIpAddress(unsigned long ipAddress)
{
  m_ipAddress = ipAddress;
}

void Network::setIpAddress(char * ipAddress)
{
  m_ipAddress = inet_addr(ipAddress);
}

unsigned long Network::getIpAddress()
{
    return m_ipAddress;
}

void Network::setPort(unsigned short port)
{
  m_port = port;
}

unsigned short Network::getPort()
{
    return m_port;
}

bool Network::openConnection()
{
    int iResult = 0;
#ifdef _WIN32
    //-----------------------------------------------
    // Initialize Winsock
    WSADATA wsaData;
    iResult = WSAStartup(MAKEWORD(2, 2), &wsaData);
    if (iResult != NO_ERROR)
    {
        closeConnection();
        return false;
    }
#endif

    //-----------------------------------------------
    // Create a receiver socket to receive datagrams
    m_recvSocket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (m_recvSocket == INVALID_SOCKET) {
        closeConnection();
        return false;
    }

    //-----------------------------------------------
    // Bind the socket to any address and the specified port.
    sockaddr_in recvAddr;
    recvAddr.sin_family = AF_INET;
    recvAddr.sin_port = getPort();
    recvAddr.sin_addr.s_addr = getIpAddress();

    iResult = connect(m_recvSocket, (sockaddr *)& recvAddr, sizeof(recvAddr));
    if (iResult != 0)
    {
        closeConnection();
        return false;
    }

    // Set the timeout for the socket to 5 seconds
    long timeoutSeconds = 5L;
#ifdef _WIN32
    // On Windows timeout is a DWORD in milliseconds (https://docs.microsoft.com/en-us/windows/desktop/api/winsock/nf-winsock-setsockopt)
    long timeoutMs = timeoutSeconds * 1000L;
    iResult = setsockopt(m_recvSocket, SOL_SOCKET, SO_RCVTIMEO, (const char*)&timeoutMs, sizeof(DWORD));
#else
    struct timeval tv;
    tv.tv_sec = timeoutSeconds;  /* 5 seconds Timeout */
    tv.tv_usec = 0L;  // Not init'ing this can cause strange errors
    iResult = setsockopt(m_recvSocket, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv, sizeof(struct timeval));
#endif

    if (iResult != 0)
    {
      return false;
    }

    return true;
}


bool Network::openUDPSocket()
{
  int iResult = 0;
  int trueVal = 1;
  int timeout = 1000;
#ifdef _WIN32
  //-----------------------------------------------
  // Initialize Winsock
  WSADATA wsaData;
  iResult = WSAStartup(MAKEWORD(2, 2), &wsaData);
  if (iResult != NO_ERROR)
  {
    closeConnection();
    return false;
  }
#endif

  m_recvSocket = socket(AF_INET, SOCK_DGRAM, 0);
  if (m_recvSocket == INVALID_SOCKET) {
    closeConnection();
    return false;
  }  
  
  if (setsockopt(m_recvSocket, SOL_SOCKET, SO_RCVTIMEO, (char*)&timeout, sizeof(timeout)) < 0)
  {
    return false;
  }
  if (setsockopt(m_recvSocket, SOL_SOCKET, SO_BROADCAST, (char*)&trueVal,
    sizeof(trueVal)) < 0)
  {
    return false;
  }
  
  m_udpAddr.sin_family = AF_INET;
  m_udpAddr.sin_addr.s_addr = m_ipAddress;
  m_udpAddr.sin_port = htons(m_port);
  m_isUDP = true;
  return true;
}

void Network::closeConnection()
{
    //-----------------------------------------------
    // Close the socket when finished receiving datagrams
#ifdef _WIN32
    closesocket(m_recvSocket);
    WSACleanup();
#else
    close(m_recvSocket);
#endif
    m_recvSocket = INVALID_SOCKET;
}

int Network::write(const std::vector<char> &buffer)
{
  if (m_isUDP)
  {
    sendto(m_recvSocket, buffer.data(), (int)buffer.size(), 0,
      (struct sockaddr*) &m_udpAddr, sizeof(m_udpAddr));
  }
  else
  {
    return send(m_recvSocket, buffer.data(), (int)buffer.size(), 0);
  }
}

int Network::write(const std::vector<uint8_t> &buffer)
{
  if (m_isUDP)
  {
    sendto(m_recvSocket, reinterpret_cast<const char*>(buffer.data()), (int)buffer.size(), 0,
      (struct sockaddr*) &m_udpAddr, sizeof(m_udpAddr));
  }
  else
  { 
    return send(m_recvSocket, reinterpret_cast<const char*>(buffer.data()), (int)buffer.size(), 0);
  }
}

bool Network::syncCoLa() const
{
  size_t elements = 0;
  char buffer[1];
  int bytesReceived = 0;

  while (elements < 4)
  {
    bytesReceived = recv(m_recvSocket, buffer, 1, 0);
    if (bytesReceived == SOCKET_ERROR)
    {
      this->printLastErrorMessage();
      return false;
    }
    if (0x02 == buffer[0])
    {
      elements++;
    }
    else
    {
      elements = 0;
    }
  }

  return true;
}

bool Network::receiveData(uint32_t bytesToReceive, std::vector<char>& buffer)
{
  buffer.resize(bytesToReceive);
  return receiveData(bytesToReceive, buffer.data());
}

bool Network::receiveData(uint32_t bytesToReceive, std::vector<uint8_t>& buffer)
{
  buffer.resize(bytesToReceive);
  return receiveData(bytesToReceive, reinterpret_cast<char*>(buffer.data()));
}

bool Network::receiveData(uint32_t bytesToReceive, char* pBuffer)
{
  int bytesReceived = 0;
  while (bytesToReceive > 0)
  {
    bytesReceived = recv(m_recvSocket, pBuffer, bytesToReceive, 0);

    if (bytesReceived == SOCKET_ERROR || bytesReceived == 0)
    {
      this->printLastErrorMessage();
      return false;
    }
    pBuffer += bytesReceived;
    bytesToReceive -= bytesReceived;
  }
  pBuffer = NULL;
  return true;
}

bool Network::receiveData(uint32_t maxBytesToReceive, uint32_t& receivedBytes, char* pBuffer)
{
  receivedBytes = recv(m_recvSocket, pBuffer, maxBytesToReceive, 0);
  if (maxBytesToReceive == SOCKET_ERROR || receivedBytes == 0)
  {
    this->printLastErrorMessage();
    return false;
  }
  pBuffer = NULL;
  return true;
}

void Network::printLastErrorMessage() const
{
#ifdef _WIN32
  int errorCode = WSAGetLastError();
  wchar_t *errorMessage = NULL;
  FormatMessageW(FORMAT_MESSAGE_ALLOCATE_BUFFER | FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_IGNORE_INSERTS,
    NULL, errorCode,
    MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT),
    (LPWSTR)&errorMessage, 0, NULL);
  fprintf(stderr, "Error %d: %S\n", errorCode, errorMessage);
#else
  fprintf(stderr, "Error %d: %s\n", errno, strerror(errno));
#endif
}
