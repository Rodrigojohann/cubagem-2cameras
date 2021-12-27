//
// Copyright note: Redistribution and use in source, with or without modification, are permitted.
// 
// Created: October 2018
// 
// @author:  Johan Falk
// SICK AG, Waldkirch
// email: TechSupport0905@sick.de
// 
// Last commit: $Date: 2017-09-27 10:44:05 +0200 (Mi, 27 Sep 2017) $
// Last editor: $Author: falkjo $
// 
// Version "$Revision: 14104 $"
//

#include "CoLaBCommandBuilder.h"

#include "MD5.h"
#include "VisionaryEndian.h"

CoLaBCommandBuilder::CoLaBCommandBuilder(CoLaCommandType::Enum type, const char * name)
  : m_type(type)
  , m_name(name)
{
  writeHeader(m_type, m_name);
}

CoLaBCommandBuilder::~CoLaBCommandBuilder()
{
}

CoLaBCommandBuilder& CoLaBCommandBuilder::parameterSInt(const int8_t sInt)
{
  m_buffer.push_back(static_cast<uint8_t>(sInt));
  return *this;
}

CoLaBCommandBuilder& CoLaBCommandBuilder::parameterUSInt(const uint8_t uSInt)
{
  m_buffer.push_back(uSInt);
  return *this;
}

CoLaBCommandBuilder& CoLaBCommandBuilder::parameterInt(const int16_t integer)
{
  const int16_t bigEndianValue = nativeToBigEndian(integer);
  m_buffer.insert(m_buffer.end(), reinterpret_cast<const uint8_t*>(&bigEndianValue), reinterpret_cast<const uint8_t*>(&bigEndianValue) + 2);
  return *this;
}

CoLaBCommandBuilder& CoLaBCommandBuilder::parameterUInt(const uint16_t uInt)
{
  const uint16_t bigEndianValue = nativeToBigEndian(uInt);
  m_buffer.insert(m_buffer.end(), reinterpret_cast<const uint8_t*>(&bigEndianValue), reinterpret_cast<const uint8_t*>(&bigEndianValue) + 2);
  return *this;
}

CoLaBCommandBuilder& CoLaBCommandBuilder::parameterDInt(const int32_t dInt)
{
  const int32_t bigEndianValue = nativeToBigEndian(dInt);
  m_buffer.insert(m_buffer.end(), reinterpret_cast<const uint8_t*>(&bigEndianValue), reinterpret_cast<const uint8_t*>(&bigEndianValue) + 4);
  return *this;
}

CoLaBCommandBuilder& CoLaBCommandBuilder::parameterUDInt(const uint32_t uDInt)
{
  const uint32_t bigEndianValue = nativeToBigEndian(uDInt);
  m_buffer.insert(m_buffer.end(), reinterpret_cast<const uint8_t*>(&bigEndianValue), reinterpret_cast<const uint8_t*>(&bigEndianValue) + 4);
  return *this;
}

CoLaBCommandBuilder& CoLaBCommandBuilder::parameterReal(const float real)
{
  const float bigEndianValue = nativeToBigEndian(real);
  m_buffer.insert(m_buffer.end(), reinterpret_cast<const uint8_t*>(&bigEndianValue), reinterpret_cast<const uint8_t*>(&bigEndianValue) + 4);
  return *this;
}

CoLaBCommandBuilder& CoLaBCommandBuilder::parameterLReal(const double lReal)
{
  const double bigEndianValue = nativeToBigEndian(lReal);
  m_buffer.insert(m_buffer.end(), reinterpret_cast<const uint8_t*>(&bigEndianValue), reinterpret_cast<const uint8_t*>(&bigEndianValue) + 8);
  return *this;
}

CoLaBCommandBuilder& CoLaBCommandBuilder::parameterBool(const bool boolean)
{
  *this << static_cast<uint8_t>(boolean);
  return *this;
}

CoLaBCommandBuilder& CoLaBCommandBuilder::parameterPasswordMD5(const char * str)
{
  uint32_t valueUDInt = 0;
  if (str != NULL && strlen(str) > 0)
  {
    const unsigned char* byteData = MD5(str).getDigest();

    // 128 bit to 32 bit using XOR
    int byte0 = byteData[0] ^ byteData[4] ^ byteData[8] ^ byteData[12];
    int byte1 = byteData[1] ^ byteData[5] ^ byteData[9] ^ byteData[13];
    int byte2 = byteData[2] ^ byteData[6] ^ byteData[10] ^ byteData[14];
    int byte3 = byteData[3] ^ byteData[7] ^ byteData[11] ^ byteData[15];
    valueUDInt = static_cast<uint32_t>(byte0 | (byte1 << 8) | (byte2 << 16) | (byte3 << 24));
  }

  // Add as UDInt, it is already big endian
  parameterUDInt(valueUDInt);

  return *this;
}

CoLaBCommandBuilder& CoLaBCommandBuilder::operator<<(const char* str)
{
  m_buffer.insert(m_buffer.end(), str, str+strlen(str));
  return *this;
}

CoLaBCommandBuilder& CoLaBCommandBuilder::operator<<(const int8_t sInt)
{
  return parameterSInt(sInt);
}

CoLaBCommandBuilder& CoLaBCommandBuilder::operator<<(const uint8_t uSInt)
{
  return parameterUSInt(uSInt);
}

CoLaBCommandBuilder& CoLaBCommandBuilder::operator<<(const int16_t integer)
{
  return parameterInt(integer);
}

CoLaBCommandBuilder& CoLaBCommandBuilder::operator<<(const uint16_t uInt)
{
  return parameterUInt(uInt);
}

CoLaBCommandBuilder& CoLaBCommandBuilder::operator<<(const int32_t dInt)
{
  return parameterDInt(dInt);
}

CoLaBCommandBuilder& CoLaBCommandBuilder::operator<<(const uint32_t uDInt)
{
  return parameterUDInt(uDInt);
}

CoLaBCommandBuilder& CoLaBCommandBuilder::operator<<(const float real)
{
  return parameterReal(real);
}

CoLaBCommandBuilder & CoLaBCommandBuilder::operator<<(const double lReal)
{
  return parameterLReal(lReal);
}

CoLaBCommandBuilder& CoLaBCommandBuilder::operator<<(const bool boolean)
{
  return parameterBool(boolean);
}

const CoLaBCommand CoLaBCommandBuilder::build()
{
  // Copy buffer
  std::vector<uint8_t> buffer = m_buffer;

  // Overwrite length
  *reinterpret_cast<uint32_t*>(&buffer[4]) = nativeToBigEndian(static_cast<uint32_t>(buffer.size()) - 8);

  // Add checksum to end
  buffer.insert(buffer.end(), calculateChecksum(buffer));

  return CoLaBCommand(buffer);
}

void CoLaBCommandBuilder::writeHeader(CoLaCommandType::Enum type, const char* name)
{
  // Write magic word
  *this << (uint8_t)0x02;
  *this << (uint8_t)0x02;
  *this << (uint8_t)0x02;
  *this << (uint8_t)0x02;
  *this << (uint32_t)0; // Set length to 0 for now

  // Write command type
  switch (type)
  {
  case CoLaCommandType::READ_VARIABLE: *this << "sRN "; break;
  case CoLaCommandType::READ_VARIABLE_RESPONSE: *this << "sRA "; break;
  case CoLaCommandType::WRITE_VARIABLE: *this << "sWN "; break;
  case CoLaCommandType::WRITE_VARIABLE_RESPONSE: *this << "sWA "; break;
  case CoLaCommandType::METHOD_INVOCATION: *this << "sMN "; break;
  case CoLaCommandType::METHOD_RETURN_VALUE: *this << "sAN "; break;
  case CoLaCommandType::COLA_ERROR: *this << "sFA"; break;
  }

  // Write command name
  *this << name << " ";
}

uint8_t CoLaBCommandBuilder::calculateChecksum(const std::vector<uint8_t>& buffer)
{
    uint8_t checksum = 0;
    for (size_t i = 8; i < buffer.size(); i++)
    {
      checksum ^= buffer[i];
    }
    return checksum;
}
