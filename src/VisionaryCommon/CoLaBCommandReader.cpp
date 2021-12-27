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

#include "CoLaBCommandReader.h"
#include "VisionaryEndian.h"

CoLaBCommandReader::CoLaBCommandReader(CoLaBCommand command)
  : m_command(command)
{
  m_currentPosition = command.getParameterOffset();
}

CoLaBCommandReader::~CoLaBCommandReader()
{
}

void CoLaBCommandReader::rewind()
{
  m_currentPosition = m_command.getParameterOffset();
}

const int8_t CoLaBCommandReader::readSInt()
{
  const int8_t value = static_cast<const int8_t>(m_command.getBuffer()[m_currentPosition]);
  m_currentPosition += 1;
  return value;
}

const uint8_t CoLaBCommandReader::readUSInt()
{
  const uint8_t value = m_command.getBuffer()[m_currentPosition];
  m_currentPosition += 1;
  return value;
}

const int16_t CoLaBCommandReader::readInt()
{
  const int16_t value = readUnalignBigEndian<int16_t>(&m_command.getBuffer()[m_currentPosition]);
  m_currentPosition += 2;
  return value;
}

const uint16_t CoLaBCommandReader::readUInt()
{
  const uint16_t value = readUnalignBigEndian<uint16_t>(&m_command.getBuffer()[m_currentPosition]);
  m_currentPosition += 2;
  return value;
}

const int32_t CoLaBCommandReader::readDInt()
{
  const int32_t value = readUnalignBigEndian<int32_t>(&m_command.getBuffer()[m_currentPosition]);
  m_currentPosition += 4;
  return value;
}

const uint32_t CoLaBCommandReader::readUDInt()
{
  const uint32_t value = readUnalignBigEndian<uint32_t>(&m_command.getBuffer()[m_currentPosition]);
  m_currentPosition += 4;
  return value;
}

const float CoLaBCommandReader::readReal()
{
  const float value = readUnalignBigEndian<float>(&m_command.getBuffer()[m_currentPosition]);
  m_currentPosition += 4;
  return value;
}

const double CoLaBCommandReader::readLReal()
{
  const double value = readUnalignBigEndian<double>(&m_command.getBuffer()[m_currentPosition]);
  m_currentPosition += 8;
  return value;
}

const bool CoLaBCommandReader::readBool()
{
  return readUSInt() == 1;
}

std::string CoLaBCommandReader::readFlexString()
{
  std::string str(reinterpret_cast<const char*>(&m_command.getBuffer()[m_currentPosition]), readUInt());
  m_currentPosition += str.length();
  return str;
}
