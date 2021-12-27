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

#include "CoLaBCommand.h"

#include <string>
#include "VisionaryEndian.h"

CoLaBCommand::CoLaBCommand(CoLaCommandType::Enum commandType, CoLaError::Enum error, const char* name)
  : m_buffer()
  , m_type(commandType)
  , m_error(error)
  , m_name(name)
  , m_parameterOffset(0)
{

}

CoLaBCommand::CoLaBCommand(std::vector<uint8_t> buffer)
  : m_buffer(buffer)
  , m_type(CoLaCommandType::UNKNOWN)
  , m_error(CoLaError::OK)
  , m_name("")
  , m_parameterOffset(0)
{
  // Read type from header
  std::string typeStr(reinterpret_cast<char*>(&buffer[8]), 3);
  if (typeStr.compare("sRN") == 0) m_type = CoLaCommandType::READ_VARIABLE;
  else if (typeStr.compare("sRA") == 0) m_type = CoLaCommandType::READ_VARIABLE_RESPONSE;
  else if (typeStr.compare("sWN") == 0) m_type = CoLaCommandType::WRITE_VARIABLE;
  else if (typeStr.compare("sWA") == 0) m_type = CoLaCommandType::WRITE_VARIABLE_RESPONSE;
  else if (typeStr.compare("sMN") == 0) m_type = CoLaCommandType::METHOD_INVOCATION;
  else if (typeStr.compare("sAN") == 0) m_type = CoLaCommandType::METHOD_RETURN_VALUE;
  else if (typeStr.compare("sFA") == 0) m_type = CoLaCommandType::COLA_ERROR;

  if(m_type == CoLaCommandType::COLA_ERROR)
  {
    m_parameterOffset = 4 + 4 + 3; // Magic word, length, sFA

    // Read error code
    m_error = static_cast<CoLaError::Enum>((static_cast<uint16_t>(buffer[m_parameterOffset]) << 8) | buffer[m_parameterOffset + 1]);
  }
  else if (m_type == CoLaCommandType::NETWORK_ERROR)
  {
    m_error = CoLaError::NETWORK_ERROR;
  }
  else if (m_type != CoLaCommandType::UNKNOWN)
  {
    // Find name and parameter start
    for (size_t i = 12; i < buffer.size(); i++)
    {
      if (buffer.at(i) == ' ')
      {
        m_name = std::string(reinterpret_cast<const char*>(&buffer[12]), i - 12);
        m_parameterOffset = i + 1; // Skip space
        break;
      }
    }
  }
}

CoLaBCommand::~CoLaBCommand()
{
}

const std::vector<uint8_t>& CoLaBCommand::getBuffer()
{
  return m_buffer;
}

const CoLaCommandType::Enum CoLaBCommand::getType()
{
  return m_type;
}

const char* CoLaBCommand::getName()
{
  return m_name.c_str();
}

size_t CoLaBCommand::getParameterOffset()
{
  return m_parameterOffset;
}

CoLaError::Enum CoLaBCommand::getError()
{
  return m_error;
}

CoLaBCommand CoLaBCommand::networkErrorCommand()
{
  return CoLaBCommand(CoLaCommandType::NETWORK_ERROR, CoLaError::NETWORK_ERROR, "");
}
