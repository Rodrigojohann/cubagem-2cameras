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

#pragma once

#include <stdint.h>
#include <string>
#include <vector>
#include "CoLaError.h"
#include "CoLaCommandType.h"

class CoLaBCommand
{
private:
  std::vector<uint8_t> m_buffer;
  CoLaCommandType::Enum m_type;
  std::string m_name;
  size_t m_parameterOffset;
  CoLaError::Enum m_error;

  /// <summary>Construct a new <see cref="CoLaBCommand" /> with the given command type, error, and name, but without any data.</summary>
  CoLaBCommand(CoLaCommandType::Enum commandType, CoLaError::Enum error, const char* name);

public:
  /// <summary>Construct a new <see cref="CoLaBCommand" /> from the given data buffer.</summary>
  CoLaBCommand(std::vector<uint8_t> buffer);
  ~CoLaBCommand();

  /// <summary>Get the binary data buffer.</summary>
  const std::vector<uint8_t>& getBuffer();

  /// <summary>Get the type of command.</summary>
  const CoLaCommandType::Enum getType();

  /// <summary>Get the name of command.</summary>
  const char* getName();

  /// <summary>Get offset in bytes to where first parameter starts.</summary>
  size_t getParameterOffset();

  /// <summary>Get error.</summary>
  CoLaError::Enum getError();

  /// <summary>Create a command for network errors.</summary>
  static CoLaBCommand networkErrorCommand();
};

