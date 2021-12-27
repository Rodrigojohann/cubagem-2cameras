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

namespace CoLaUserLevel
{
  /// <summary>Available CoLa user levels.</summary>
  enum Enum : int8_t
  {
    RUN = 0,
    OPERATOR = 1,
    MAINTENANCE = 2,
    AUTHORIZED_CLIENT = 3,
    SERVICE = 4
  };
}