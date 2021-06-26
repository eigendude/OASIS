/*
 *  Copyright (C) 2021 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See DOCS/LICENSING.md for more information.
 */
#pragma once

#include "led/LedTypes.h"

namespace rclcpp
{
  class Logger;
}

namespace OASIS
{
namespace LINUX
{

/*!
 * \brief Raspberry Pi LEDs
 *
 * TODO: Encapsulate behind an interface
 */
class RaspberryPiArray
{
public:
  /*!
   * \brief Do a scan and return the discovered LEDs
   *
   * \return The LEDs in order of the physical array, or empty if no LEDs were
   *         discovered.
   */
  static LED::LedVector GetLEDs(rclcpp::Logger& logger);
};

}
}
