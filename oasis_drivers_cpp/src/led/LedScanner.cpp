/*
 *  Copyright (C) 2021 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See DOCS/LICENSING.md for more information.
 */

#include "LedScanner.h"
#include "platform/linux/led/BeagleBoneArray.h"
#include "platform/linux/led/LinuxKeyboardLEDs.h"
#include "platform/linux/led/RaspberryPiArray.h"

using namespace OASIS;
using namespace LED;

LedVector LedScanner::GetLEDs(rclcpp::Logger& logger)
{
  LedVector leds;

  leds = LINUX::BeagleBoneArray::GetLEDs(logger);
  if (leds.empty())
    leds = LINUX::RaspberryPiArray::GetLEDs(logger);
  if (leds.empty())
    leds = LINUX::LinuxKeyboardLEDs::GetLEDs(logger);

  return leds;
}
