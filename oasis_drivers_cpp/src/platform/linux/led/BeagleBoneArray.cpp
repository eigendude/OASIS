/*
 *  Copyright (C) 2021 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See DOCS/LICENSING.md for more information.
 */

#include "BeagleBoneArray.h"

#include "LinuxLED.h"

#include <string>
#include <vector>

using namespace OASIS;
using namespace LINUX;

LED::LedVector BeagleBoneArray::GetLEDs(rclcpp::Logger& logger)
{
  LED::LedVector leds;

  const std::vector<std::string> sysfsNodes = {
      "beaglebone:green:heartbeat",
      "beaglebone:green:mmc0",
      "beaglebone:green:usr2",
      "beaglebone:green:usr3",
  };

  for (const std::string& sysfsNode : sysfsNodes)
  {
    LED::LedPtr led(new LinuxLED(sysfsNode, logger));
    if (led->Open())
      leds.emplace_back(std::move(led));
  }

  return leds;
}
