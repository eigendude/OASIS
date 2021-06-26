/*
 *  Copyright (C) 2021 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See DOCS/LICENSING.md for more information.
 */

#include "RaspberryPiArray.h"
#include "LinuxLED.h"

#include <string>
#include <vector>

using namespace OASIS;
using namespace LINUX;

LED::LedVector RaspberryPiArray::GetLEDs(rclcpp::Logger& logger)
{
  LED::LedVector leds;

  const std::vector<std::string> sysfsNodes = {
      // ACT (a green LED)
      "led0",

      // PWR (a red LED)
      //   - Not controllable on early Pi models (always on while powered)
      //   - Not found on the Pi Zero
      "led1",
  };

  for (const std::string &sysfsNode : sysfsNodes)
  {
    LED::LedPtr led(new LinuxLED(sysfsNode, logger));
    if (led->Open())
      leds.emplace_back(std::move(led));
  }

  return leds;
}
