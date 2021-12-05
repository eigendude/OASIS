/*
 *  Copyright (C) 2021 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See DOCS/LICENSING.md for more information.
 */

#include "LinuxKeyboardLEDs.h"

#include "LinuxLED.h"

#include <string>
#include <vector>

using namespace OASIS;
using namespace LINUX;

LED::LedVector LinuxKeyboardLEDs::GetLEDs(rclcpp::Logger& logger)
{
  LED::LedVector leds;

  const std::vector<std::string> sysfsNodes = {
      // Ubuntu VM on macOS VirtualBox host
      "input2::capslock", "input2::numlock", "input2::scrolllock",

      // Raspberry Pi 3
      "input8::capslock",

      // Lenovo laptop
      "input4::capslock", "input4::numlock", "input4::scrolllock",

      // Dell netbook
      "input3::capslock", "input3::numlock", "input3::scrolllock",

      // TODO: Iterate over input:: nodes
  };

  for (const std::string& sysfsNode : sysfsNodes)
  {
    LED::LedPtr led(new LinuxLED(sysfsNode, logger));
    if (led->Open())
      leds.emplace_back(std::move(led));
  }

  return leds;
}
