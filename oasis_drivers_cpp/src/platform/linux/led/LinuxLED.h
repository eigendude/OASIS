/*
 *  Copyright (C) 2021 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See DOCS/LICENSING.md for more information.
 */
#pragma once

#include "led/ILed.h"

#include <string>

namespace rclcpp
{
  class Logger;
}

namespace OASIS
{
namespace LINUX
{

/*!
 * \brief Sysfs LED on Linux
 */
class LinuxLED : public LED::ILed
{
public:
  LinuxLED(const std::string &sysfsNode, rclcpp::Logger& logger);
  ~LinuxLED() override;

  // Implementation of ILed
  bool Open() override;
  void Close() override;
  void Update(bool force) override;

private:
  // Construction parameters
  const std::string m_sysfsNode;
  const std::string m_sysfsPath;
  rclcpp::Logger& m_logger;

  // Linux parameters
  int m_brightnessfd = -1; // File descriptor of brightness node

  // State parameters
  bool m_enabled = false;
};

}
}
