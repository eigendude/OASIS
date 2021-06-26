/*
 *  Copyright (C) 2021 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See DOCS/LICENSING.md for more information.
 */

#include "LinuxLED.h"

#include <errno.h>
#include <fcntl.h>
#include <rclcpp/logging.hpp>
#include <string.h>
#include <unistd.h>

using namespace OASIS;
using namespace LINUX;

LinuxLED::LinuxLED(const std::string &sysfsNode, rclcpp::Logger& logger) :
  m_sysfsNode(sysfsNode),
  m_sysfsPath("/sys/class/leds/" + sysfsNode + "/brightness"),
  m_logger(logger)
{
}

LinuxLED::~LinuxLED()
{
  Close();
}

bool LinuxLED::Open()
{
  m_brightnessfd = open(m_sysfsPath.c_str(), O_WRONLY);
  if (m_brightnessfd < 0)
    return false;

  RCLCPP_DEBUG(m_logger, "Found LED %s", m_sysfsNode.c_str());

  return true;
}

void LinuxLED::Close()
{
  if (m_brightnessfd != -1)
  {
    RCLCPP_DEBUG(m_logger, "Closing LED %s", m_sysfsNode.c_str());

    close(m_brightnessfd);
    m_brightnessfd = -1;
  }
}

void LinuxLED::Update(bool force)
{
  if (m_brightnessfd < 0)
    return;

  bool enabled = (m_brightness > 0.5f);

  if (m_enabled != enabled || force)
  {
    if (enabled)
      write(m_brightnessfd, "1\n", 2);
    else
      write(m_brightnessfd, "0\n", 2);

    m_enabled = enabled;
  }
}
