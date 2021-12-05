/*
 *  Copyright (C) 2021 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See DOCS/LICENSING.md for more information.
 */

#include "led/LedScanner.h" // TODO: Move to platform
#include "led/LedServer.h"
#include "led/LedTypes.h"
#include "ros/LedServerNode.h"
#include "utils/LogUtils.h"

#include <memory>

#include <rclcpp/rclcpp.hpp>

using namespace OASIS;

int main(int argc, char** argv)
{
  // Initialize ROS
  rclcpp::init(argc, argv);

  // Create node
  std::shared_ptr<rclcpp::Node> node = std::make_shared<OASIS::ROS::LedServerNode>();

  // Initialize logging
  rclcpp::Logger logger = OASIS::UTILS::LogUtils::InitializeLogging(node);

  // Scan for LEDs
  LED::LedVector leds = LED::LedScanner::GetLEDs(logger);

  if (!leds.empty())
  {
    RCLCPP_INFO(logger, "Found %u LEDs", leds.size());

    LED::LedServer ledServer(std::move(leds));

    if (ledServer.Initialize())
    {
      RCLCPP_DEBUG(logger, "Running ROS node");
      rclcpp::spin(node);
      ledServer.Deinitialize();
    }
    else
    {
      RCLCPP_ERROR(logger, "Failed to initialize ROS node");
    }
  }
  else
  {
    RCLCPP_ERROR(logger, "No LEDs found");
  }

  RCLCPP_DEBUG(logger, "Shutting down");
  rclcpp::shutdown();

  return 0;
}
