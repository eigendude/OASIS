/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "nodes/Acs37800PowerMeterNode.hpp"

#include <exception>
#include <memory>

#include <rclcpp/rclcpp.hpp>

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  try
  {
    const auto node = std::make_shared<OASIS::ROS::Acs37800PowerMeterNode>();
    rclcpp::spin(node);
  }
  catch (const std::exception& error)
  {
    RCLCPP_FATAL(rclcpp::get_logger("acs37800_power_meter"), "Startup failed: %s", error.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
