/*
 *  Copyright (C) 2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "nodes/BackgroundSubtractorNode.h"

#include <memory>

#include <rclcpp/rclcpp.hpp>

using namespace OASIS;

namespace
{
constexpr const char* NODE_NAME = "background_subtractor";
} // namespace

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>(NODE_NAME);
  BackgroundSubtractorNode backgroundSubtractor(*node);
  if (!backgroundSubtractor.Initialize())
  {
    RCLCPP_FATAL(node->get_logger(), "Failed to start background subtractor node");
    return 1;
  }

  rclcpp::spin(node);

  backgroundSubtractor.Deinitialize();

  rclcpp::shutdown();
  return 0;
}
