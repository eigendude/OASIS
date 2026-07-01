/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "nodes/CheckerboardDetectorNode.h"

#include <memory>

#include <rclcpp/rclcpp.hpp>

namespace
{
constexpr const char* NODE_NAME = "checkerboard_detector";
} // namespace

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>(NODE_NAME);
  OASIS::CheckerboardDetectorNode detector(*node);
  if (!detector.Initialize())
  {
    RCLCPP_FATAL(node->get_logger(), "Failed to start checkerboard detector node");
    return 1;
  }

  rclcpp::spin(node);

  detector.Deinitialize();

  rclcpp::shutdown();
  return 0;
}
