/*
 *  Copyright (C) 2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "nodes/MultiModelerNode.h"

#include <memory>

#include <rclcpp/rclcpp.hpp>

using namespace OASIS;

namespace
{
// Default node name
constexpr const char* NODE_NAME = "multi_modeler";
} // namespace

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>(NODE_NAME);
  MultiModelerNode multiModeler(*node);
  if (!multiModeler.Initialize())
  {
    RCLCPP_FATAL(node->get_logger(), "Failed to start multi modeler node");
    return 1;
  }

  rclcpp::spin(node);

  multiModeler.Deinitialize();

  rclcpp::shutdown();
  return 0;
}
