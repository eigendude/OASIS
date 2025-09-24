/*
 *  Copyright (C) 2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "nodes/BackgroundModelerNode.h"

#include <memory>

#include <rclcpp/rclcpp.hpp>

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("background_modeler");
  OASIS::BackgroundModelerNode backgroundModeler(*node);
  if (!backgroundModeler.Start())
  {
    RCLCPP_FATAL(node->get_logger(), "Failed to start background modeler node");
    return 1;
  }

  rclcpp::spin(node);

  backgroundModeler.Stop();

  rclcpp::shutdown();
  return 0;
}
