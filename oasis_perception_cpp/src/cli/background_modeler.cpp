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

using namespace OASIS;

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  std::shared_ptr<BackgroundModelerNode> node = std::make_shared<BackgroundModelerNode>();
  node->Initialize();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
