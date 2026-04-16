/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "nodes/Bno086ImuNode.hpp"

#include <memory>

#include <rclcpp/rclcpp.hpp>

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  std::shared_ptr<OASIS::ROS::Bno086ImuNode> node = std::make_shared<OASIS::ROS::Bno086ImuNode>();

  if (!node->Initialize())
    return -1;

  rclcpp::spin(node);

  node->Deinitialize();

  rclcpp::shutdown();

  return 0;
}
