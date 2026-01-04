/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "nodes/Mmc5983maMagnetometerNode.h"

#include <memory>

#include <rclcpp/rclcpp.hpp>

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  std::shared_ptr<OASIS::ROS::Mmc5983maMagnetometerNode> node =
      std::make_shared<OASIS::ROS::Mmc5983maMagnetometerNode>();

  if (!node->Initialize())
    return -1;

  rclcpp::spin(node);

  node->Deinitialize();

  rclcpp::shutdown();

  return 0;
}
