/*
 *  Copyright (C) 2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "ros/Mpu6050Node.h"

#include <memory>

#include <rclcpp/rclcpp.hpp>

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  std::shared_ptr<OASIS::ROS::Mpu6050Node> node = std::make_shared<OASIS::ROS::Mpu6050Node>();

  if (!node->Initialize())
    return -1;

  rclcpp::spin(node);

  node->Deinitialize();

  rclcpp::shutdown();

  return 0;
}
