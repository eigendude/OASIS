/*
 *  Copyright (C) 2021 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "cec/CecServer.h"
#include "ros/CecServerNode.h"
#include "utils/LogUtils.h"

#include <memory>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char *argv[])
{
  // Initialize ROS
  rclcpp::init(argc, argv);

  // Create node
  std::shared_ptr<OASIS::ROS::CecServerNode> node = std::make_shared<OASIS::ROS::CecServerNode>();

  // Initialize logging
  rclcpp::Logger logger = OASIS::UTILS::LogUtils::InitializeLogging(node);

  {
    OASIS::CEC::CecServer cecServer(*node, node, logger);
    node->RegisterServer(cecServer);

    if (cecServer.Initialize())
    {
      rclcpp::spin(node);
      cecServer.Deinitialize();
    }

    node->UnregisterServer();
  }

  rclcpp::shutdown();

  return 0;
}
