/*
 *  Copyright (C) 2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "nodes/MonocularSlamNode.h"

#include <memory>
#include <string>

#include <rclcpp/parameter.hpp>
#include <rclcpp/rclcpp.hpp>

using namespace OASIS;

namespace
{
// Default node name
constexpr const char* ROS_NODE_NAME = "monocular_slam";
constexpr const char* SYSTEM_ID_PARAMETER = "system_id";

void EnsureSystemId(rclcpp::Node& node)
{
  std::string systemId;
  if (!node.get_parameter(SYSTEM_ID_PARAMETER, systemId) || systemId.empty())
  {
    systemId = node.get_name();
    node.set_parameter(rclcpp::Parameter(SYSTEM_ID_PARAMETER, systemId));
  }
}
} // namespace

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = std::make_shared<rclcpp::Node>(ROS_NODE_NAME);

  ROS::MonocularSlamNode monocularSlam{*node};
  EnsureSystemId(*node);
  if (!monocularSlam.Initialize())
  {
    RCLCPP_ERROR(node->get_logger(), "Error starting %s", node->get_name());
    return 1;
  }

  rclcpp::spin(node);

  monocularSlam.Deinitialize();

  rclcpp::shutdown();

  return 0;
}
