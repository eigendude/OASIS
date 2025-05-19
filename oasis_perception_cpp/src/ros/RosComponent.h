/*
 *  Copyright (C) 2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */
#pragma once

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

namespace oasis_perception
{
class RosComponent
{
public:
  RosComponent(const std::string& componentName, const rclcpp::NodeOptions& options);
  virtual ~RosComponent() = default;

  // This function is required for the component to be registered with ROS 2.
  //
  // It returns the node base interface, which is used by the component manager
  // to manage the lifecycle of the component.
  std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> get_node_base_interface();

protected:
  std::shared_ptr<rclcpp::Node> m_node;
};
} // namespace oasis_perception
