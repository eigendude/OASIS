/*
 *  Copyright (C) 2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "RosComponent.h"

using namespace oasis_perception;

RosComponent::RosComponent(const std::string& componentName, const rclcpp::NodeOptions& options)
  : m_node(std::make_shared<rclcpp::Node>(componentName, options))
{
}

std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> RosComponent::get_node_base_interface()
{
  return m_node->get_node_base_interface();
}
