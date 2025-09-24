/*
 *  Copyright (C) 2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "nodes/OpticalFlowNode.h"
#include "ros/RosComponent.h"

namespace oasis_perception
{
// Default component name
constexpr const char* ROS_COMPONENT_NAME = "optical_flow_component";

class OpticalFlowComponent : public RosComponent
{
public:
  OpticalFlowComponent(const rclcpp::NodeOptions& options)
    : RosComponent(ROS_COMPONENT_NAME, options)
  {
    m_interface = std::make_unique<OpticalFlowNode>(*m_node);
    if (!m_interface->Initialize())
      throw std::runtime_error(std::string{"Failed to start "} + m_node->get_name());
  }

  ~OpticalFlowComponent() override
  {
    if (m_interface)
    {
      m_interface->Deinitialize();
      m_interface.reset();
    }
  }

private:
  std::unique_ptr<OpticalFlowNode> m_interface;
};
} // namespace oasis_perception

// Register the component with ROS 2
RCLCPP_COMPONENTS_REGISTER_NODE(oasis_perception::OpticalFlowComponent);
