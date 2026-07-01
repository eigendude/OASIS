/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "nodes/CheckerboardDetectorNode.h"
#include "ros/RosComponent.h"

#include <memory>
#include <stdexcept>
#include <string>

#include <rclcpp_components/register_node_macro.hpp>

namespace
{
constexpr const char* ROS_COMPONENT_NAME = "checkerboard_detector_component";
} // namespace

namespace oasis_perception
{
class CheckerboardDetectorComponent : public oasis_perception::RosComponent
{
public:
  explicit CheckerboardDetectorComponent(const rclcpp::NodeOptions& options)
    : oasis_perception::RosComponent(ROS_COMPONENT_NAME, options)
  {
    m_interface = std::make_unique<OASIS::CheckerboardDetectorNode>(*m_node);
    if (!m_interface->Initialize())
    {
      throw std::runtime_error(std::string{"Failed to start component interface for node "} +
                               m_node->get_name());
    }
  }

  ~CheckerboardDetectorComponent() override
  {
    if (m_interface)
    {
      m_interface->Deinitialize();
      m_interface.reset();
    }
  }

private:
  std::unique_ptr<OASIS::CheckerboardDetectorNode> m_interface;
};
} // namespace oasis_perception

RCLCPP_COMPONENTS_REGISTER_NODE(oasis_perception::CheckerboardDetectorComponent);
