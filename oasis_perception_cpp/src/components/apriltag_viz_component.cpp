/*
 *  Copyright (C) 2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "nodes/AprilTagVizNode.h"
#include "ros/RosComponent.h"

#include <stdexcept>

using namespace OASIS;

namespace oasis_perception
{
// Default component name
constexpr const char* ROS_COMPONENT_NAME = "apriltag_viz_component";

class AprilTagVizComponent : public RosComponent
{
public:
  explicit AprilTagVizComponent(const rclcpp::NodeOptions& options)
    : RosComponent(ROS_COMPONENT_NAME, options)
  {
    m_interface = std::make_unique<AprilTagVizNode>(*m_node);
    if (!m_interface->Initialize())
      throw std::runtime_error(std::string{"Failed to start "} + m_node->get_name());
  }

  ~AprilTagVizComponent() override
  {
    if (m_interface)
    {
      m_interface->Deinitialize();
      m_interface.reset();
    }
  }

private:
  std::unique_ptr<AprilTagVizNode> m_interface;
};
} // namespace oasis_perception

// Register the component with ROS 2
RCLCPP_COMPONENTS_REGISTER_NODE(oasis_perception::AprilTagVizComponent);
