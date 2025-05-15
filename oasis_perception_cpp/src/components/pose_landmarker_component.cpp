/*
 *  Copyright (C) 2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "nodes/PoseLandmarkerNode.h"
#include "ros/RosComponent.h"

namespace
{
// Default component name
constexpr const char* ROS_COMPONENT_NAME = "pose_landmarker_component";
} // namespace

namespace oasis_perception
{
class PoseLandmarkerComponent : public RosComponent
{
public:
  PoseLandmarkerComponent(const rclcpp::NodeOptions& options)
    : RosComponent(ROS_COMPONENT_NAME, options)
  {
    m_interface = std::make_unique<PoseLandmarkerNode>(*m_node);
    if (!m_interface->Start())
      throw std::runtime_error(std::string{"Failed to start "} + m_node->get_name());
  }

  ~PoseLandmarkerComponent() override
  {
    if (m_interface)
    {
      m_interface->Stop();
      m_interface.reset();
    }
  }

private:
  std::unique_ptr<PoseLandmarkerNode> m_interface;
};
} // namespace oasis_perception

// Register the component with ROS 2
RCLCPP_COMPONENTS_REGISTER_NODE(oasis_perception::PoseLandmarkerComponent);
