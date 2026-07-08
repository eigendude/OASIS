/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "nodes/PoseLandmarkRendererNode.h"
#include "ros/RosComponent.h"

#include <rclcpp_components/register_node_macro.hpp>

namespace
{
constexpr const char* ROS_COMPONENT_NAME = "pose_landmark_renderer_component";
} // namespace

namespace pose_landmark_renderer
{
class PoseLandmarkRendererComponent : public oasis_perception::RosComponent
{
public:
  explicit PoseLandmarkRendererComponent(const rclcpp::NodeOptions& options)
    : RosComponent(ROS_COMPONENT_NAME, options)
  {
    interface = std::make_unique<oasis_perception::PoseLandmarkRendererNode>(*m_node);
    if (!interface->Start())
      throw std::runtime_error(std::string{"Failed to start "} + m_node->get_name());
  }

  ~PoseLandmarkRendererComponent() override
  {
    if (interface)
    {
      interface->Stop();
      interface.reset();
    }
  }

private:
  std::unique_ptr<oasis_perception::PoseLandmarkRendererNode> interface;
};
} // namespace pose_landmark_renderer

RCLCPP_COMPONENTS_REGISTER_NODE(pose_landmark_renderer::PoseLandmarkRendererComponent);
