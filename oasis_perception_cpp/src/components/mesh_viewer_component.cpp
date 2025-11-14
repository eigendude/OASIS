/*
 *  Copyright (C) 2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "nodes/MeshViewerNode.h"
#include "ros/RosComponent.h"

#include <memory>
#include <stdexcept>
#include <string>

namespace
{
constexpr const char* ROS_COMPONENT_NAME = "mesh_viewer_component";
} // namespace

namespace oasis_perception
{
class MeshViewerComponent : public oasis_perception::RosComponent
{
public:
  explicit MeshViewerComponent(const rclcpp::NodeOptions& options)
    : oasis_perception::RosComponent(ROS_COMPONENT_NAME, options)
  {
    m_interface = std::make_unique<OASIS::MeshViewerNode>(*m_node);
    if (!m_interface->Initialize())
    {
      throw std::runtime_error(std::string{"Failed to start component interface for node "} +
                               m_node->get_name());
    }
  }

  ~MeshViewerComponent() override
  {
    if (m_interface)
    {
      m_interface->Deinitialize();
      m_interface.reset();
    }
  }

private:
  std::unique_ptr<OASIS::MeshViewerNode> m_interface;
};
} // namespace oasis_perception

RCLCPP_COMPONENTS_REGISTER_NODE(oasis_perception::MeshViewerComponent);
