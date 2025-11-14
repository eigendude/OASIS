/*
 *  Copyright (C) 2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "nodes/MeshViewerNode.h"

#include <memory>

#include <rclcpp/rclcpp.hpp>

namespace
{
constexpr const char* NODE_NAME = "mesh_viewer";
} // namespace

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>(NODE_NAME);

  OASIS::MeshViewerNode meshViewer(*node);
  if (!meshViewer.Initialize())
  {
    RCLCPP_FATAL(node->get_logger(), "Failed to start mesh viewer node");
    return 1;
  }

  rclcpp::spin(node);

  meshViewer.Deinitialize();

  rclcpp::shutdown();
  return 0;
}
