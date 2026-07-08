/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "nodes/PoseLandmarkRendererNode.h"

#include <memory>

#include <rclcpp/rclcpp.hpp>

using namespace oasis_perception;

namespace
{
constexpr const char* ROS_NODE_NAME = "pose_landmark_renderer";
} // namespace

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = std::make_shared<rclcpp::Node>(ROS_NODE_NAME);

  PoseLandmarkRendererNode poseLandmarkRenderer{*node};
  if (!poseLandmarkRenderer.Start())
  {
    RCLCPP_ERROR(node->get_logger(), "Error starting %s", node->get_name());
    return 1;
  }

  rclcpp::spin(node);

  poseLandmarkRenderer.Stop();

  rclcpp::shutdown();

  return 0;
}
