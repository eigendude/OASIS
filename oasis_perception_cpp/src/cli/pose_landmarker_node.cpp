/*
 *  Copyright (C) 2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "nodes/PoseLandmarkerNode.h"

#include <memory>

#include <rclcpp/rclcpp.hpp>

using namespace oasis_perception;

namespace
{
// Default node name
constexpr const char* ROS_NODE_NAME = "pose_landmarker";
} // namespace

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = std::make_shared<rclcpp::Node>(ROS_NODE_NAME);

  PoseLandmarkerNode poseLandmarker{*node};
  if (!poseLandmarker.Start())
  {
    RCLCPP_ERROR(node->get_logger(), "Error starting %s", node->get_name());
    return 1;
  }

  rclcpp::spin(node);

  poseLandmarker.Stop();

  rclcpp::shutdown();

  return 0;
}
