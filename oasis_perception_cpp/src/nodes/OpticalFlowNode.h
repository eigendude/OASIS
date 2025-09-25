/*
 *  Copyright (C) 2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */
#pragma once

#include <memory>

namespace rclcpp
{
class Node;
}

namespace OASIS
{
namespace VIDEO
{
class OpticalFlow;
}

namespace ROS
{
class OpticalFlowNode
{
public:
  OpticalFlowNode(rclcpp::Node& node);
  ~OpticalFlowNode();

  bool Initialize();
  void Deinitialize();

private:
  // Construction parameters
  rclcpp::Node& m_node;

  // Optical flow instance
  std::unique_ptr<VIDEO::OpticalFlow> m_opticalFlow;
};
} // namespace ROS
} // namespace OASIS
