/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#pragma once

#include "cockpit/cockpit_visualizer.hpp"

#include <chrono>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace OASIS::Visualization
{
/** ROS integration for the Falcon cockpit renderer */
class CockpitVisualizerNode : public rclcpp::Node
{
public:
  explicit CockpitVisualizerNode(const rclcpp::NodeOptions& options);

private:
  void PublishFrame();

  std::unique_ptr<CockpitVisualizer> visualizer_;
  std::chrono::steady_clock::time_point start_time_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};
} // namespace OASIS::Visualization
