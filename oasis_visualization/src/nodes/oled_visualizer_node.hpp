/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#pragma once

#include "oled/oled_visualizer.hpp"

#include <chrono>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace OASIS::Visualization
{
/** ROS integration for the perspective OLED renderer */
class OledVisualizerNode : public rclcpp::Node
{
public:
  explicit OledVisualizerNode(const rclcpp::NodeOptions& options);

private:
  static void ValidatePositive(const char* name, double value);
  void PublishFrame();

  std::unique_ptr<OledVisualizer> visualizer;
  double revolution_seconds;
  double rotation_nonlinearity;
  std::chrono::steady_clock::time_point start_time;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher;
  rclcpp::TimerBase::SharedPtr timer_;
};
} // namespace OASIS::Visualization
