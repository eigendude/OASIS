/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "nodes/cockpit_visualizer_node.hpp"

#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <stdexcept>

namespace OASIS::Visualization
{
namespace
{
// OLED parameters
constexpr int DISPLAY_WIDTH = 128;
constexpr int DISPLAY_HEIGHT = 32;

// ROS parameters
constexpr const char* PARAM_FRAME_RATE = "frame_rate";
constexpr const char* PARAM_TARGET_LIT_FRACTION = "target_lit_fraction";
constexpr const char* PARAM_MINIMUM_LIT_FRACTION = "minimum_lit_fraction";
constexpr const char* PARAM_MAXIMUM_LIT_FRACTION = "maximum_lit_fraction";
constexpr const char* PARAM_RANDOM_SEED = "random_seed";
constexpr double DEFAULT_FRAME_RATE = 10.0;
constexpr double DEFAULT_TARGET = 0.55;
constexpr double DEFAULT_MINIMUM = 0.45;
constexpr double DEFAULT_MAXIMUM = 0.68;
constexpr std::int64_t DEFAULT_SEED = 0x4F415349;
} // namespace

CockpitVisualizerNode::CockpitVisualizerNode(const rclcpp::NodeOptions& options)
  : Node("cockpit_visualizer", options)
{
  const double frameRate = declare_parameter<double>(PARAM_FRAME_RATE, DEFAULT_FRAME_RATE);
  const double target = declare_parameter<double>(PARAM_TARGET_LIT_FRACTION, DEFAULT_TARGET);
  const double minimum = declare_parameter<double>(PARAM_MINIMUM_LIT_FRACTION, DEFAULT_MINIMUM);
  const double maximum = declare_parameter<double>(PARAM_MAXIMUM_LIT_FRACTION, DEFAULT_MAXIMUM);
  const std::int64_t seed = declare_parameter<std::int64_t>(PARAM_RANDOM_SEED, DEFAULT_SEED);
  if (!std::isfinite(frameRate) || frameRate <= 0.0)
    throw std::runtime_error("parameter 'frame_rate' must be finite and greater than zero");
  if (seed < 0 || seed > std::numeric_limits<std::uint32_t>::max())
    throw std::runtime_error("parameter 'random_seed' must fit an unsigned 32-bit integer");

  visualizer_ = std::make_unique<CockpitVisualizer>(CockpitVisualizerConfig{
      DISPLAY_WIDTH, DISPLAY_HEIGHT, target, minimum, maximum, static_cast<std::uint32_t>(seed)});
  publisher_ = create_publisher<sensor_msgs::msg::Image>("image", rclcpp::SensorDataQoS());
  start_time_ = std::chrono::steady_clock::now();
  const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / frameRate));
  if (period.count() == 0)
    throw std::runtime_error("parameter 'frame_rate' is too large");
  timer_ = create_wall_timer(period, [this]() { PublishFrame(); });
}

void CockpitVisualizerNode::PublishFrame()
{
  const std::chrono::duration<double> elapsed = std::chrono::steady_clock::now() - start_time_;
  const cv::Mat frame = visualizer_->Render(elapsed.count());
  auto message = std::make_unique<sensor_msgs::msg::Image>();
  message->header.stamp = now();
  message->height = DISPLAY_HEIGHT;
  message->width = DISPLAY_WIDTH;
  message->encoding = "mono8";
  message->is_bigendian = false;
  message->step = DISPLAY_WIDTH;
  message->data.resize(DISPLAY_WIDTH * DISPLAY_HEIGHT);
  for (int row = 0; row < DISPLAY_HEIGHT; ++row)
    std::memcpy(message->data.data() + row * DISPLAY_WIDTH, frame.ptr(row), DISPLAY_WIDTH);
  publisher_->publish(std::move(message));
}
} // namespace OASIS::Visualization
