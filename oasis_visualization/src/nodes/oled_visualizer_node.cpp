/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "nodes/oled_visualizer_node.hpp"

#include <cmath>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <string>

namespace OASIS::Visualization
{
namespace
{
// ROS node name
constexpr const char* DEFAULT_NODE_NAME = "oled_visualizer";

// ROS parameters
constexpr const char* PARAM_IMAGE_PATH = "image_path";
constexpr const char* PARAM_FRAME_RATE = "frame_rate";
constexpr const char* PARAM_REVOLUTION_SECONDS = "revolution_seconds";
constexpr const char* PARAM_FOCAL_LENGTH = "focal_length";
constexpr const char* PARAM_CAMERA_DISTANCE = "camera_distance";
constexpr const char* PARAM_MODEL_SCALE = "model_scale";

// ROS topics
constexpr const char* IMAGE_TOPIC = "image";

// OLED parameters
constexpr int DISPLAY_WIDTH = 128;
constexpr int DISPLAY_HEIGHT = 32;
constexpr double DEFAULT_FRAME_RATE = 30.0;

// Visualization parameters
constexpr double DEFAULT_REVOLUTION_SECONDS = 8.0;
constexpr double DEFAULT_FOCAL_LENGTH = 95.0;
constexpr double DEFAULT_CAMERA_DISTANCE = 150.0;
constexpr double DEFAULT_MODEL_SCALE = 1.4;

// Mathematical constants
constexpr double TWO_PI = 6.28318530717958647692;
} // namespace

OledVisualizerNode::OledVisualizerNode(const rclcpp::NodeOptions& options)
  : Node(DEFAULT_NODE_NAME, options)
{
  std::string imagePath = declare_parameter<std::string>(PARAM_IMAGE_PATH, "");
  const double frameRate = declare_parameter<double>(PARAM_FRAME_RATE, DEFAULT_FRAME_RATE);
  revolution_seconds =
      declare_parameter<double>(PARAM_REVOLUTION_SECONDS, DEFAULT_REVOLUTION_SECONDS);

  const OledVisualizerConfig config{
      DISPLAY_WIDTH,
      DISPLAY_HEIGHT,
      declare_parameter<double>(PARAM_FOCAL_LENGTH, DEFAULT_FOCAL_LENGTH),
      declare_parameter<double>(PARAM_CAMERA_DISTANCE, DEFAULT_CAMERA_DISTANCE),
      declare_parameter<double>(PARAM_MODEL_SCALE, DEFAULT_MODEL_SCALE),
  };

  ValidatePositive(PARAM_FRAME_RATE, frameRate);
  ValidatePositive(PARAM_REVOLUTION_SECONDS, revolution_seconds);

  if (imagePath.empty())
    throw std::runtime_error("parameter 'image_path' must not be empty");

  visualizer = std::make_unique<OledVisualizer>(imagePath, config);

  publisher = create_publisher<sensor_msgs::msg::Image>(IMAGE_TOPIC, rclcpp::SensorDataQoS());
  start_time = std::chrono::steady_clock::now();

  const auto period = std::chrono::duration<double>(1.0 / frameRate);
  const auto timerPeriod = std::chrono::duration_cast<std::chrono::nanoseconds>(period);
  if (timerPeriod.count() == 0)
    throw std::runtime_error("parameter 'frame_rate' is too large");
  timer_ = create_wall_timer(timerPeriod, [this]() { PublishFrame(); });
}

void OledVisualizerNode::ValidatePositive(const char* name, double value)
{
  if (!std::isfinite(value) || value <= 0.0)
  {
    throw std::runtime_error(std::string{"parameter '"} + name +
                             "' must be finite and greater than zero");
  }
}

void OledVisualizerNode::PublishFrame()
{
  const std::chrono::duration<double> elapsed = std::chrono::steady_clock::now() - start_time;
  const double angle = std::fmod(elapsed.count(), revolution_seconds) * TWO_PI / revolution_seconds;
  const cv::Mat frame = visualizer->Render(angle);

  auto message = std::make_unique<sensor_msgs::msg::Image>();
  message->header.stamp = now();
  message->height = DISPLAY_HEIGHT;
  message->width = DISPLAY_WIDTH;
  message->encoding = "mono8";
  message->is_bigendian = false;
  message->step = DISPLAY_WIDTH;
  message->data.resize(DISPLAY_WIDTH * DISPLAY_HEIGHT);
  if (frame.isContinuous())
  {
    std::memcpy(message->data.data(), frame.data, message->data.size());
  }
  else
  {
    for (int row = 0; row < DISPLAY_HEIGHT; ++row)
    {
      std::memcpy(message->data.data() + row * DISPLAY_WIDTH, frame.ptr(row), DISPLAY_WIDTH);
    }
  }
  publisher->publish(std::move(message));
}
} // namespace OASIS::Visualization
