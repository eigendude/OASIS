/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "nodes/Ssd1305DisplayNode.hpp"

#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/image_encodings.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <string>
#include <utility>

namespace OASIS::ROS
{
Ssd1305DisplayNode::Ssd1305DisplayNode(const rclcpp::NodeOptions& options)
  : rclcpp::Node("ssd1305_display", options)
{
  OASIS::Display::Ssd1305Config config;
  config.i2cDevice = declare_parameter<std::string>("i2c_device", "/dev/i2c-1");
  config.i2cAddress = static_cast<std::uint8_t>(
      declare_parameter<int>("i2c_address", 0x3c));
  config.gpioChip = declare_parameter<std::string>("gpio_chip", "/dev/gpiochip0");
  config.resetGpio = static_cast<std::uint32_t>(
      declare_parameter<int>("reset_gpio", 4));
  config.width = static_cast<std::uint32_t>(declare_parameter<int>("width", 128));
  config.height = static_cast<std::uint32_t>(declare_parameter<int>("height", 32));
  config.columnOffset = static_cast<std::uint8_t>(
      declare_parameter<int>("column_offset", 4));
  config.contrast = static_cast<std::uint8_t>(
      std::clamp(declare_parameter<int>("contrast", 255), 0, 255));
  config.blankOnClose = declare_parameter<bool>("blank_on_shutdown", true);

  m_width = config.width;
  m_height = config.height;
  m_rotation = declare_parameter<int>("rotation", 0);
  m_luminanceThreshold = std::clamp(
      declare_parameter<int>("luminance_threshold", 128), 0, 255);
  m_enabled = declare_parameter<bool>("enabled", true);
  const double updateRateHz =
      declare_parameter<double>("update_rate_hz", 15.0);

  if (m_rotation != 0 && m_rotation != 180)
    throw std::invalid_argument("rotation must be 0 or 180 for a 128x32 panel");
  if (!std::isfinite(updateRateHz) || updateRateHz <= 0.0)
    throw std::invalid_argument("update_rate_hz must be positive");

  std::string error;
  if (!m_display.Open(config, error))
    throw std::runtime_error("Failed to initialize SSD1305 display: " + error);
  if (!m_display.SetEnabled(m_enabled, error))
    throw std::runtime_error("Failed to set SSD1305 enabled state: " + error);

  m_pendingFramebuffer.assign(m_display.FramebufferSize(), 0);

  m_imageSubscription = create_subscription<sensor_msgs::msg::Image>(
      "display/image", rclcpp::SensorDataQoS().keep_last(1),
      std::bind(&Ssd1305DisplayNode::OnImage, this, std::placeholders::_1));

  m_clearService = create_service<std_srvs::srv::Trigger>(
      "display/clear",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
             std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        std::string serviceError;
        response->success = m_display.Clear(serviceError);
        response->message = response->success ? "display cleared" : serviceError;
      });

  m_enabledService = create_service<std_srvs::srv::SetBool>(
      "display/set_enabled",
      [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
             std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
        std::string serviceError;
        response->success = m_display.SetEnabled(request->data, serviceError);
        response->message = response->success ? "display state updated" : serviceError;
        if (response->success)
          m_enabled = request->data;
      });

  const auto period = std::chrono::duration<double>(1.0 / updateRateHz);
  m_updateTimer = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&Ssd1305DisplayNode::Flush, this));

  RCLCPP_INFO(get_logger(),
              "SSD1305 display initialized: %ux%u at %s address 0x%02x",
              m_width, m_height, config.i2cDevice.c_str(), config.i2cAddress);
}

Ssd1305DisplayNode::~Ssd1305DisplayNode()
{
  m_display.Close();
}

void Ssd1305DisplayNode::OnImage(
    const sensor_msgs::msg::Image::ConstSharedPtr& image)
{
  std::vector<std::uint8_t> framebuffer;
  std::string error;
  if (!ConvertImage(*image, framebuffer, error))
  {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                         "Rejected display image: %s", error.c_str());
    return;
  }

  std::lock_guard<std::mutex> lock(m_mutex);
  m_pendingFramebuffer = std::move(framebuffer);
  m_framePending = true;
}

void Ssd1305DisplayNode::Flush()
{
  if (!m_enabled)
    return;

  std::vector<std::uint8_t> framebuffer;
  {
    std::lock_guard<std::mutex> lock(m_mutex);
    if (!m_framePending)
      return;
    framebuffer = m_pendingFramebuffer;
    m_framePending = false;
  }

  std::string error;
  if (!m_display.WriteFramebuffer(framebuffer, error))
  {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
                          "SSD1305 framebuffer write failed: %s", error.c_str());
  }
}

bool Ssd1305DisplayNode::ConvertImage(
    const sensor_msgs::msg::Image& image,
    std::vector<std::uint8_t>& framebuffer,
    std::string& error) const
{
  if (image.width != m_width || image.height != m_height)
  {
    error = "expected " + std::to_string(m_width) + "x" +
            std::to_string(m_height) + ", got " +
            std::to_string(image.width) + "x" + std::to_string(image.height);
    return false;
  }

  const bool supported =
      image.encoding == sensor_msgs::image_encodings::MONO8 ||
      image.encoding == sensor_msgs::image_encodings::RGB8 ||
      image.encoding == sensor_msgs::image_encodings::BGR8;
  if (!supported)
  {
    error = "unsupported encoding " + image.encoding;
    return false;
  }

  const std::size_t channels =
      image.encoding == sensor_msgs::image_encodings::MONO8 ? 1U : 3U;
  const std::size_t minimumStep = static_cast<std::size_t>(m_width) * channels;
  if (image.step < minimumStep ||
      image.data.size() < static_cast<std::size_t>(image.step) * m_height)
  {
    error = "image data is shorter than its dimensions and step";
    return false;
  }

  framebuffer.assign(static_cast<std::size_t>(m_width) * (m_height / 8U), 0);
  for (std::uint32_t y = 0; y < m_height; ++y)
  {
    for (std::uint32_t x = 0; x < m_width; ++x)
    {
      const std::uint32_t sourceX = m_rotation == 0 ? x : m_width - 1U - x;
      const std::uint32_t sourceY = m_rotation == 0 ? y : m_height - 1U - y;
      if (!PixelIsOn(image, sourceX, sourceY))
        continue;

      const std::size_t index =
          static_cast<std::size_t>(y / 8U) * m_width + x;
      framebuffer[index] |= static_cast<std::uint8_t>(1U << (y & 7U));
    }
  }

  return true;
}

bool Ssd1305DisplayNode::PixelIsOn(const sensor_msgs::msg::Image& image,
                                   std::uint32_t x,
                                   std::uint32_t y) const
{
  const std::uint8_t* row = image.data.data() +
                            static_cast<std::size_t>(y) * image.step;
  if (image.encoding == sensor_msgs::image_encodings::MONO8)
    return row[x] >= m_luminanceThreshold;

  const std::uint8_t* pixel = row + static_cast<std::size_t>(x) * 3U;
  const bool rgb = image.encoding == sensor_msgs::image_encodings::RGB8;
  const int red = pixel[rgb ? 0 : 2];
  const int green = pixel[1];
  const int blue = pixel[rgb ? 2 : 0];
  const int luminance = (77 * red + 150 * green + 29 * blue) >> 8;
  return luminance >= m_luminanceThreshold;
}
} // namespace OASIS::ROS

RCLCPP_COMPONENTS_REGISTER_NODE(OASIS::ROS::Ssd1305DisplayNode)
