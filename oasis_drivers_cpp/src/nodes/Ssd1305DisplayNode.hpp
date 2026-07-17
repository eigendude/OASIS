/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#pragma once

#include "display/Ssd1305Display.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <cstdint>
#include <memory>
#include <mutex>
#include <vector>

namespace OASIS::ROS
{
class Ssd1305DisplayNode : public rclcpp::Node
{
public:
  explicit Ssd1305DisplayNode(const rclcpp::NodeOptions& options);
  ~Ssd1305DisplayNode() override;

private:
  void OnImage(const sensor_msgs::msg::Image::ConstSharedPtr& image);
  void Flush();
  bool ConvertImage(const sensor_msgs::msg::Image& image,
                    std::vector<std::uint8_t>& framebuffer,
                    std::string& error) const;
  bool PixelIsOn(const sensor_msgs::msg::Image& image,
                 std::uint32_t x, std::uint32_t y) const;

  OASIS::Display::Ssd1305Display m_display;
  std::uint32_t m_width{128};
  std::uint32_t m_height{32};
  int m_rotation{0};
  int m_luminanceThreshold{128};
  bool m_enabled{true};

  mutable std::mutex m_mutex;
  std::vector<std::uint8_t> m_pendingFramebuffer;
  bool m_framePending{false};

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr m_imageSubscription;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr m_clearService;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr m_enabledService;
  rclcpp::TimerBase::SharedPtr m_updateTimer;
};
} // namespace OASIS::ROS
