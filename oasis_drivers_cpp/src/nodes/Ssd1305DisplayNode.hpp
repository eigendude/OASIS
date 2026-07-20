/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#pragma once

#include "display/ssd1305/Ssd1305Device.hpp"
#include "display/ssd1305/Ssd1305Framebuffer.hpp"

#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>

#include <oasis_msgs/srv/set_display_contrast.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/service.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/timer.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace OASIS::ROS
{
class Ssd1305DisplayNodeTestAccess;

class Ssd1305DisplayNode : public rclcpp::Node
{
public:
  using SleepFunction = std::function<void(std::chrono::nanoseconds)>;

  explicit Ssd1305DisplayNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  Ssd1305DisplayNode(const rclcpp::NodeOptions& options,
                     std::unique_ptr<OASIS::Display::Ssd1305DeviceInterface> device,
                     SleepFunction sleep = {});
  ~Ssd1305DisplayNode() override;

private:
  friend class Ssd1305DisplayNodeTestAccess;

  enum class FlushStatus
  {
    NoChange,
    Updated,
    QueuedWhileDisabled,
    Failed,
  };

  struct FlushResult
  {
    FlushStatus status;
    std::string message;
  };

  enum class ControllerState
  {
    Disconnected,
    Recovering,
    Stabilizing,
    Ready,
  };

  void ReadConfig();
  void AttemptInitialConnection();
  bool AttemptReconnect();
  bool CompleteStabilization();
  // m_deviceMutex must be held before calling this method
  void SetDisplayEnabledAfterPowerSettle(bool enabled);
  // m_deviceMutex must be held before calling this method
  void EnterReconnectModeLocked(const std::string& error, bool initial_failure);
  void HandleImage(sensor_msgs::msg::Image::ConstSharedPtr image);
  FlushResult FlushPendingFrame();
  void HandleEnableDisplay(const std_srvs::srv::SetBool::Request::SharedPtr request,
                           const std_srvs::srv::SetBool::Response::SharedPtr response);
  void HandleClearDisplay(const std_srvs::srv::Trigger::Request::SharedPtr request,
                          const std_srvs::srv::Trigger::Response::SharedPtr response);
  void HandleSetInvert(const std_srvs::srv::SetBool::Request::SharedPtr request,
                       const std_srvs::srv::SetBool::Response::SharedPtr response);
  void HandleSetContrast(const oasis_msgs::srv::SetDisplayContrast::Request::SharedPtr request,
                         const oasis_msgs::srv::SetDisplayContrast::Response::SharedPtr response);

  OASIS::Display::Ssd1305DeviceConfig m_deviceConfig;
  OASIS::Display::Ssd1305FramebufferConfig m_framebufferConfig;
  double m_updateRateHz;
  double m_reconnectIntervalSec;
  double m_reconnectSettleSec;
  double m_displayPowerSettleSec;
  bool m_blankOnShutdown;
  bool m_enablePartialUpdates;

  std::unique_ptr<OASIS::Display::Ssd1305DeviceInterface> m_device;
  SleepFunction m_sleep;
  OASIS::Display::Ssd1305Framebuffer::Buffer m_frontBuffer;
  OASIS::Display::Ssd1305Framebuffer m_pendingBuffer;
  std::mutex m_pendingMutex;
  std::mutex m_deviceMutex;
  bool m_hasPendingFrame;
  std::uint64_t m_desiredGeneration;
  std::atomic_bool m_displayEnabled;
  // Protected by m_deviceMutex together with m_frontBuffer and the reconnect
  // attempt counter
  ControllerState m_controllerState;
  unsigned m_failedReconnectAttempts;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr m_imageSubscription;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr m_enableDisplayService;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr m_clearDisplayService;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr m_setInvertService;
  rclcpp::Service<oasis_msgs::srv::SetDisplayContrast>::SharedPtr m_setContrastService;
  rclcpp::TimerBase::SharedPtr m_updateTimer;
  rclcpp::TimerBase::SharedPtr m_reconnectTimer;
  rclcpp::TimerBase::SharedPtr m_stabilizationTimer;
};
} // namespace OASIS::ROS
