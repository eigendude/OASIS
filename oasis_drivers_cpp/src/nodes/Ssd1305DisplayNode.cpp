/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "Ssd1305DisplayNode.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>
#include <thread>
#include <utility>

#include <rclcpp/qos.hpp>
#include <rclcpp_components/register_node_macro.hpp>

using OASIS::Display::Ssd1305ImageView;
using OASIS::Display::Ssd1305Rotation;
using OASIS::ROS::Ssd1305DisplayNode;

namespace
{
// ROS node name
constexpr const char* DEFAULT_NODE_NAME = "ssd1305_display";

// ROS topics
constexpr const char* TOPIC_IMAGE = "image";

// ROS services
constexpr const char* SERVICE_SET_ENABLED = "set_enabled";
constexpr const char* SERVICE_CLEAR = "clear";
constexpr const char* SERVICE_SET_INVERT = "set_invert";
constexpr const char* SERVICE_SET_CONTRAST = "set_contrast";

// ROS parameters
constexpr const char* PARAM_I2C_DEVICE = "i2c_device";
constexpr const char* PARAM_I2C_ADDRESS = "i2c_address";
constexpr const char* PARAM_WIDTH = "width";
constexpr const char* PARAM_HEIGHT = "height";
constexpr const char* PARAM_COLUMN_OFFSET = "column_offset";
constexpr const char* PARAM_CONTRAST = "contrast";
constexpr const char* PARAM_THRESHOLD = "threshold";
constexpr const char* PARAM_INVERT_PIXELS = "invert_pixels";
constexpr const char* PARAM_ROTATION = "rotation";
constexpr const char* PARAM_UPDATE_RATE_HZ = "update_rate_hz";
constexpr const char* PARAM_RECONNECT_INTERVAL_SEC = "reconnect_interval_sec";
constexpr const char* PARAM_RECONNECT_SETTLE_SEC = "reconnect_settle_sec";
constexpr const char* PARAM_DISPLAY_POWER_SETTLE_SEC = "display_power_settle_sec";
constexpr const char* PARAM_ENABLED = "enabled";
constexpr const char* PARAM_BLANK_ON_SHUTDOWN = "blank_on_shutdown";
constexpr const char* PARAM_REJECT_WRONG_DIMENSIONS = "reject_wrong_dimensions";
constexpr const char* PARAM_CLIP_WRONG_DIMENSIONS = "clip_wrong_dimensions";
constexpr const char* PARAM_ENABLE_PARTIAL_UPDATES = "enable_partial_updates";

constexpr const char* DEFAULT_I2C_DEVICE = "/dev/i2c-1";
constexpr int DEFAULT_I2C_ADDRESS = 0x3C;
constexpr int DEFAULT_WIDTH = static_cast<int>(OASIS::Display::SSD1305_DISPLAY_WIDTH);
constexpr int DEFAULT_HEIGHT = static_cast<int>(OASIS::Display::SSD1305_DISPLAY_HEIGHT);
constexpr int DEFAULT_COLUMN_OFFSET = 4;
constexpr int DEFAULT_CONTRAST = 0xFF;
constexpr int DEFAULT_THRESHOLD = 127;
constexpr bool DEFAULT_INVERT_PIXELS = false;
constexpr int DEFAULT_ROTATION = 0;
constexpr double DEFAULT_UPDATE_RATE_HZ = 30.0;
constexpr double DEFAULT_RECONNECT_INTERVAL_SEC = 1.0;
constexpr double DEFAULT_RECONNECT_SETTLE_SEC = 0.5;
constexpr double DEFAULT_DISPLAY_POWER_SETTLE_SEC = 0.25;
constexpr bool DEFAULT_ENABLED = true;
constexpr bool DEFAULT_BLANK_ON_SHUTDOWN = true;
constexpr bool DEFAULT_REJECT_WRONG_DIMENSIONS = true;
constexpr bool DEFAULT_CLIP_WRONG_DIMENSIONS = false;
constexpr bool DEFAULT_ENABLE_PARTIAL_UPDATES = true;

Ssd1305Rotation ParseRotation(int rotation_degrees)
{
  switch (rotation_degrees)
  {
    case 0:
      return Ssd1305Rotation::Rotate0;
    case 90:
      return Ssd1305Rotation::Rotate90;
    case 180:
      return Ssd1305Rotation::Rotate180;
    case 270:
      return Ssd1305Rotation::Rotate270;
    default:
      throw std::invalid_argument("rotation must be one of 0, 90, 180, or 270 degrees");
  }
}

std::chrono::nanoseconds TimerPeriod(double rate_hz)
{
  if (rate_hz <= 0.0)
    throw std::invalid_argument("update_rate_hz must be positive");

  const auto period = std::chrono::duration<double>(1.0 / rate_hz);
  const auto period_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(period);
  if (period_ns.count() <= 0)
    throw std::invalid_argument("update_rate_hz is too large to represent as a timer period");
  return period_ns;
}
} // namespace

Ssd1305DisplayNode::Ssd1305DisplayNode(const rclcpp::NodeOptions& options)
  : Ssd1305DisplayNode(options, nullptr)
{
}

Ssd1305DisplayNode::Ssd1305DisplayNode(
    const rclcpp::NodeOptions& options,
    std::unique_ptr<OASIS::Display::Ssd1305DeviceInterface> device,
    SleepFunction sleep)
  : rclcpp::Node(DEFAULT_NODE_NAME, options),
    m_deviceConfig{
        .i2c_device = DEFAULT_I2C_DEVICE,
        .i2c_address = DEFAULT_I2C_ADDRESS,
        .width = static_cast<std::size_t>(DEFAULT_WIDTH),
        .height = static_cast<std::size_t>(DEFAULT_HEIGHT),
        .column_offset = static_cast<std::uint8_t>(DEFAULT_COLUMN_OFFSET),
        .contrast = static_cast<std::uint8_t>(DEFAULT_CONTRAST),
    },
    m_framebufferConfig{
        .rotation = ParseRotation(DEFAULT_ROTATION),
        .threshold = static_cast<std::uint8_t>(DEFAULT_THRESHOLD),
        .invert_pixels = DEFAULT_INVERT_PIXELS,
        .reject_wrong_dimensions = DEFAULT_REJECT_WRONG_DIMENSIONS,
        .clip_wrong_dimensions = DEFAULT_CLIP_WRONG_DIMENSIONS,
    },
    m_updateRateHz(DEFAULT_UPDATE_RATE_HZ),
    m_reconnectIntervalSec(DEFAULT_RECONNECT_INTERVAL_SEC),
    m_reconnectSettleSec(DEFAULT_RECONNECT_SETTLE_SEC),
    m_displayPowerSettleSec(DEFAULT_DISPLAY_POWER_SETTLE_SEC),
    m_blankOnShutdown(DEFAULT_BLANK_ON_SHUTDOWN),
    m_enablePartialUpdates(DEFAULT_ENABLE_PARTIAL_UPDATES),
    m_device(std::move(device)),
    m_sleep(std::move(sleep)),
    m_frontBuffer{},
    m_hasPendingFrame(false),
    m_desiredGeneration(0),
    m_displayEnabled(DEFAULT_ENABLED),
    m_controllerState(ControllerState::Disconnected),
    m_failedReconnectAttempts(0)
{
  ReadConfig();

  if (!m_sleep)
    m_sleep = [](std::chrono::nanoseconds duration) { std::this_thread::sleep_for(duration); };

  if (!m_device)
    m_device = std::make_unique<OASIS::Display::Ssd1305Device>(m_deviceConfig);

  // ROS topics
  m_imageSubscription = create_subscription<sensor_msgs::msg::Image>(
      TOPIC_IMAGE, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::Image::ConstSharedPtr image) { HandleImage(std::move(image)); });

  // ROS services
  m_enableDisplayService = create_service<std_srvs::srv::SetBool>(
      SERVICE_SET_ENABLED, [this](const std_srvs::srv::SetBool::Request::SharedPtr request,
                                  const std_srvs::srv::SetBool::Response::SharedPtr response)
      { HandleEnableDisplay(request, response); });
  m_clearDisplayService = create_service<std_srvs::srv::Trigger>(
      SERVICE_CLEAR, [this](const std_srvs::srv::Trigger::Request::SharedPtr request,
                            const std_srvs::srv::Trigger::Response::SharedPtr response)
      { HandleClearDisplay(request, response); });
  m_setInvertService = create_service<std_srvs::srv::SetBool>(
      SERVICE_SET_INVERT, [this](const std_srvs::srv::SetBool::Request::SharedPtr request,
                                 const std_srvs::srv::SetBool::Response::SharedPtr response)
      { HandleSetInvert(request, response); });
  m_setContrastService = create_service<oasis_msgs::srv::SetDisplayContrast>(
      SERVICE_SET_CONTRAST,
      [this](const oasis_msgs::srv::SetDisplayContrast::Request::SharedPtr request,
             const oasis_msgs::srv::SetDisplayContrast::Response::SharedPtr response)
      { HandleSetContrast(request, response); });

  // ROS timers
  m_updateTimer =
      create_wall_timer(TimerPeriod(m_updateRateHz), [this]() { (void)FlushPendingFrame(); });
  m_reconnectTimer = create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                           std::chrono::duration<double>(m_reconnectIntervalSec)),
                                       [this]() { (void)AttemptReconnect(); });
  const auto settle_period = std::max(std::chrono::nanoseconds(1),
                                      std::chrono::duration_cast<std::chrono::nanoseconds>(
                                          std::chrono::duration<double>(m_reconnectSettleSec)));
  m_stabilizationTimer =
      create_wall_timer(settle_period, [this]() { (void)CompleteStabilization(); });
  m_stabilizationTimer->cancel();

  AttemptInitialConnection();

  RCLCPP_INFO(get_logger(),
              "SSD1305 configured: device=%s address=0x%02X size=%zux%zu "
              "column_offset=%u rotation=%d contrast=%u update_rate_hz=%.3f "
              "display_power_settle_sec=%.3f enabled=%s partial_updates=%s",
              m_deviceConfig.i2c_device.c_str(), m_deviceConfig.i2c_address, m_deviceConfig.width,
              m_deviceConfig.height, m_deviceConfig.column_offset,
              static_cast<int>(get_parameter(PARAM_ROTATION).as_int()), m_deviceConfig.contrast,
              m_updateRateHz, m_displayPowerSettleSec, m_displayEnabled ? "true" : "false",
              m_enablePartialUpdates ? "true" : "false");
}

Ssd1305DisplayNode::~Ssd1305DisplayNode()
{
  if (m_updateTimer)
    m_updateTimer->cancel();
  if (m_reconnectTimer)
    m_reconnectTimer->cancel();
  if (m_stabilizationTimer)
    m_stabilizationTimer->cancel();

  // m_deviceMutex serializes all controller access, including shutdown
  std::scoped_lock device_lock(m_deviceMutex);
  if (!m_device)
    return;

  if (m_controllerState != ControllerState::Ready)
    return;

  if (m_blankOnShutdown)
  {
    try
    {
      OASIS::Display::Ssd1305Framebuffer blank;
      m_device->WriteFullFrame(blank.Data());
    }
    catch (const std::exception& error)
    {
      RCLCPP_WARN(get_logger(), "Failed to blank SSD1305 during shutdown: %s", error.what());
    }
  }

  try
  {
    m_device->SetDisplayEnabled(false);
  }
  catch (const std::exception& error)
  {
    RCLCPP_WARN(get_logger(), "Failed to turn off SSD1305 during shutdown: %s", error.what());
  }
}

void Ssd1305DisplayNode::ReadConfig()
{
  declare_parameter(PARAM_I2C_DEVICE, std::string(DEFAULT_I2C_DEVICE));
  declare_parameter(PARAM_I2C_ADDRESS, DEFAULT_I2C_ADDRESS);
  declare_parameter(PARAM_WIDTH, DEFAULT_WIDTH);
  declare_parameter(PARAM_HEIGHT, DEFAULT_HEIGHT);
  declare_parameter(PARAM_COLUMN_OFFSET, DEFAULT_COLUMN_OFFSET);
  declare_parameter(PARAM_CONTRAST, DEFAULT_CONTRAST);
  declare_parameter(PARAM_THRESHOLD, DEFAULT_THRESHOLD);
  declare_parameter(PARAM_INVERT_PIXELS, DEFAULT_INVERT_PIXELS);
  declare_parameter(PARAM_ROTATION, DEFAULT_ROTATION);
  declare_parameter(PARAM_UPDATE_RATE_HZ, DEFAULT_UPDATE_RATE_HZ);
  declare_parameter(PARAM_RECONNECT_INTERVAL_SEC, DEFAULT_RECONNECT_INTERVAL_SEC);
  declare_parameter(PARAM_RECONNECT_SETTLE_SEC, DEFAULT_RECONNECT_SETTLE_SEC);
  declare_parameter(PARAM_DISPLAY_POWER_SETTLE_SEC, DEFAULT_DISPLAY_POWER_SETTLE_SEC);
  declare_parameter(PARAM_ENABLED, DEFAULT_ENABLED);
  declare_parameter(PARAM_BLANK_ON_SHUTDOWN, DEFAULT_BLANK_ON_SHUTDOWN);
  declare_parameter(PARAM_REJECT_WRONG_DIMENSIONS, DEFAULT_REJECT_WRONG_DIMENSIONS);
  declare_parameter(PARAM_CLIP_WRONG_DIMENSIONS, DEFAULT_CLIP_WRONG_DIMENSIONS);
  declare_parameter(PARAM_ENABLE_PARTIAL_UPDATES, DEFAULT_ENABLE_PARTIAL_UPDATES);

  const int i2c_address = static_cast<int>(get_parameter(PARAM_I2C_ADDRESS).as_int());
  const int width = static_cast<int>(get_parameter(PARAM_WIDTH).as_int());
  const int height = static_cast<int>(get_parameter(PARAM_HEIGHT).as_int());
  const int column_offset = static_cast<int>(get_parameter(PARAM_COLUMN_OFFSET).as_int());
  const int contrast = static_cast<int>(get_parameter(PARAM_CONTRAST).as_int());
  const int threshold = static_cast<int>(get_parameter(PARAM_THRESHOLD).as_int());
  const double reconnect_interval_sec = get_parameter(PARAM_RECONNECT_INTERVAL_SEC).as_double();
  const double reconnect_settle_sec = get_parameter(PARAM_RECONNECT_SETTLE_SEC).as_double();
  const double display_power_settle_sec = get_parameter(PARAM_DISPLAY_POWER_SETTLE_SEC).as_double();

  if (column_offset < 0 || column_offset > 4)
    throw std::invalid_argument("column_offset must be in [0, 4] for 128 x 32 SSD1305 panels");
  if (contrast < 0 || contrast > 255)
    throw std::invalid_argument("contrast must be in [0, 255]");
  if (threshold < 0 || threshold > 255)
    throw std::invalid_argument("threshold must be in [0, 255]");
  if (!std::isfinite(reconnect_interval_sec) || reconnect_interval_sec <= 0.0)
    throw std::invalid_argument("reconnect_interval_sec must be finite and positive");
  if (!std::isfinite(reconnect_settle_sec) || reconnect_settle_sec < 0.0)
    throw std::invalid_argument("reconnect_settle_sec must be finite and nonnegative");
  if (!std::isfinite(display_power_settle_sec) || display_power_settle_sec < 0.0)
    throw std::invalid_argument("display_power_settle_sec must be finite and nonnegative");
  if (width != DEFAULT_WIDTH || height != DEFAULT_HEIGHT)
    throw std::invalid_argument("width and height must be exactly 128 and 32 for Product 4567");

  m_deviceConfig.i2c_device = get_parameter(PARAM_I2C_DEVICE).as_string();
  m_deviceConfig.i2c_address = i2c_address;
  m_deviceConfig.width = static_cast<std::size_t>(width);
  m_deviceConfig.height = static_cast<std::size_t>(height);
  m_deviceConfig.column_offset = static_cast<std::uint8_t>(column_offset);
  m_deviceConfig.contrast = static_cast<std::uint8_t>(contrast);
  m_framebufferConfig.threshold = static_cast<std::uint8_t>(threshold);
  m_framebufferConfig.invert_pixels = get_parameter(PARAM_INVERT_PIXELS).as_bool();
  m_framebufferConfig.reject_wrong_dimensions =
      get_parameter(PARAM_REJECT_WRONG_DIMENSIONS).as_bool();
  m_framebufferConfig.clip_wrong_dimensions = get_parameter(PARAM_CLIP_WRONG_DIMENSIONS).as_bool();
  m_framebufferConfig.rotation =
      ParseRotation(static_cast<int>(get_parameter(PARAM_ROTATION).as_int()));
  m_updateRateHz = get_parameter(PARAM_UPDATE_RATE_HZ).as_double();
  m_reconnectIntervalSec = reconnect_interval_sec;
  m_reconnectSettleSec = reconnect_settle_sec;
  m_displayPowerSettleSec = display_power_settle_sec;
  m_displayEnabled = get_parameter(PARAM_ENABLED).as_bool();
  m_blankOnShutdown = get_parameter(PARAM_BLANK_ON_SHUTDOWN).as_bool();
  m_enablePartialUpdates = get_parameter(PARAM_ENABLE_PARTIAL_UPDATES).as_bool();
}

void Ssd1305DisplayNode::AttemptInitialConnection()
{
  OASIS::Display::Ssd1305Framebuffer::Buffer desired{};
  std::uint64_t generation = 0;
  std::scoped_lock device_lock(m_deviceMutex);
  {
    std::scoped_lock lock(m_pendingMutex);
    desired = m_pendingBuffer.Data();
    generation = m_desiredGeneration;
  }

  try
  {
    m_controllerState = ControllerState::Recovering;
    // Healthy startup intentionally mirrors Recover(): initialize, restore
    // contrast, commit a full frame, then restore the requested power state
    m_device->Initialize();
    m_device->SetContrast(m_deviceConfig.contrast);
    m_device->WriteFullFrame(desired);
    SetDisplayEnabledAfterPowerSettle(m_displayEnabled);
    m_frontBuffer = desired;
    m_controllerState = ControllerState::Ready;
    std::scoped_lock pending_lock(m_pendingMutex);
    if (m_desiredGeneration == generation)
    {
      m_pendingBuffer.MarkClean();
      m_hasPendingFrame = false;
    }
  }
  catch (const std::exception& error)
  {
    EnterReconnectModeLocked(error.what(), true);
  }
}

void Ssd1305DisplayNode::EnterReconnectModeLocked(const std::string& error, bool initial_failure)
{
  if (m_stabilizationTimer)
    m_stabilizationTimer->cancel();
  m_controllerState = ControllerState::Disconnected;
  m_failedReconnectAttempts = 0;
  {
    std::scoped_lock lock(m_pendingMutex);
    m_hasPendingFrame = true;
  }
  RCLCPP_WARN(get_logger(),
              initial_failure ? "SSD1305 unavailable at startup; entering reconnect mode: %s"
                              : "SSD1305 unavailable; entering reconnect mode: %s",
              error.c_str());
}

bool Ssd1305DisplayNode::AttemptReconnect()
{
  OASIS::Display::Ssd1305Framebuffer::Buffer desired{};
  std::scoped_lock device_lock(m_deviceMutex);
  if (m_controllerState != ControllerState::Disconnected)
    return false;

  {
    std::scoped_lock lock(m_pendingMutex);
    desired = m_pendingBuffer.Data();
  }
  m_controllerState = ControllerState::Recovering;
  try
  {
    // Keep the panel off until the final synchronization has restored the
    // latest state and allowed its power circuitry to settle
    m_device->Recover(desired, m_deviceConfig.contrast, false);
    m_controllerState = ControllerState::Stabilizing;
    m_stabilizationTimer->reset();
    RCLCPP_DEBUG(get_logger(), "SSD1305 recovery completed; stabilizing for %.3f seconds",
                 m_reconnectSettleSec);
    return true;
  }
  catch (const std::exception& error)
  {
    m_controllerState = ControllerState::Disconnected;
    ++m_failedReconnectAttempts;
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                         "SSD1305 reconnect attempt %u failed: %s", m_failedReconnectAttempts,
                         error.what());
    return false;
  }
}

bool Ssd1305DisplayNode::CompleteStabilization()
{
  m_stabilizationTimer->cancel();
  OASIS::Display::Ssd1305Framebuffer::Buffer desired{};
  std::uint64_t generation = 0;
  std::scoped_lock device_lock(m_deviceMutex);
  if (m_controllerState != ControllerState::Stabilizing)
    return false;

  {
    std::scoped_lock lock(m_pendingMutex);
    desired = m_pendingBuffer.Data();
    generation = m_desiredGeneration;
  }

  try
  {
    m_device->SetContrast(m_deviceConfig.contrast);
    m_device->WriteFullFrame(desired);
    SetDisplayEnabledAfterPowerSettle(m_displayEnabled);
    m_frontBuffer = desired;
    m_controllerState = ControllerState::Ready;
    {
      std::scoped_lock lock(m_pendingMutex);
      if (m_desiredGeneration == generation)
      {
        m_pendingBuffer.MarkClean();
        m_hasPendingFrame = false;
      }
    }
    if (m_failedReconnectAttempts == 0)
      RCLCPP_INFO(get_logger(), "SSD1305 reconnected and restored on first attempt");
    else
      RCLCPP_INFO(get_logger(),
                  "SSD1305 reconnected and restored after %u failed reconnect attempts",
                  m_failedReconnectAttempts);
    m_failedReconnectAttempts = 0;
    return true;
  }
  catch (const std::exception& error)
  {
    m_controllerState = ControllerState::Disconnected;
    ++m_failedReconnectAttempts;
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                         "SSD1305 stabilization failed on reconnect attempt %u: %s",
                         m_failedReconnectAttempts, error.what());
    return false;
  }
}

void Ssd1305DisplayNode::SetDisplayEnabledAfterPowerSettle(bool enabled)
{
  if (enabled && m_displayPowerSettleSec > 0.0)
  {
    const auto settle_duration = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(m_displayPowerSettleSec));
    m_sleep(settle_duration);
  }
  m_device->SetDisplayEnabled(enabled);
}

void Ssd1305DisplayNode::HandleImage(sensor_msgs::msg::Image::ConstSharedPtr image)
{
  try
  {
    Ssd1305ImageView view{
        .data = std::span<const std::uint8_t>(image->data.data(), image->data.size()),
        .width = image->width,
        .height = image->height,
        .step = image->step,
        .encoding = image->encoding,
    };

    std::scoped_lock lock(m_pendingMutex);
    m_pendingBuffer.Update(view, m_framebufferConfig);
    m_hasPendingFrame = true;
    ++m_desiredGeneration;
  }
  catch (const std::exception& error)
  {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Rejected SSD1305 image: %s",
                         error.what());
  }
}

Ssd1305DisplayNode::FlushResult Ssd1305DisplayNode::FlushPendingFrame()
{
  OASIS::Display::Ssd1305Framebuffer::Buffer desired{};
  std::uint64_t generation = 0;
  {
    std::scoped_lock lock(m_pendingMutex);
    if (!m_hasPendingFrame)
      return {FlushStatus::NoChange, "no pending SSD1305 frame"};
    if (!m_displayEnabled)
      return {FlushStatus::QueuedWhileDisabled, "SSD1305 frame queued while display is disabled"};
  }

  // m_deviceMutex serializes controller operations. Normal refreshes must
  // recheck enabled state while holding it so a successful disable cannot be
  // followed by a stale framebuffer write.
  std::scoped_lock device_lock(m_deviceMutex);
  if (m_controllerState != ControllerState::Ready)
    return {FlushStatus::Failed, "SSD1305 frame queued for reconnect"};
  if (!m_displayEnabled)
    return {FlushStatus::QueuedWhileDisabled, "SSD1305 frame queued while display is disabled"};

  {
    std::scoped_lock lock(m_pendingMutex);
    if (!m_hasPendingFrame)
      return {FlushStatus::NoChange, "no pending SSD1305 frame"};
    desired = m_pendingBuffer.Data();
    generation = m_desiredGeneration;
  }

  try
  {
    bool wrote_frame = false;
    if (m_enablePartialUpdates)
    {
      const auto dirty_pages = OASIS::Display::ComparePages(m_frontBuffer, desired);
      for (std::size_t page = 0; page < dirty_pages.size(); ++page)
      {
        if (!dirty_pages[page])
          continue;
        m_device->WritePage(desired, page);
        wrote_frame = true;
        const auto first = desired.begin() + static_cast<std::ptrdiff_t>(
                                                 page * OASIS::Display::SSD1305_DISPLAY_WIDTH);
        std::copy_n(first, OASIS::Display::SSD1305_DISPLAY_WIDTH,
                    m_frontBuffer.begin() +
                        static_cast<std::ptrdiff_t>(page * OASIS::Display::SSD1305_DISPLAY_WIDTH));
      }
    }
    else
    {
      m_device->WriteFullFrame(desired);
      wrote_frame = true;
      m_frontBuffer = desired;
    }
    {
      std::scoped_lock lock(m_pendingMutex);
      if (m_desiredGeneration == generation)
      {
        m_pendingBuffer.MarkClean();
        m_hasPendingFrame = false;
      }
    }
    return {wrote_frame ? FlushStatus::Updated : FlushStatus::NoChange,
            wrote_frame ? "SSD1305 frame updated" : "SSD1305 display already synchronized"};
  }
  catch (const std::exception& error)
  {
    EnterReconnectModeLocked(error.what(), false);
    return {FlushStatus::Failed, error.what()};
  }
}

void Ssd1305DisplayNode::HandleEnableDisplay(
    const std_srvs::srv::SetBool::Request::SharedPtr request,
    const std_srvs::srv::SetBool::Response::SharedPtr response)
{
  const bool requested_enabled = request->data;
  m_displayEnabled = requested_enabled;
  std::scoped_lock device_lock(m_deviceMutex);
  try
  {
    if (m_controllerState != ControllerState::Ready)
    {
      response->success = true;
      response->message = requested_enabled ? "SSD1305 enable queued for reconnect"
                                            : "SSD1305 disable queued for reconnect";
      return;
    }

    if (!requested_enabled)
    {
      m_device->SetDisplayEnabled(false);
    }
    else
    {
      OASIS::Display::Ssd1305Framebuffer::Buffer desired{};
      {
        std::scoped_lock lock(m_pendingMutex);
        desired = m_pendingBuffer.Data();
      }
      m_device->WriteFullFrame(desired);
      m_device->SetContrast(m_deviceConfig.contrast);
      m_device->SetDisplayEnabled(true);
      m_frontBuffer = desired;
      std::scoped_lock lock(m_pendingMutex);
      if (m_pendingBuffer.Data() == desired)
      {
        m_pendingBuffer.MarkClean();
        m_hasPendingFrame = false;
      }
    }
    response->success = true;
    response->message = m_displayEnabled ? "SSD1305 display enabled" : "SSD1305 display disabled";
  }
  catch (const std::exception& error)
  {
    EnterReconnectModeLocked(error.what(), false);
    response->success = true;
    response->message = requested_enabled ? "SSD1305 enable queued after device failure"
                                          : "SSD1305 disable queued after device failure";
  }
}

void Ssd1305DisplayNode::HandleClearDisplay(
    const std_srvs::srv::Trigger::Request::SharedPtr request,
    const std_srvs::srv::Trigger::Response::SharedPtr response)
{
  (void)request;

  try
  {
    {
      std::scoped_lock device_lock(m_deviceMutex);
      std::scoped_lock pending_lock(m_pendingMutex);
      const bool desired_clear =
          std::all_of(m_pendingBuffer.Data().begin(), m_pendingBuffer.Data().end(),
                      [](std::uint8_t byte) { return byte == 0; });
      const bool confirmed_clear = std::all_of(m_frontBuffer.begin(), m_frontBuffer.end(),
                                               [](std::uint8_t byte) { return byte == 0; });
      if (m_displayEnabled && !m_hasPendingFrame && desired_clear && confirmed_clear)
      {
        response->success = true;
        response->message = "SSD1305 display already clear";
        return;
      }
    }

    {
      std::scoped_lock lock(m_pendingMutex);
      m_pendingBuffer.Clear(false);
      m_hasPendingFrame = true;
      ++m_desiredGeneration;
    }
    const FlushResult result = FlushPendingFrame();
    switch (result.status)
    {
      case FlushStatus::Updated:
        response->success = true;
        response->message = "SSD1305 display cleared";
        break;
      case FlushStatus::QueuedWhileDisabled:
        response->success = true;
        response->message = "SSD1305 clear queued while display is disabled";
        break;
      case FlushStatus::NoChange:
        response->success = true;
        response->message = "SSD1305 display already clear";
        break;
      case FlushStatus::Failed:
        response->success = true;
        response->message = "SSD1305 clear queued for reconnect";
        break;
    }
  }
  catch (const std::exception& error)
  {
    response->success = false;
    response->message = error.what();
  }
}

void Ssd1305DisplayNode::HandleSetInvert(const std_srvs::srv::SetBool::Request::SharedPtr request,
                                         const std_srvs::srv::SetBool::Response::SharedPtr response)
{
  bool invert_pixels = false;
  {
    std::scoped_lock lock(m_pendingMutex);
    if (m_framebufferConfig.invert_pixels != request->data)
    {
      m_framebufferConfig.invert_pixels = request->data;
      m_pendingBuffer.Invert();
      m_hasPendingFrame = true;
      ++m_desiredGeneration;
    }
    invert_pixels = m_framebufferConfig.invert_pixels;
  }
  response->success = true;
  {
    std::scoped_lock device_lock(m_deviceMutex);
    const bool queued = m_controllerState != ControllerState::Ready;
    response->message = invert_pixels ? (queued ? "SSD1305 pixel inversion queued for reconnect"
                                                : "SSD1305 pixel inversion enabled")
                                      : (queued ? "SSD1305 pixel inversion queued for reconnect"
                                                : "SSD1305 pixel inversion disabled");
  }
}

void Ssd1305DisplayNode::HandleSetContrast(
    const oasis_msgs::srv::SetDisplayContrast::Request::SharedPtr request,
    const oasis_msgs::srv::SetDisplayContrast::Response::SharedPtr response)
{
  std::scoped_lock device_lock(m_deviceMutex);
  try
  {
    m_deviceConfig.contrast = request->contrast;
    if (m_controllerState != ControllerState::Ready)
    {
      response->success = true;
      response->message = "SSD1305 contrast queued for reconnect";
      return;
    }
    m_device->SetContrast(request->contrast);
    response->success = true;
    response->message = "SSD1305 contrast updated";
  }
  catch (const std::exception& error)
  {
    EnterReconnectModeLocked(error.what(), false);
    response->success = true;
    response->message = "SSD1305 contrast queued after device failure";
  }
}

RCLCPP_COMPONENTS_REGISTER_NODE(OASIS::ROS::Ssd1305DisplayNode)
