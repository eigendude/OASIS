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
#include <stdexcept>
#include <utility>

#include <rclcpp/qos.hpp>
#include <rclcpp_components/register_node_macro.hpp>

using OASIS::Display::Ssd1305ImageView;
using OASIS::Display::Ssd1305Rotation;
using OASIS::ROS::Ssd1305DisplayNode;

namespace
{
constexpr const char* DEFAULT_NODE_NAME = "ssd1305_display";
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
constexpr const char* PARAM_RECOVER_AFTER_FAILURES = "recover_after_failures";
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
constexpr int DEFAULT_RECOVER_AFTER_FAILURES = 3;
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
    std::unique_ptr<OASIS::Display::Ssd1305DeviceInterface> device)
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
    m_recoverAfterFailures(static_cast<unsigned>(DEFAULT_RECOVER_AFTER_FAILURES)),
    m_blankOnShutdown(DEFAULT_BLANK_ON_SHUTDOWN),
    m_enablePartialUpdates(DEFAULT_ENABLE_PARTIAL_UPDATES),
    m_device(std::move(device)),
    m_frontBuffer{},
    m_hasPendingFrame(false),
    m_displayEnabled(DEFAULT_ENABLED),
    m_consecutiveFailures(0)
{
  ReadConfig();
  InitializeDevice();

  m_imageSubscription = create_subscription<sensor_msgs::msg::Image>(
      "display/image", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::Image::ConstSharedPtr image) { HandleImage(std::move(image)); });
  m_enableDisplayService = create_service<std_srvs::srv::SetBool>(
      "display/set_enabled", [this](const std_srvs::srv::SetBool::Request::SharedPtr request,
                                    const std_srvs::srv::SetBool::Response::SharedPtr response)
      { HandleEnableDisplay(request, response); });
  m_clearDisplayService = create_service<std_srvs::srv::Trigger>(
      "display/clear", [this](const std_srvs::srv::Trigger::Request::SharedPtr request,
                              const std_srvs::srv::Trigger::Response::SharedPtr response)
      { HandleClearDisplay(request, response); });
  m_setInvertService = create_service<std_srvs::srv::SetBool>(
      "display/set_invert", [this](const std_srvs::srv::SetBool::Request::SharedPtr request,
                                   const std_srvs::srv::SetBool::Response::SharedPtr response)
      { HandleSetInvert(request, response); });
  m_setContrastService = create_service<oasis_msgs::srv::SetDisplayContrast>(
      "display/set_contrast",
      [this](const oasis_msgs::srv::SetDisplayContrast::Request::SharedPtr request,
             const oasis_msgs::srv::SetDisplayContrast::Response::SharedPtr response)
      { HandleSetContrast(request, response); });
  m_updateTimer =
      create_wall_timer(TimerPeriod(m_updateRateHz), [this]() { (void)FlushPendingFrame(); });

  RCLCPP_INFO(get_logger(),
              "SSD1305 initialized: device=%s address=0x%02X size=%zux%zu "
              "column_offset=%u rotation=%d contrast=%u update_rate_hz=%.3f "
              "enabled=%s partial_updates=%s",
              m_deviceConfig.i2c_device.c_str(), m_deviceConfig.i2c_address, m_deviceConfig.width,
              m_deviceConfig.height, m_deviceConfig.column_offset,
              static_cast<int>(get_parameter(PARAM_ROTATION).as_int()), m_deviceConfig.contrast,
              m_updateRateHz, m_displayEnabled ? "true" : "false",
              m_enablePartialUpdates ? "true" : "false");
}

Ssd1305DisplayNode::~Ssd1305DisplayNode()
{
  if (m_updateTimer)
    m_updateTimer->cancel();

  // m_deviceMutex serializes all controller access, including shutdown
  std::scoped_lock device_lock(m_deviceMutex);
  if (!m_device)
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
  declare_parameter(PARAM_RECOVER_AFTER_FAILURES, DEFAULT_RECOVER_AFTER_FAILURES);
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
  const int recover_after_failures =
      static_cast<int>(get_parameter(PARAM_RECOVER_AFTER_FAILURES).as_int());

  if (column_offset < 0 || column_offset > 4)
    throw std::invalid_argument("column_offset must be in [0, 4] for 128 x 32 SSD1305 panels");
  if (contrast < 0 || contrast > 255)
    throw std::invalid_argument("contrast must be in [0, 255]");
  if (threshold < 0 || threshold > 255)
    throw std::invalid_argument("threshold must be in [0, 255]");
  if (recover_after_failures <= 0)
    throw std::invalid_argument("recover_after_failures must be positive");
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
  m_recoverAfterFailures = static_cast<unsigned>(recover_after_failures);
  m_displayEnabled = get_parameter(PARAM_ENABLED).as_bool();
  m_blankOnShutdown = get_parameter(PARAM_BLANK_ON_SHUTDOWN).as_bool();
  m_enablePartialUpdates = get_parameter(PARAM_ENABLE_PARTIAL_UPDATES).as_bool();
}

void Ssd1305DisplayNode::InitializeDevice()
{
  if (!m_device)
    m_device = std::make_unique<OASIS::Display::Ssd1305Device>(m_deviceConfig);
  m_device->Initialize();
  m_device->WriteFullFrame(m_pendingBuffer.Data());
  m_device->SetContrast(m_deviceConfig.contrast);
  if (m_displayEnabled)
    m_device->SetDisplayEnabled(true);
  m_frontBuffer = m_pendingBuffer.Data();
  m_pendingBuffer.MarkClean();
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
  if (!m_displayEnabled)
    return {FlushStatus::QueuedWhileDisabled, "SSD1305 frame queued while display is disabled"};

  {
    std::scoped_lock lock(m_pendingMutex);
    if (!m_hasPendingFrame)
      return {FlushStatus::NoChange, "no pending SSD1305 frame"};
    desired = m_pendingBuffer.Data();
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
      if (m_pendingBuffer.Data() == desired)
      {
        m_pendingBuffer.MarkClean();
        m_hasPendingFrame = false;
      }
    }
    if (m_consecutiveFailures > 0)
    {
      RCLCPP_INFO(get_logger(), "SSD1305 recovered after %u failed writes", m_consecutiveFailures);
    }
    m_consecutiveFailures = 0;
    return {wrote_frame ? FlushStatus::Updated : FlushStatus::NoChange,
            wrote_frame ? "SSD1305 frame updated" : "SSD1305 display already synchronized"};
  }
  catch (const std::exception& error)
  {
    std::string failure_message = error.what();
    ++m_consecutiveFailures;
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
                          "SSD1305 write failed (%u consecutive): %s", m_consecutiveFailures,
                          error.what());
    if (m_consecutiveFailures >= m_recoverAfterFailures)
    {
      try
      {
        m_device->Recover(desired, m_displayEnabled);
        m_frontBuffer = desired;
        {
          std::scoped_lock lock(m_pendingMutex);
          if (m_pendingBuffer.Data() == desired)
          {
            m_pendingBuffer.MarkClean();
            m_hasPendingFrame = false;
          }
        }
        m_consecutiveFailures = 0;
        return {FlushStatus::Updated, "SSD1305 frame updated after recovery"};
      }
      catch (const std::exception& recover_error)
      {
        failure_message = std::string(error.what()) + "; recovery failed: " + recover_error.what();
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "SSD1305 recovery failed: %s",
                              recover_error.what());
      }
    }
    return {FlushStatus::Failed, failure_message};
  }
}

void Ssd1305DisplayNode::HandleEnableDisplay(
    const std_srvs::srv::SetBool::Request::SharedPtr request,
    const std_srvs::srv::SetBool::Response::SharedPtr response)
{
  try
  {
    std::scoped_lock device_lock(m_deviceMutex);
    if (request->data == m_displayEnabled)
    {
      response->success = true;
      response->message = m_displayEnabled ? "SSD1305 display enabled" : "SSD1305 display disabled";
      return;
    }

    if (!request->data)
    {
      m_device->SetDisplayEnabled(false);
      m_displayEnabled = false;
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
      m_displayEnabled = true;
      m_consecutiveFailures = 0;
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
    response->success = false;
    response->message = error.what();
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
        response->success = false;
        response->message = "SSD1305 clear failed: " + result.message;
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
  {
    std::scoped_lock lock(m_pendingMutex);
    if (m_framebufferConfig.invert_pixels != request->data)
    {
      m_framebufferConfig.invert_pixels = request->data;
      m_pendingBuffer.Invert();
      m_hasPendingFrame = true;
    }
  }
  response->success = true;
  response->message = m_framebufferConfig.invert_pixels ? "SSD1305 pixel inversion enabled"
                                                        : "SSD1305 pixel inversion disabled";
}

void Ssd1305DisplayNode::HandleSetContrast(
    const oasis_msgs::srv::SetDisplayContrast::Request::SharedPtr request,
    const oasis_msgs::srv::SetDisplayContrast::Response::SharedPtr response)
{
  try
  {
    std::scoped_lock device_lock(m_deviceMutex);
    m_device->SetContrast(request->contrast);
    m_deviceConfig.contrast = request->contrast;
    response->success = true;
    response->message = "SSD1305 contrast updated";
  }
  catch (const std::exception& error)
  {
    response->success = false;
    response->message = error.what();
  }
}

RCLCPP_COMPONENTS_REGISTER_NODE(OASIS::ROS::Ssd1305DisplayNode)
