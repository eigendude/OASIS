/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "nodes/Ssd1305DisplayNode.hpp"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>

using OASIS::Display::SSD1305_DISPLAY_WIDTH;
using OASIS::Display::Ssd1305DeviceInterface;
using OASIS::Display::Ssd1305Framebuffer;
using OASIS::ROS::Ssd1305DisplayNode;

namespace
{
class FakeDevice final : public Ssd1305DeviceInterface
{
public:
  struct State
  {
    bool fail_full_frame{false};
    bool fail_page{false};
    bool fail_initialize{false};
    bool fail_recover_transport{false};
    bool fail_contrast{false};
    bool fail_display_on{false};
    bool fail_display_off{false};
    unsigned initialize_calls{0};
    unsigned fail_initialize_on_call{0};
    unsigned initialize_failures_remaining{0};
    unsigned full_frame_calls{0};
    unsigned fail_full_frame_on_call{0};
    std::vector<std::string> events;
    std::vector<std::size_t> pages;
    std::vector<Ssd1305Framebuffer::Buffer> full_frames;
    std::vector<Ssd1305Framebuffer::Buffer> page_frames;
    std::function<void()> during_full_frame;
  };

  explicit FakeDevice(std::shared_ptr<State> state) : m_state(std::move(state)) {}

  void RecoverTransport() override
  {
    m_state->events.emplace_back("recover_transport");
    if (m_state->fail_recover_transport)
      throw std::runtime_error("transport recovery failed");
  }

  void Initialize() override
  {
    ++m_state->initialize_calls;
    m_state->events.emplace_back("display_off");
    m_state->events.emplace_back("initialize");
    if (m_state->fail_initialize || m_state->initialize_calls == m_state->fail_initialize_on_call ||
        m_state->initialize_failures_remaining > 0)
    {
      if (m_state->initialize_failures_remaining > 0)
        --m_state->initialize_failures_remaining;
      throw std::runtime_error("initialize failed");
    }
    ConfigureOrientation();
    ConfigureAddressing();
  }

  void ConfigureOrientation() override { m_state->events.emplace_back("orientation"); }

  void ConfigureAddressing() override { m_state->events.emplace_back("addressing"); }

  void SetDisplayEnabled(bool enabled) override
  {
    m_state->events.emplace_back(enabled ? "display_on" : "display_off");
    if (m_state->fail_display_on && enabled)
      throw std::runtime_error("DISPLAY_ON failed");
    if (m_state->fail_display_off && !enabled)
      throw std::runtime_error("DISPLAY_OFF failed");
  }

  void SetContrast(std::uint8_t contrast) override
  {
    m_state->events.emplace_back("contrast:" + std::to_string(contrast));
    if (m_state->fail_contrast)
      throw std::runtime_error("contrast failed");
  }

  void WriteFullFrame(const Ssd1305Framebuffer::Buffer& framebuffer) override
  {
    ++m_state->full_frame_calls;
    m_state->events.emplace_back("full");
    m_state->full_frames.push_back(framebuffer);
    if (m_state->during_full_frame)
      m_state->during_full_frame();
    if (m_state->fail_full_frame || m_state->full_frame_calls == m_state->fail_full_frame_on_call)
      throw std::runtime_error("full frame write failed");
  }

  void WritePage(const Ssd1305Framebuffer::Buffer& framebuffer, std::size_t page) override
  {
    m_state->events.emplace_back("page:" + std::to_string(page));
    m_state->pages.push_back(page);
    m_state->page_frames.push_back(framebuffer);
    if (m_state->fail_page)
      throw std::runtime_error("page write failed");
  }

  void RestoreAfterInitialization(const Ssd1305Framebuffer::Buffer& framebuffer,
                                  std::uint8_t contrast) override
  {
    Initialize();
    SetContrast(contrast);
    WriteFullFrame(framebuffer);
  }

  void RestoreFramebufferState(const Ssd1305Framebuffer::Buffer& framebuffer, bool enabled) override
  {
    ConfigureOrientation();
    ConfigureAddressing();
    WriteFullFrame(framebuffer);
    if (enabled)
      SetDisplayEnabled(true);
  }

private:
  std::shared_ptr<State> m_state;
};
} // namespace

namespace OASIS::ROS
{
class Ssd1305DisplayNodeTestAccess
{
public:
  using FlushResult = Ssd1305DisplayNode::FlushResult;
  using FlushStatus = Ssd1305DisplayNode::FlushStatus;

  static void CancelUpdateTimer(Ssd1305DisplayNode& node) { node.m_updateTimer->cancel(); }

  static void CancelReconnectTimer(Ssd1305DisplayNode& node) { node.m_reconnectTimer->cancel(); }

  static bool Reconnect(Ssd1305DisplayNode& node) { return node.AttemptReconnect(); }

  static void RetryInitialConnection(Ssd1305DisplayNode& node) { node.AttemptInitialConnection(); }

  static void SetSleepFunction(Ssd1305DisplayNode& node, Ssd1305DisplayNode::SleepFunction sleep)
  {
    node.m_sleep = std::move(sleep);
  }

  static bool Ready(Ssd1305DisplayNode& node)
  {
    std::scoped_lock lock(node.m_deviceMutex);
    return node.m_controllerState == Ssd1305DisplayNode::ControllerState::Ready;
  }

  static bool Disconnected(Ssd1305DisplayNode& node)
  {
    std::scoped_lock lock(node.m_deviceMutex);
    return node.m_controllerState == Ssd1305DisplayNode::ControllerState::Disconnected;
  }

  static unsigned FailedReconnectAttempts(Ssd1305DisplayNode& node)
  {
    std::scoped_lock lock(node.m_deviceMutex);
    return node.m_failedReconnectAttempts;
  }

  static void SetPendingPixel(Ssd1305DisplayNode& node, std::size_t x, std::size_t y, bool lit)
  {
    auto image = std::make_shared<sensor_msgs::msg::Image>();
    image->width = SSD1305_DISPLAY_WIDTH;
    image->height = OASIS::Display::SSD1305_DISPLAY_HEIGHT;
    image->step = SSD1305_DISPLAY_WIDTH;
    image->encoding = "mono8";
    image->data.assign(image->width * image->height, 0);
    image->data[y * image->step + x] = lit ? 255 : 0;

    node.HandleImage(image);
  }

  static void SetHasPendingFrame(Ssd1305DisplayNode& node, bool pending)
  {
    std::scoped_lock lock(node.m_pendingMutex);
    node.m_hasPendingFrame = pending;
    if (pending)
      ++node.m_desiredGeneration;
  }

  static bool HasPendingFrame(Ssd1305DisplayNode& node)
  {
    std::scoped_lock lock(node.m_pendingMutex);
    return node.m_hasPendingFrame;
  }

  static bool DisplayEnabled(const Ssd1305DisplayNode& node) { return node.m_displayEnabled; }

  static void SetFrontBufferByte(Ssd1305DisplayNode& node, std::size_t index, std::uint8_t value)
  {
    std::scoped_lock lock(node.m_deviceMutex);
    node.m_frontBuffer[index] = value;
  }

  static std::uint8_t FrontBufferByte(Ssd1305DisplayNode& node, std::size_t index)
  {
    std::scoped_lock lock(node.m_deviceMutex);
    return node.m_frontBuffer[index];
  }

  static Ssd1305Framebuffer::Buffer FrontBuffer(Ssd1305DisplayNode& node)
  {
    std::scoped_lock lock(node.m_deviceMutex);
    return node.m_frontBuffer;
  }

  static std::uint8_t RequestedContrast(Ssd1305DisplayNode& node)
  {
    std::scoped_lock lock(node.m_deviceMutex);
    return node.m_deviceConfig.contrast;
  }

  static Ssd1305Framebuffer::Buffer PendingBuffer(Ssd1305DisplayNode& node)
  {
    std::scoped_lock lock(node.m_pendingMutex);
    return node.m_pendingBuffer.Data();
  }

  static Ssd1305DisplayNode::FlushResult Flush(Ssd1305DisplayNode& node)
  {
    return node.FlushPendingFrame();
  }

  static void HandleClear(Ssd1305DisplayNode& node,
                          const std_srvs::srv::Trigger::Request::SharedPtr request,
                          const std_srvs::srv::Trigger::Response::SharedPtr response)
  {
    node.HandleClearDisplay(request, response);
  }

  static void HandleEnable(Ssd1305DisplayNode& node,
                           const std_srvs::srv::SetBool::Request::SharedPtr request,
                           const std_srvs::srv::SetBool::Response::SharedPtr response)
  {
    node.HandleEnableDisplay(request, response);
  }

  static void HandleInvert(Ssd1305DisplayNode& node,
                           const std_srvs::srv::SetBool::Request::SharedPtr request,
                           const std_srvs::srv::SetBool::Response::SharedPtr response)
  {
    node.HandleSetInvert(request, response);
  }

  static void HandleContrast(
      Ssd1305DisplayNode& node,
      const oasis_msgs::srv::SetDisplayContrast::Request::SharedPtr request,
      const oasis_msgs::srv::SetDisplayContrast::Response::SharedPtr response)
  {
    node.HandleSetContrast(request, response);
  }
};
} // namespace OASIS::ROS

namespace
{
class Ssd1305DisplayNodeTest : public testing::Test
{
public:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }

  static rclcpp::NodeOptions Options(bool enabled)
  {
    return rclcpp::NodeOptions().parameter_overrides({
        rclcpp::Parameter("enabled", enabled),
        rclcpp::Parameter("update_rate_hz", 1000.0),
    });
  }

  std::shared_ptr<Ssd1305DisplayNode> MakeNode(bool enabled,
                                               std::shared_ptr<FakeDevice::State>& state)
  {
    state = std::make_shared<FakeDevice::State>();
    return MakeNodeWithState(enabled, state);
  }

  std::shared_ptr<Ssd1305DisplayNode> MakeNodeWithState(
      bool enabled, const std::shared_ptr<FakeDevice::State>& state)
  {
    auto sleep = [state](std::chrono::nanoseconds duration)
    { state->events.emplace_back("sleep_ms:" + std::to_string(duration.count() / 1000000)); };
    auto node = std::make_shared<Ssd1305DisplayNode>(
        Options(enabled), std::make_unique<FakeDevice>(state), std::move(sleep));
    OASIS::ROS::Ssd1305DisplayNodeTestAccess::CancelUpdateTimer(*node);
    OASIS::ROS::Ssd1305DisplayNodeTestAccess::CancelReconnectTimer(*node);
    return node;
  }
};
} // namespace

TEST_F(Ssd1305DisplayNodeTest, ClearWhileDisabledQueuesWithoutHardwareWrite)
{
  std::shared_ptr<FakeDevice::State> state;
  auto node = MakeNode(false, state);
  const std::size_t event_count = state->events.size();

  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto response = std::make_shared<std_srvs::srv::Trigger::Response>();
  OASIS::ROS::Ssd1305DisplayNodeTestAccess::HandleClear(*node, request, response);

  EXPECT_TRUE(response->success);
  EXPECT_EQ(response->message, "SSD1305 clear queued while display is disabled");
  EXPECT_EQ(state->events.size(), event_count);
  EXPECT_TRUE(OASIS::ROS::Ssd1305DisplayNodeTestAccess::HasPendingFrame(*node));
}

TEST_F(Ssd1305DisplayNodeTest, ClearWhileEnabledReportsWriteFailure)
{
  std::shared_ptr<FakeDevice::State> state;
  auto node = MakeNode(true, state);
  OASIS::ROS::Ssd1305DisplayNodeTestAccess::SetFrontBufferByte(*node, 0, 1);
  state->fail_page = true;

  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto response = std::make_shared<std_srvs::srv::Trigger::Response>();
  OASIS::ROS::Ssd1305DisplayNodeTestAccess::HandleClear(*node, request, response);

  EXPECT_TRUE(response->success);
  EXPECT_EQ(response->message, "SSD1305 clear queued for reconnect");
  EXPECT_TRUE(OASIS::ROS::Ssd1305DisplayNodeTestAccess::HasPendingFrame(*node));
}

TEST_F(Ssd1305DisplayNodeTest, NormalEnableWritesFullFrameThenTurnsOnWithoutRecovery)
{
  std::shared_ptr<FakeDevice::State> state;
  auto node = MakeNode(false, state);
  state->events.clear();
  OASIS::ROS::Ssd1305DisplayNodeTestAccess::SetPendingPixel(*node, 0, 0, true);
  OASIS::ROS::Ssd1305DisplayNodeTestAccess::SetHasPendingFrame(*node, true);

  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  auto response = std::make_shared<std_srvs::srv::SetBool::Response>();
  request->data = true;
  OASIS::ROS::Ssd1305DisplayNodeTestAccess::HandleEnable(*node, request, response);

  EXPECT_TRUE(response->success);
  EXPECT_TRUE(OASIS::ROS::Ssd1305DisplayNodeTestAccess::DisplayEnabled(*node));
  EXPECT_EQ(state->events, (std::vector<std::string>{"contrast:255", "orientation", "addressing",
                                                     "full", "display_on"}));
  EXPECT_TRUE(std::none_of(state->events.begin(), state->events.end(),
                           [](const auto& event) { return event.starts_with("sleep_ms:"); }));
  EXPECT_EQ(std::find(state->events.begin(), state->events.end(), "initialize"),
            state->events.end());
  EXPECT_EQ(OASIS::ROS::Ssd1305DisplayNodeTestAccess::FrontBufferByte(*node, 0), 1);
  EXPECT_FALSE(OASIS::ROS::Ssd1305DisplayNodeTestAccess::HasPendingFrame(*node));
}

TEST_F(Ssd1305DisplayNodeTest, FailedEnableLeavesDisplayDisabledAndPendingFrameIntact)
{
  std::shared_ptr<FakeDevice::State> state;
  auto node = MakeNode(false, state);
  OASIS::ROS::Ssd1305DisplayNodeTestAccess::SetPendingPixel(*node, 0, 0, true);
  OASIS::ROS::Ssd1305DisplayNodeTestAccess::SetHasPendingFrame(*node, true);
  state->fail_display_on = true;

  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  auto response = std::make_shared<std_srvs::srv::SetBool::Response>();
  request->data = true;
  OASIS::ROS::Ssd1305DisplayNodeTestAccess::HandleEnable(*node, request, response);

  EXPECT_TRUE(response->success);
  EXPECT_TRUE(OASIS::ROS::Ssd1305DisplayNodeTestAccess::DisplayEnabled(*node));
  EXPECT_TRUE(OASIS::ROS::Ssd1305DisplayNodeTestAccess::HasPendingFrame(*node));
  EXPECT_EQ(OASIS::ROS::Ssd1305DisplayNodeTestAccess::FrontBufferByte(*node, 0), 0);
}

TEST_F(Ssd1305DisplayNodeTest, OrdinaryDisableIsImmediate)
{
  std::shared_ptr<FakeDevice::State> state;
  auto node = MakeNode(true, state);
  state->events.clear();

  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  auto response = std::make_shared<std_srvs::srv::SetBool::Response>();
  request->data = false;
  OASIS::ROS::Ssd1305DisplayNodeTestAccess::HandleEnable(*node, request, response);

  EXPECT_TRUE(response->success);
  EXPECT_EQ(state->events, (std::vector<std::string>{"display_off"}));
}

TEST_F(Ssd1305DisplayNodeTest, PartialUpdateConfirmsOnlyWrittenPages)
{
  std::shared_ptr<FakeDevice::State> state;
  auto node = MakeNode(true, state);
  state->events.clear();
  OASIS::ROS::Ssd1305DisplayNodeTestAccess::SetPendingPixel(*node, 0, 16, true);
  OASIS::ROS::Ssd1305DisplayNodeTestAccess::SetHasPendingFrame(*node, true);

  const auto result = OASIS::ROS::Ssd1305DisplayNodeTestAccess::Flush(*node);

  EXPECT_EQ(result.status, OASIS::ROS::Ssd1305DisplayNodeTestAccess::FlushStatus::Updated);
  ASSERT_EQ(state->pages.size(), 1U);
  EXPECT_EQ(state->pages[0], 2U);
  EXPECT_EQ(
      OASIS::ROS::Ssd1305DisplayNodeTestAccess::FrontBufferByte(*node, 2U * SSD1305_DISPLAY_WIDTH),
      1);
  EXPECT_EQ(OASIS::ROS::Ssd1305DisplayNodeTestAccess::FrontBufferByte(*node, 0), 0);
}

TEST_F(Ssd1305DisplayNodeTest, SoftwareInversionTwiceRestoresPendingFramebuffer)
{
  std::shared_ptr<FakeDevice::State> state;
  auto node = MakeNode(false, state);
  OASIS::ROS::Ssd1305DisplayNodeTestAccess::SetPendingPixel(*node, 0, 0, true);
  const auto original = OASIS::ROS::Ssd1305DisplayNodeTestAccess::PendingBuffer(*node);

  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  auto response = std::make_shared<std_srvs::srv::SetBool::Response>();
  request->data = true;
  OASIS::ROS::Ssd1305DisplayNodeTestAccess::HandleInvert(*node, request, response);
  request->data = false;
  OASIS::ROS::Ssd1305DisplayNodeTestAccess::HandleInvert(*node, request, response);

  EXPECT_TRUE(response->success);
  EXPECT_TRUE(OASIS::ROS::Ssd1305DisplayNodeTestAccess::HasPendingFrame(*node));
  EXPECT_EQ(OASIS::ROS::Ssd1305DisplayNodeTestAccess::PendingBuffer(*node), original);
}

TEST_F(Ssd1305DisplayNodeTest, ShutdownBlanksThenTurnsOffWithoutRecovery)
{
  std::shared_ptr<FakeDevice::State> state;
  auto node = MakeNode(true, state);
  state->events.clear();

  node.reset();

  EXPECT_EQ(state->events, (std::vector<std::string>{"full", "display_off"}));
}

TEST_F(Ssd1305DisplayNodeTest, HealthyStartupFullySynchronizesController)
{
  std::shared_ptr<FakeDevice::State> state;
  auto node = MakeNode(true, state);

  EXPECT_TRUE(OASIS::ROS::Ssd1305DisplayNodeTestAccess::Ready(*node));
  EXPECT_EQ(state->events,
            (std::vector<std::string>{"display_off", "initialize", "orientation", "addressing",
                                      "contrast:255", "full", "sleep_ms:250", "orientation",
                                      "addressing", "full", "display_on"}));
  EXPECT_EQ(OASIS::ROS::Ssd1305DisplayNodeTestAccess::FrontBuffer(*node),
            OASIS::ROS::Ssd1305DisplayNodeTestAccess::PendingBuffer(*node));
  EXPECT_FALSE(OASIS::ROS::Ssd1305DisplayNodeTestAccess::HasPendingFrame(*node));
}

TEST_F(Ssd1305DisplayNodeTest, DisabledStartupRestoresStateWithoutPowerSettle)
{
  std::shared_ptr<FakeDevice::State> state;
  auto node = MakeNode(false, state);

  EXPECT_TRUE(OASIS::ROS::Ssd1305DisplayNodeTestAccess::Ready(*node));
  EXPECT_EQ(state->events, (std::vector<std::string>{"display_off", "initialize", "orientation",
                                                     "addressing", "contrast:255", "full"}));
}

TEST_F(Ssd1305DisplayNodeTest, ZeroPowerSettleStillRunsFinalRestoreWithoutSleeping)
{
  auto state = std::make_shared<FakeDevice::State>();
  const auto options = Options(true).parameter_overrides({
      rclcpp::Parameter("enabled", true),
      rclcpp::Parameter("update_rate_hz", 1000.0),
      rclcpp::Parameter("display_power_settle_sec", 0.0),
  });
  auto node = std::make_shared<Ssd1305DisplayNode>(
      options, std::make_unique<FakeDevice>(state), [state](std::chrono::nanoseconds duration)
      { state->events.emplace_back("sleep_ms:" + std::to_string(duration.count() / 1000000)); });

  EXPECT_EQ(state->events,
            (std::vector<std::string>{"display_off", "initialize", "orientation", "addressing",
                                      "contrast:255", "full", "orientation", "addressing", "full",
                                      "display_on"}));
}

TEST_F(Ssd1305DisplayNodeTest, InitialFramebufferFailureLeavesPendingAndDisconnected)
{
  auto state = std::make_shared<FakeDevice::State>();
  state->fail_full_frame_on_call = 1;
  auto node = MakeNodeWithState(true, state);

  EXPECT_TRUE(OASIS::ROS::Ssd1305DisplayNodeTestAccess::Disconnected(*node));
  EXPECT_TRUE(OASIS::ROS::Ssd1305DisplayNodeTestAccess::HasPendingFrame(*node));
  EXPECT_EQ(std::count(state->events.begin(), state->events.end(), "display_on"), 0);
}

TEST_F(Ssd1305DisplayNodeTest, FinalFramebufferFailureLeavesPendingAndDisconnected)
{
  auto state = std::make_shared<FakeDevice::State>();
  state->fail_full_frame_on_call = 2;
  auto node = MakeNodeWithState(true, state);

  EXPECT_TRUE(OASIS::ROS::Ssd1305DisplayNodeTestAccess::Disconnected(*node));
  EXPECT_TRUE(OASIS::ROS::Ssd1305DisplayNodeTestAccess::HasPendingFrame(*node));
  EXPECT_EQ(std::count(state->events.begin(), state->events.end(), "display_on"), 0);
}

TEST_F(Ssd1305DisplayNodeTest, ReconnectRecoversTransportBeforeControllerRestore)
{
  auto state = std::make_shared<FakeDevice::State>();
  state->fail_initialize = true;
  auto node = MakeNodeWithState(true, state);
  state->fail_initialize = false;
  state->events.clear();

  EXPECT_TRUE(OASIS::ROS::Ssd1305DisplayNodeTestAccess::Reconnect(*node));
  EXPECT_EQ(state->events, (std::vector<std::string>{
                               "recover_transport", "display_off", "initialize", "orientation",
                               "addressing", "contrast:255", "full", "sleep_ms:250", "orientation",
                               "addressing", "full", "display_on"}));
  EXPECT_TRUE(OASIS::ROS::Ssd1305DisplayNodeTestAccess::Ready(*node));
}

TEST_F(Ssd1305DisplayNodeTest, ReconnectTransportFailureDoesNotInitializeController)
{
  auto state = std::make_shared<FakeDevice::State>();
  state->fail_initialize = true;
  auto node = MakeNodeWithState(true, state);
  state->fail_initialize = false;
  state->fail_recover_transport = true;
  state->events.clear();

  EXPECT_FALSE(OASIS::ROS::Ssd1305DisplayNodeTestAccess::Reconnect(*node));
  EXPECT_EQ(state->events, (std::vector<std::string>{"recover_transport"}));
  EXPECT_TRUE(OASIS::ROS::Ssd1305DisplayNodeTestAccess::Disconnected(*node));
  EXPECT_EQ(OASIS::ROS::Ssd1305DisplayNodeTestAccess::FailedReconnectAttempts(*node), 1U);
}

TEST_F(Ssd1305DisplayNodeTest, GenerationChangeDuringSettleUsesLatestFramebuffer)
{
  auto state = std::make_shared<FakeDevice::State>();
  state->fail_initialize = true;
  auto node = MakeNodeWithState(true, state);
  state->fail_initialize = false;
  OASIS::ROS::Ssd1305DisplayNodeTestAccess::SetSleepFunction(
      *node, [&node](std::chrono::nanoseconds)
      { OASIS::ROS::Ssd1305DisplayNodeTestAccess::SetPendingPixel(*node, 7, 0, true); });

  OASIS::ROS::Ssd1305DisplayNodeTestAccess::RetryInitialConnection(*node);

  ASSERT_GE(state->full_frames.size(), 2U);
  EXPECT_EQ(state->full_frames.back()[7], 1);
  EXPECT_EQ(OASIS::ROS::Ssd1305DisplayNodeTestAccess::FrontBufferByte(*node, 7), 1);
  EXPECT_FALSE(OASIS::ROS::Ssd1305DisplayNodeTestAccess::HasPendingFrame(*node));
}

TEST_F(Ssd1305DisplayNodeTest, GenerationChangeDuringFinalWriteRemainsPendingUntilFlush)
{
  auto state = std::make_shared<FakeDevice::State>();
  state->fail_initialize = true;
  auto node = MakeNodeWithState(true, state);
  state->fail_initialize = false;
  OASIS::ROS::Ssd1305DisplayNodeTestAccess::SetPendingPixel(*node, 0, 0, true);
  state->during_full_frame = [&node, &state]()
  {
    if (state->full_frame_calls == 2)
    {
      state->during_full_frame = nullptr;
      OASIS::ROS::Ssd1305DisplayNodeTestAccess::SetPendingPixel(*node, 1, 0, true);
    }
  };

  OASIS::ROS::Ssd1305DisplayNodeTestAccess::RetryInitialConnection(*node);

  EXPECT_TRUE(OASIS::ROS::Ssd1305DisplayNodeTestAccess::Ready(*node));
  EXPECT_EQ(OASIS::ROS::Ssd1305DisplayNodeTestAccess::FrontBufferByte(*node, 0), 1);
  EXPECT_EQ(OASIS::ROS::Ssd1305DisplayNodeTestAccess::FrontBufferByte(*node, 1), 0);
  EXPECT_TRUE(OASIS::ROS::Ssd1305DisplayNodeTestAccess::HasPendingFrame(*node));
  EXPECT_EQ(OASIS::ROS::Ssd1305DisplayNodeTestAccess::Flush(*node).status,
            OASIS::ROS::Ssd1305DisplayNodeTestAccess::FlushStatus::Updated);
  EXPECT_EQ(OASIS::ROS::Ssd1305DisplayNodeTestAccess::FrontBufferByte(*node, 0), 0);
  EXPECT_EQ(OASIS::ROS::Ssd1305DisplayNodeTestAccess::FrontBufferByte(*node, 1), 1);
  EXPECT_FALSE(OASIS::ROS::Ssd1305DisplayNodeTestAccess::HasPendingFrame(*node));
}

TEST_F(Ssd1305DisplayNodeTest, DisplayOnFailurePreservesConfirmedStateUntilReconnect)
{
  auto state = std::make_shared<FakeDevice::State>();
  state->fail_initialize = true;
  auto node = MakeNodeWithState(true, state);
  state->fail_initialize = false;
  OASIS::ROS::Ssd1305DisplayNodeTestAccess::SetPendingPixel(*node, 4, 0, true);
  state->fail_display_on = true;
  state->events.clear();

  OASIS::ROS::Ssd1305DisplayNodeTestAccess::RetryInitialConnection(*node);

  EXPECT_TRUE(OASIS::ROS::Ssd1305DisplayNodeTestAccess::Disconnected(*node));
  EXPECT_TRUE(OASIS::ROS::Ssd1305DisplayNodeTestAccess::HasPendingFrame(*node));
  EXPECT_EQ(std::count(state->events.begin(), state->events.end(), "display_on"), 1);
  EXPECT_EQ(OASIS::ROS::Ssd1305DisplayNodeTestAccess::FrontBufferByte(*node, 4), 0);
  state->fail_display_on = false;
  EXPECT_TRUE(OASIS::ROS::Ssd1305DisplayNodeTestAccess::Reconnect(*node));
  EXPECT_EQ(OASIS::ROS::Ssd1305DisplayNodeTestAccess::FrontBufferByte(*node, 4), 1);
  EXPECT_FALSE(OASIS::ROS::Ssd1305DisplayNodeTestAccess::HasPendingFrame(*node));
}

TEST_F(Ssd1305DisplayNodeTest, PageFailureSuppressesWritesUntilRecovery)
{
  std::shared_ptr<FakeDevice::State> state;
  auto node = MakeNode(true, state);
  state->events.clear();
  state->fail_page = true;
  OASIS::ROS::Ssd1305DisplayNodeTestAccess::SetPendingPixel(*node, 0, 8, true);

  EXPECT_EQ(OASIS::ROS::Ssd1305DisplayNodeTestAccess::Flush(*node).status,
            OASIS::ROS::Ssd1305DisplayNodeTestAccess::FlushStatus::Failed);
  const std::size_t event_count = state->events.size();
  state->fail_page = false;
  EXPECT_EQ(OASIS::ROS::Ssd1305DisplayNodeTestAccess::Flush(*node).status,
            OASIS::ROS::Ssd1305DisplayNodeTestAccess::FlushStatus::Failed);
  EXPECT_EQ(state->events.size(), event_count);
  EXPECT_TRUE(OASIS::ROS::Ssd1305DisplayNodeTestAccess::Reconnect(*node));
  EXPECT_TRUE(OASIS::ROS::Ssd1305DisplayNodeTestAccess::Ready(*node));
}

TEST_F(Ssd1305DisplayNodeTest, FullFrameUpdateFailureRequiresRecovery)
{
  auto state = std::make_shared<FakeDevice::State>();
  const auto options = Options(true).parameter_overrides({
      rclcpp::Parameter("enabled", true),
      rclcpp::Parameter("update_rate_hz", 1000.0),
      rclcpp::Parameter("enable_partial_updates", false),
  });
  auto node = std::make_shared<Ssd1305DisplayNode>(options, std::make_unique<FakeDevice>(state),
                                                   [](std::chrono::nanoseconds) {});
  OASIS::ROS::Ssd1305DisplayNodeTestAccess::CancelUpdateTimer(*node);
  OASIS::ROS::Ssd1305DisplayNodeTestAccess::CancelReconnectTimer(*node);
  state->fail_full_frame = true;
  OASIS::ROS::Ssd1305DisplayNodeTestAccess::SetPendingPixel(*node, 1, 1, true);

  EXPECT_EQ(OASIS::ROS::Ssd1305DisplayNodeTestAccess::Flush(*node).status,
            OASIS::ROS::Ssd1305DisplayNodeTestAccess::FlushStatus::Failed);
  EXPECT_TRUE(OASIS::ROS::Ssd1305DisplayNodeTestAccess::Disconnected(*node));
  state->fail_full_frame = false;
  EXPECT_TRUE(OASIS::ROS::Ssd1305DisplayNodeTestAccess::Reconnect(*node));
}

TEST_F(Ssd1305DisplayNodeTest, ReconnectFailureCounterIncrementsAndResetsAfterSuccess)
{
  auto state = std::make_shared<FakeDevice::State>();
  state->fail_initialize = true;
  auto node = MakeNodeWithState(true, state);
  state->fail_initialize = false;
  state->initialize_failures_remaining = 2;

  EXPECT_FALSE(OASIS::ROS::Ssd1305DisplayNodeTestAccess::Reconnect(*node));
  EXPECT_EQ(OASIS::ROS::Ssd1305DisplayNodeTestAccess::FailedReconnectAttempts(*node), 1U);
  EXPECT_FALSE(OASIS::ROS::Ssd1305DisplayNodeTestAccess::Reconnect(*node));
  EXPECT_EQ(OASIS::ROS::Ssd1305DisplayNodeTestAccess::FailedReconnectAttempts(*node), 2U);
  EXPECT_TRUE(OASIS::ROS::Ssd1305DisplayNodeTestAccess::Reconnect(*node));
  EXPECT_EQ(OASIS::ROS::Ssd1305DisplayNodeTestAccess::FailedReconnectAttempts(*node), 0U);
}

TEST_F(Ssd1305DisplayNodeTest, DisconnectedServicesPreserveRequestedState)
{
  auto state = std::make_shared<FakeDevice::State>();
  state->fail_initialize = true;
  auto node = MakeNodeWithState(true, state);
  auto enable_request = std::make_shared<std_srvs::srv::SetBool::Request>();
  auto enable_response = std::make_shared<std_srvs::srv::SetBool::Response>();
  enable_request->data = false;
  OASIS::ROS::Ssd1305DisplayNodeTestAccess::HandleEnable(*node, enable_request, enable_response);
  auto contrast_request = std::make_shared<oasis_msgs::srv::SetDisplayContrast::Request>();
  auto contrast_response = std::make_shared<oasis_msgs::srv::SetDisplayContrast::Response>();
  contrast_request->contrast = 0x40;
  OASIS::ROS::Ssd1305DisplayNodeTestAccess::HandleContrast(*node, contrast_request,
                                                           contrast_response);

  EXPECT_FALSE(OASIS::ROS::Ssd1305DisplayNodeTestAccess::DisplayEnabled(*node));
  EXPECT_EQ(OASIS::ROS::Ssd1305DisplayNodeTestAccess::RequestedContrast(*node), 0x40);
  state->fail_initialize = false;
  state->events.clear();
  EXPECT_TRUE(OASIS::ROS::Ssd1305DisplayNodeTestAccess::Reconnect(*node));
  EXPECT_NE(std::find(state->events.begin(), state->events.end(), "contrast:64"),
            state->events.end());
  EXPECT_EQ(std::find(state->events.begin(), state->events.end(), "display_on"),
            state->events.end());
}

TEST_F(Ssd1305DisplayNodeTest, LiveContrastFailureQueuesRequestedValueForReconnect)
{
  std::shared_ptr<FakeDevice::State> state;
  auto node = MakeNode(true, state);
  state->fail_contrast = true;
  auto request = std::make_shared<oasis_msgs::srv::SetDisplayContrast::Request>();
  auto response = std::make_shared<oasis_msgs::srv::SetDisplayContrast::Response>();
  request->contrast = 0x40;

  OASIS::ROS::Ssd1305DisplayNodeTestAccess::HandleContrast(*node, request, response);

  EXPECT_TRUE(OASIS::ROS::Ssd1305DisplayNodeTestAccess::Disconnected(*node));
  EXPECT_EQ(OASIS::ROS::Ssd1305DisplayNodeTestAccess::RequestedContrast(*node), 0x40);
  state->fail_contrast = false;
  state->events.clear();
  EXPECT_TRUE(OASIS::ROS::Ssd1305DisplayNodeTestAccess::Reconnect(*node));
  EXPECT_NE(std::find(state->events.begin(), state->events.end(), "contrast:64"),
            state->events.end());
}

TEST_F(Ssd1305DisplayNodeTest, FailedReconnectPreservesPendingAndConfirmedBuffers)
{
  std::shared_ptr<FakeDevice::State> state;
  auto node = MakeNode(true, state);
  const auto confirmed = OASIS::ROS::Ssd1305DisplayNodeTestAccess::FrontBuffer(*node);
  OASIS::ROS::Ssd1305DisplayNodeTestAccess::SetPendingPixel(*node, 0, 0, true);
  state->fail_page = true;
  (void)OASIS::ROS::Ssd1305DisplayNodeTestAccess::Flush(*node);
  state->fail_page = false;
  state->fail_initialize = true;

  EXPECT_FALSE(OASIS::ROS::Ssd1305DisplayNodeTestAccess::Reconnect(*node));
  EXPECT_TRUE(OASIS::ROS::Ssd1305DisplayNodeTestAccess::HasPendingFrame(*node));
  EXPECT_EQ(OASIS::ROS::Ssd1305DisplayNodeTestAccess::FrontBuffer(*node), confirmed);
  EXPECT_EQ(OASIS::ROS::Ssd1305DisplayNodeTestAccess::FailedReconnectAttempts(*node), 1U);
}

TEST_F(Ssd1305DisplayNodeTest, RejectsInvalidReconnectIntervals)
{
  for (const double value : {0.0, -1.0, std::numeric_limits<double>::infinity(),
                             std::numeric_limits<double>::quiet_NaN()})
  {
    const auto options = rclcpp::NodeOptions().parameter_overrides(
        {rclcpp::Parameter("reconnect_interval_sec", value)});
    EXPECT_THROW(Ssd1305DisplayNode(
                     options, std::make_unique<FakeDevice>(std::make_shared<FakeDevice::State>())),
                 std::invalid_argument);
  }
}

TEST_F(Ssd1305DisplayNodeTest, RejectsInvalidDisplayPowerSettleValues)
{
  for (const double value :
       {-1.0, std::numeric_limits<double>::infinity(), std::numeric_limits<double>::quiet_NaN()})
  {
    const auto options = rclcpp::NodeOptions().parameter_overrides(
        {rclcpp::Parameter("display_power_settle_sec", value)});
    EXPECT_THROW(Ssd1305DisplayNode(
                     options, std::make_unique<FakeDevice>(std::make_shared<FakeDevice::State>())),
                 std::invalid_argument);
  }
}

TEST_F(Ssd1305DisplayNodeTest, ShutdownWhileDisconnectedDoesNotTouchHardware)
{
  auto state = std::make_shared<FakeDevice::State>();
  state->fail_initialize = true;
  auto node = MakeNodeWithState(true, state);
  const std::size_t event_count = state->events.size();

  node.reset();

  EXPECT_EQ(state->events.size(), event_count);
}


TEST_F(Ssd1305DisplayNodeTest, ShutdownSuppressesBlankAndDisplayOffFailures)
{
  std::shared_ptr<FakeDevice::State> state;
  auto node = MakeNode(true, state);
  state->events.clear();
  state->fail_full_frame = true;
  state->fail_display_off = true;

  EXPECT_NO_THROW(node.reset());

  EXPECT_EQ(state->events, (std::vector<std::string>{"full", "display_off"}));
}
