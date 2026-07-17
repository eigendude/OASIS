/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "nodes/Ssd1305DisplayNode.hpp"

#include <algorithm>
#include <cstdint>
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
    bool fail_display_on{false};
    bool fail_display_off{false};
    std::vector<std::string> events;
    std::vector<std::size_t> pages;
    std::vector<Ssd1305Framebuffer::Buffer> full_frames;
    std::vector<Ssd1305Framebuffer::Buffer> page_frames;
    std::vector<Ssd1305Framebuffer::Buffer> recovered_frames;
  };

  explicit FakeDevice(std::shared_ptr<State> state) : m_state(std::move(state)) {}

  void Initialize() override { m_state->events.emplace_back("initialize"); }

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
  }

  void WriteFullFrame(const Ssd1305Framebuffer::Buffer& framebuffer) override
  {
    m_state->events.emplace_back("full");
    m_state->full_frames.push_back(framebuffer);
    if (m_state->fail_full_frame)
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

  void Recover(const Ssd1305Framebuffer::Buffer& framebuffer, bool enabled) override
  {
    m_state->events.emplace_back(enabled ? "recover_on" : "recover_off");
    m_state->recovered_frames.push_back(framebuffer);
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
    node.m_hasPendingFrame = pending;
  }

  static bool HasPendingFrame(const Ssd1305DisplayNode& node) { return node.m_hasPendingFrame; }

  static bool DisplayEnabled(const Ssd1305DisplayNode& node) { return node.m_displayEnabled; }

  static void SetFrontBufferByte(Ssd1305DisplayNode& node, std::size_t index, std::uint8_t value)
  {
    node.m_frontBuffer[index] = value;
  }

  static std::uint8_t FrontBufferByte(const Ssd1305DisplayNode& node, std::size_t index)
  {
    return node.m_frontBuffer[index];
  }

  static Ssd1305Framebuffer::Buffer PendingBuffer(const Ssd1305DisplayNode& node)
  {
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
    auto node =
        std::make_shared<Ssd1305DisplayNode>(Options(enabled), std::make_unique<FakeDevice>(state));
    OASIS::ROS::Ssd1305DisplayNodeTestAccess::CancelUpdateTimer(*node);
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

  EXPECT_FALSE(response->success);
  EXPECT_NE(response->message.find("page write failed"), std::string::npos);
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
  EXPECT_EQ(state->events, (std::vector<std::string>{"full", "contrast:255", "display_on"}));
  EXPECT_TRUE(state->recovered_frames.empty());
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

  EXPECT_FALSE(response->success);
  EXPECT_FALSE(OASIS::ROS::Ssd1305DisplayNodeTestAccess::DisplayEnabled(*node));
  EXPECT_TRUE(OASIS::ROS::Ssd1305DisplayNodeTestAccess::HasPendingFrame(*node));
  EXPECT_EQ(OASIS::ROS::Ssd1305DisplayNodeTestAccess::FrontBufferByte(*node, 0), 0);
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
  EXPECT_TRUE(state->recovered_frames.empty());
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
  EXPECT_TRUE(state->recovered_frames.empty());
}
