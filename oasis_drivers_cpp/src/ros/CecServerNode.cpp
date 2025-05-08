/*
 *  Copyright (C) 2021-2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "CecServerNode.h"

#include "cec/CecServer.h"
#include "cec/ICecAdapter.h"

#include <functional>

#include <oasis_msgs/msg/power_mode.hpp>
#include <rclcpp/node.hpp>
#include <std_msgs/msg/header.hpp>

namespace
{
constexpr const char* NODE_NAME = "cec_server";

constexpr const char* MACHINE_NAME = "zotac"; // TODO

constexpr const char* POWER_EVENT_TOPIC = "power_event";
constexpr const char* POWER_CONTROL_SERVICE = "power_control";

// TODO: Hardware config
constexpr const char* HDMI_CEC_ADAPTER = "/sys/devices/pci0000:00/0000:00:14.0/usb2/2-6";
constexpr const char* DISPLAY_PORT_CEC_ADAPTER = "/sys/devices/pci0000:00/0000:00:14.0/usb2/2-5";
constexpr const char* LEFT_TV = HDMI_CEC_ADAPTER;
constexpr const char* RIGHT_TV = DISPLAY_PORT_CEC_ADAPTER;
} // namespace

using namespace OASIS;
using namespace ROS;

using std::placeholders::_1;
using std::placeholders::_2;

CecServerNode::CecServerNode() : rclcpp::Node(NODE_NAME), m_systemName(MACHINE_NAME)
{
  using PowerControl = oasis_msgs::srv::PowerControl;
  using PowerEvent = oasis_msgs::msg::PowerEvent;

  // Create publisher
  m_publisher = create_publisher<PowerEvent>(POWER_EVENT_TOPIC, 10);

  // Create service
  m_service = create_service<PowerControl>(POWER_CONTROL_SERVICE,
                                           std::bind(&CecServerNode::OnPowerControl, this, _1, _2));
}

CecServerNode::~CecServerNode() = default;

void CecServerNode::RegisterServer(OASIS::CEC::CecServer& cecServer)
{
  m_cecServer = &cecServer;
}

void CecServerNode::UnregisterServer()
{
  m_cecServer = nullptr;
}

void CecServerNode::OnPowerOn(const OASIS::CEC::ICecAdapter& adapter)
{
  PublishPowerEvent(adapter.GetDevicePath(), true);
}

void CecServerNode::OnPowerOff(const OASIS::CEC::ICecAdapter& adapter)
{
  PublishPowerEvent(adapter.GetDevicePath(), false);
}

void CecServerNode::OnPowerControl(
    const std::shared_ptr<oasis_msgs::srv::PowerControl::Request> request,
    std::shared_ptr<oasis_msgs::srv::PowerControl::Response>)
{
  using PowerMode = oasis_msgs::msg::PowerMode;

  const std::string& device = request->device;
  const std::string& mode = request->power_mode;

  RCLCPP_INFO(get_logger(), "ROS: Power request: %s %s", mode.c_str(), device.c_str());

  if (m_cecServer != nullptr)
  {
    if (mode == PowerMode::ON)
      m_cecServer->PowerOnDevice(device);
    else if (mode == PowerMode::OFF)
      m_cecServer->StandbyDevice(device);
  }
  else
  {
    RCLCPP_ERROR(get_logger(), "CEC server not registered");
    return;
  }
}

void CecServerNode::PublishPowerEvent(const std::string& devicePath, bool bPowerOn)
{
  using Header = std_msgs::msg::Header;
  using PowerEvent = oasis_msgs::msg::PowerEvent;
  using PowerMode = oasis_msgs::msg::PowerMode;

  auto header = Header();
  header.stamp = get_clock()->now();
  header.frame_id = m_systemName; // TODO

  auto message = PowerEvent();
  message.header = std::move(header);
  message.system = MACHINE_NAME;
  message.device = devicePath;
  message.power_mode = bPowerOn ? PowerMode::ON : PowerMode::OFF;

  m_publisher->publish(message);
}
