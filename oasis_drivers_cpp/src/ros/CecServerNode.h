/*
 *  Copyright (C) 2021 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#pragma once

#include "cec/ICecCallback.h"

#include <oasis_msgs/msg/power_event.hpp>
#include <oasis_msgs/srv/power_control.hpp>

#include <memory>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/service.hpp>
#include <string>

namespace rclcpp
{
  class Node;
}

namespace OASIS
{
namespace CEC
{
  class CecServer;
}

namespace ROS
{

class CecServerNode : public rclcpp::Node, public CEC::ICecCallback
{
public:
  CecServerNode();
  ~CecServerNode() override;

  // Getters
  const std::string& GetSystemName() const { return m_systemName; }

  // Lifecycle functions
  void RegisterServer(CEC::CecServer& cecServer);
  void UnregisterServer();

  // Implementation of ICecCallback
  void OnPowerOn(const CEC::ICecAdapter& adapter) override;
  void OnPowerOff(const CEC::ICecAdapter& adapter) override;

private:
  // ROS interface
  void OnPowerControl(const std::shared_ptr<oasis_msgs::srv::PowerControl::Request> request,
      std::shared_ptr<oasis_msgs::srv::PowerControl::Response>);

  // Internal details
  void PublishPowerEvent(const std::string& devicePath, bool bPowerOn);

  // ROS parameters
  const std::string m_systemName;
  std::shared_ptr<rclcpp::Publisher<oasis_msgs::msg::PowerEvent>> m_publisher;
  rclcpp::Service<oasis_msgs::srv::PowerControl>::SharedPtr m_service; // TODO

  // CEC parameters
  CEC::CecServer* m_cecServer = nullptr;
};

}
}
