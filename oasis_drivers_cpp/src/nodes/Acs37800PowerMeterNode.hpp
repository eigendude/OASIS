/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#pragma once

#include "power_meter/PowerMeterCore.hpp"

#include <oasis_msgs/msg/power_meter.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/timer.hpp>

namespace OASIS::ROS
{
class Acs37800PowerMeterNode : public rclcpp::Node
{
public:
  explicit Acs37800PowerMeterNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~Acs37800PowerMeterNode() override = default;

private:
  OASIS::PowerMeter::Config ReadConfig();
  void PublishMeasurement();

  OASIS::PowerMeter::Config m_config;
  rclcpp::Publisher<oasis_msgs::msg::PowerMeter>::SharedPtr m_measurementPublisher;
  rclcpp::TimerBase::SharedPtr m_measurementTimer;
};
} // namespace OASIS::ROS
