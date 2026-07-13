/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "Acs37800PowerMeterNode.hpp"

#include "power_meter/I2cMuxResolver.hpp"

#include <chrono>
#include <cstdint>

#include <rclcpp/qos.hpp>

using OASIS::PowerMeter::BuildSimulatedSample;
using OASIS::PowerMeter::Config;
using OASIS::PowerMeter::ResolveMuxAdapterIfEnabled;
using OASIS::PowerMeter::Sample;
using OASIS::ROS::Acs37800PowerMeterNode;

namespace
{
constexpr const char* kNodeName = "acs37800_power_meter";
constexpr const char* kSysfsI2cDevRoot = "/sys/class/i2c-dev";
} // namespace

Acs37800PowerMeterNode::Acs37800PowerMeterNode(const rclcpp::NodeOptions& options)
  : rclcpp::Node(kNodeName, options), m_config(ReadConfig())
{
  const auto adapter = ResolveMuxAdapterIfEnabled(kSysfsI2cDevRoot, m_config.resolve_i2c_adapter,
                                                  m_config.parent_i2c_bus, m_config.mux_address,
                                                  m_config.mux_channel);
  if (adapter)
  {
    RCLCPP_INFO(get_logger(),
                "Resolved ACS37800 mux channel: parent bus %d, mux address 0x%02x, "
                "channel %d -> %s (device address 0x%02x; simulated, device not opened)",
                m_config.parent_i2c_bus, m_config.mux_address, m_config.mux_channel,
                adapter->device_path.c_str(), m_config.i2c_address);
  }
  else
  {
    RCLCPP_INFO(get_logger(),
                "ACS37800 I2C adapter resolution is disabled; continuing in simulated mode");
  }

  m_measurementPublisher =
      create_publisher<oasis_msgs::msg::PowerMeter>("power_meter", rclcpp::SensorDataQoS());

  const auto measurement_period = std::chrono::duration<double>(1.0 / m_config.publish_rate_hz);
  m_measurementTimer =
      create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(measurement_period),
                        [this]() { PublishMeasurement(); });
}

Config Acs37800PowerMeterNode::ReadConfig()
{
  declare_parameter("parent_i2c_bus", 1);
  declare_parameter("mux_address", 112);
  declare_parameter("mux_channel", 0);
  declare_parameter("i2c_address", 96);
  declare_parameter("resolve_i2c_adapter", true);
  declare_parameter("publish_rate_hz", 10.0);
  declare_parameter("frame_id", "");
  declare_parameter("simulated_voltage", 0.0);
  declare_parameter("simulated_current", 0.0);
  declare_parameter("voltage_variance", 0.0);
  declare_parameter("current_variance", 0.0);
  declare_parameter("simulated_overcurrent", false);

  Config config;
  config.parent_i2c_bus = static_cast<int>(get_parameter("parent_i2c_bus").as_int());
  config.mux_address = static_cast<int>(get_parameter("mux_address").as_int());
  config.mux_channel = static_cast<int>(get_parameter("mux_channel").as_int());
  config.i2c_address = static_cast<int>(get_parameter("i2c_address").as_int());
  config.resolve_i2c_adapter = get_parameter("resolve_i2c_adapter").as_bool();
  config.publish_rate_hz = get_parameter("publish_rate_hz").as_double();
  config.frame_id = get_parameter("frame_id").as_string();
  config.simulated_voltage = get_parameter("simulated_voltage").as_double();
  config.simulated_current = get_parameter("simulated_current").as_double();
  config.voltage_variance = get_parameter("voltage_variance").as_double();
  config.current_variance = get_parameter("current_variance").as_double();
  config.simulated_overcurrent = get_parameter("simulated_overcurrent").as_bool();
  OASIS::PowerMeter::ValidateConfig(config);
  return config;
}

void Acs37800PowerMeterNode::PublishMeasurement()
{
  const rclcpp::Time timestamp = now();
  const Sample sample = BuildSimulatedSample(m_config);

  oasis_msgs::msg::PowerMeter message;
  message.header.stamp = timestamp;
  message.header.frame_id = m_config.frame_id;
  message.voltage = sample.voltage;
  message.voltage_variance = sample.voltage_variance;
  message.current = sample.current;
  message.current_variance = sample.current_variance;
  message.power = sample.power;
  message.power_variance = sample.power_variance;
  message.overcurrent = sample.overcurrent;
  message.status = static_cast<std::uint8_t>(sample.status);
  m_measurementPublisher->publish(message);
}
