/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#pragma once

#include "power_meter/Acs37800Device.hpp"
#include "power_meter/I2cDeviceProbe.hpp"
#include "power_meter/I2cMuxResolver.hpp"
#include "power_meter/SampleMovingAverage.hpp"

#include <filesystem>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <oasis_msgs/msg/power_meter.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/timer.hpp>

namespace OASIS::ROS
{
class Acs37800PowerMeterNode : public rclcpp::Node
{
public:
  using DeviceFactory = std::function<std::unique_ptr<OASIS::PowerMeter::IAcs37800Device>(
      const std::string&, int, const OASIS::PowerMeter::Acs37800Config&)>;

  explicit Acs37800PowerMeterNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  Acs37800PowerMeterNode(const rclcpp::NodeOptions& options,
                         std::unique_ptr<OASIS::PowerMeter::II2cDeviceProbe> probe);
  Acs37800PowerMeterNode(const rclcpp::NodeOptions& options,
                         std::unique_ptr<OASIS::PowerMeter::II2cDeviceProbe> probe,
                         DeviceFactory device_factory);
  ~Acs37800PowerMeterNode() override = default;

private:
  struct PowerMeterInstance
  {
    std::string id;
    int parent_i2c_bus;
    int mux_channel;
    int i2c_address;
    OASIS::PowerMeter::MuxChannelAdapter adapter;
    std::unique_ptr<OASIS::PowerMeter::IAcs37800Device> device;
    rclcpp::Publisher<oasis_msgs::msg::PowerMeter>::SharedPtr publisher;
    unsigned consecutive_failures{0};
    bool was_disconnected{false};
    OASIS::PowerMeter::SampleMovingAverage filter;
    OASIS::PowerMeter::ActivePowerInvariantMonitor invariant_monitor;
  };

  void ReadConfig();
  void DiscoverPowerMeters();
  void ValidatePowerMeterIds(std::size_t discovered_meter_count) const;
  void CreatePublishers();
  void PublishMeasurements();

  double m_publishRateHz{0.0};
  int m_muxAddress{0};
  int m_powerMeterAddress{0};
  unsigned m_disconnectAfterFailures{0};
  unsigned m_filterLength{3};
  std::vector<std::string> m_powerMeterIds;
  std::filesystem::path m_bootConfigPath;
  std::filesystem::path m_i2cSysfsRoot;
  OASIS::PowerMeter::Acs37800Config m_deviceConfig{};
  std::unique_ptr<OASIS::PowerMeter::II2cDeviceProbe> m_probe;
  DeviceFactory m_deviceFactory;
  std::vector<PowerMeterInstance> m_powerMeters;
  rclcpp::TimerBase::SharedPtr m_measurementTimer;
};
} // namespace OASIS::ROS
