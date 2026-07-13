/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "Acs37800PowerMeterNode.hpp"

#include "power_meter/LinuxI2cDeviceProbe.hpp"
#include "power_meter/RaspberryPiI2cConfig.hpp"

#include <chrono>
#include <cstdint>
#include <iomanip>
#include <limits>
#include <memory>
#include <set>
#include <sstream>
#include <stdexcept>
#include <utility>

#include <rclcpp/qos.hpp>
#include <rclcpp_components/register_node_macro.hpp>

using OASIS::PowerMeter::Sample;
using OASIS::ROS::Acs37800PowerMeterNode;

namespace
{
// ROS parameters
constexpr const char* DEFAULT_NODE_NAME = "acs37800_power_meter";

// Timing parameters
constexpr const char* PARAM_PUBLISH_RATE_HZ = "publish_rate_hz";
constexpr double DEFAULT_PUBLISH_RATE_HZ = 10.0;

// Hardware parameters
constexpr const char* PARAM_BOOT_CONFIG_PATH = "boot_config_path";
constexpr const char* PARAM_I2C_SYSFS_ROOT = "i2c_sysfs_root";
constexpr const char* PARAM_MUX_ADDRESS = "mux_address";
constexpr const char* PARAM_POWER_METER_ADDRESS = "power_meter_address";
constexpr const char* PARAM_POWER_METER_IDS = "power_meter_ids";
constexpr const char* DEFAULT_BOOT_CONFIG_PATH = "/boot/firmware/config.txt";
constexpr const char* DEFAULT_I2C_SYSFS_ROOT = "/sys/bus/i2c/devices";
constexpr int DEFAULT_MUX_ADDRESS = 0x70;
constexpr int DEFAULT_POWER_METER_ADDRESS = 0x60;

// ACS37800 conversion and runtime policy parameters
constexpr const char* PARAM_CURRENT_SENSE_RANGE_AMPS = "current_sense_range_amps";
constexpr const char* PARAM_VOLTAGE_DIVIDER_RESISTANCE_OHMS = "voltage_divider_resistance_ohms";
constexpr const char* PARAM_VOLTAGE_SENSE_RESISTANCE_OHMS = "voltage_sense_resistance_ohms";
constexpr const char* PARAM_EXPECTED_CRS_SNS = "expected_crs_sns";
constexpr const char* PARAM_DISCONNECT_AFTER_FAILURES = "disconnect_after_failures";

std::string HexAddress(int address)
{
  std::ostringstream stream;
  stream << "0x" << std::hex << address;
  return stream.str();
}

std::unique_ptr<OASIS::PowerMeter::IAcs37800Device> MakeDevice(
    const std::string& path, int address, const OASIS::PowerMeter::Acs37800Config& config)
{
  return std::make_unique<OASIS::PowerMeter::Acs37800Device>(path, address, config);
}

std::uint8_t ToMessageStatus(OASIS::PowerMeter::Status status)
{
  switch (status)
  {
    case OASIS::PowerMeter::Status::Ok:
      return oasis_msgs::msg::PowerMeter::STATUS_OK;
    case OASIS::PowerMeter::Status::Stale:
      return oasis_msgs::msg::PowerMeter::STATUS_STALE;
    case OASIS::PowerMeter::Status::Disconnected:
      return oasis_msgs::msg::PowerMeter::STATUS_DISCONNECTED;
    case OASIS::PowerMeter::Status::Error:
      return oasis_msgs::msg::PowerMeter::STATUS_ERROR;
  }

  throw std::invalid_argument("Unknown power-meter status");
}
} // namespace

Acs37800PowerMeterNode::Acs37800PowerMeterNode(const rclcpp::NodeOptions& options)
  : Acs37800PowerMeterNode(
        options, std::make_unique<OASIS::PowerMeter::LinuxI2cDeviceProbe>(), MakeDevice)
{
}

Acs37800PowerMeterNode::Acs37800PowerMeterNode(
    const rclcpp::NodeOptions& options, std::unique_ptr<OASIS::PowerMeter::II2cDeviceProbe> probe)
  : Acs37800PowerMeterNode(options, std::move(probe), MakeDevice)
{
}

Acs37800PowerMeterNode::Acs37800PowerMeterNode(
    const rclcpp::NodeOptions& options,
    std::unique_ptr<OASIS::PowerMeter::II2cDeviceProbe> probe,
    DeviceFactory device_factory)
  : rclcpp::Node(DEFAULT_NODE_NAME, options),
    m_probe(std::move(probe)),
    m_deviceFactory(std::move(device_factory))
{
  if (!m_probe)
    throw std::invalid_argument("I2C device probe must not be null");
  if (!m_deviceFactory)
    throw std::invalid_argument("ACS37800 device factory must not be empty");

  ReadConfig();
  DiscoverPowerMeters();

  const auto measurement_period = std::chrono::duration<double>(1.0 / m_publishRateHz);
  const auto measurement_period_ns =
      std::chrono::duration_cast<std::chrono::nanoseconds>(measurement_period);
  if (measurement_period_ns.count() <= 0)
    throw std::invalid_argument("publish_rate_hz is too large to represent as a timer period");

  CreatePublishers();
  m_measurementTimer =
      create_wall_timer(measurement_period_ns, [this]() { PublishMeasurements(); });
}

void Acs37800PowerMeterNode::ReadConfig()
{
  declare_parameter(PARAM_PUBLISH_RATE_HZ, DEFAULT_PUBLISH_RATE_HZ);
  declare_parameter(PARAM_BOOT_CONFIG_PATH, DEFAULT_BOOT_CONFIG_PATH);
  declare_parameter(PARAM_I2C_SYSFS_ROOT, DEFAULT_I2C_SYSFS_ROOT);
  declare_parameter(PARAM_MUX_ADDRESS, DEFAULT_MUX_ADDRESS);
  declare_parameter(PARAM_POWER_METER_ADDRESS, DEFAULT_POWER_METER_ADDRESS);
  declare_parameter<std::vector<std::string>>(PARAM_POWER_METER_IDS, std::vector<std::string>{});
  declare_parameter(PARAM_CURRENT_SENSE_RANGE_AMPS, 30.0);
  declare_parameter(PARAM_VOLTAGE_DIVIDER_RESISTANCE_OHMS, 2000000.0);
  declare_parameter(PARAM_VOLTAGE_SENSE_RESISTANCE_OHMS, 8200.0);
  declare_parameter(PARAM_EXPECTED_CRS_SNS, 4);
  declare_parameter(PARAM_DISCONNECT_AFTER_FAILURES, 3);

  m_publishRateHz = get_parameter(PARAM_PUBLISH_RATE_HZ).as_double();
  m_bootConfigPath = get_parameter(PARAM_BOOT_CONFIG_PATH).as_string();
  m_i2cSysfsRoot = get_parameter(PARAM_I2C_SYSFS_ROOT).as_string();
  m_muxAddress = static_cast<int>(get_parameter(PARAM_MUX_ADDRESS).as_int());
  m_powerMeterAddress = static_cast<int>(get_parameter(PARAM_POWER_METER_ADDRESS).as_int());
  m_powerMeterIds = get_parameter(PARAM_POWER_METER_IDS).as_string_array();
  m_deviceConfig.current_sense_range_amps =
      get_parameter(PARAM_CURRENT_SENSE_RANGE_AMPS).as_double();
  m_deviceConfig.voltage_divider_resistance_ohms =
      get_parameter(PARAM_VOLTAGE_DIVIDER_RESISTANCE_OHMS).as_double();
  m_deviceConfig.voltage_sense_resistance_ohms =
      get_parameter(PARAM_VOLTAGE_SENSE_RESISTANCE_OHMS).as_double();
  const auto expected_crs_sns = get_parameter(PARAM_EXPECTED_CRS_SNS).as_int();
  const auto disconnect_after_failures = get_parameter(PARAM_DISCONNECT_AFTER_FAILURES).as_int();
  if (expected_crs_sns < 0)
    throw std::invalid_argument("expected_crs_sns must be in [0, 7]");
  if (disconnect_after_failures <= 0)
    throw std::invalid_argument("disconnect_after_failures must be positive");
  m_deviceConfig.expected_crs_sns = static_cast<unsigned>(expected_crs_sns);
  m_disconnectAfterFailures = static_cast<unsigned>(disconnect_after_failures);

  OASIS::PowerMeter::ValidatePublishRate(m_publishRateHz);
  OASIS::PowerMeter::ValidateI2cAddress(m_muxAddress, PARAM_MUX_ADDRESS);
  OASIS::PowerMeter::ValidateI2cAddress(m_powerMeterAddress, PARAM_POWER_METER_ADDRESS);
  OASIS::PowerMeter::ValidateAcs37800Config(m_deviceConfig);
}

void Acs37800PowerMeterNode::DiscoverPowerMeters()
{
  const auto boot_config =
      OASIS::PowerMeter::ParseRaspberryPiI2cConfig(m_bootConfigPath, m_muxAddress);
  RCLCPP_INFO(get_logger(), "Parsed PCA9548 overlay from %s: mux address %s",
              m_bootConfigPath.c_str(), HexAddress(boot_config.mux_address).c_str());

  const auto mux = OASIS::PowerMeter::DiscoverMuxDevice(m_i2cSysfsRoot, m_muxAddress);
  RCLCPP_INFO(get_logger(), "Discovered PCA9548 mux: parent bus %d, address %s", mux.parent_bus,
              HexAddress(mux.address).c_str());
  const auto channels = OASIS::PowerMeter::DiscoverMuxChannels(mux);
  for (const auto& channel : channels)
  {
    RCLCPP_INFO(get_logger(), "Probing mux channel %d via %s at address %s", channel.channel,
                channel.device_path.c_str(), HexAddress(m_powerMeterAddress).c_str());
  }
  const auto adapters =
      OASIS::PowerMeter::ProbeDevicesOnMux(channels, m_powerMeterAddress, *m_probe);

  std::set<int> detected_channels;
  for (const auto& adapter : adapters)
    detected_channels.insert(adapter.channel);
  for (const auto& channel : channels)
  {
    if (detected_channels.contains(channel.channel))
    {
      RCLCPP_INFO(get_logger(), "Power meter detected on mux channel %d via %s", channel.channel,
                  channel.device_path.c_str());
    }
    else
    {
      RCLCPP_INFO(get_logger(), "No power meter detected on mux channel %d via %s", channel.channel,
                  channel.device_path.c_str());
    }
  }
  if (adapters.empty())
  {
    throw std::runtime_error("No power meters responded at address " +
                             HexAddress(m_powerMeterAddress) +
                             " on the discovered PCA9548 mux channels");
  }

  ValidatePowerMeterIds(adapters.size());

  m_powerMeters.reserve(adapters.size());
  for (std::size_t index = 0; index < adapters.size(); ++index)
  {
    const std::string& id = m_powerMeterIds[index];
    std::unique_ptr<OASIS::PowerMeter::IAcs37800Device> device;
    try
    {
      device = m_deviceFactory(adapters[index].device_path, m_powerMeterAddress, m_deviceConfig);
      if (!device)
        throw std::runtime_error("device factory returned null");
    }
    catch (const std::exception& error)
    {
      throw std::invalid_argument("Power meter '" + id + "': " + error.what());
    }

    const auto& info = device->GetInfo();
    RCLCPP_INFO(get_logger(),
                "Power meter '%s': parent bus %d, mux channel %d, adapter %s, "
                "address %s, current range %.0f A, crs_sns %u, voltage scaling %.9g",
                id.c_str(), mux.parent_bus, adapters[index].channel,
                adapters[index].device_path.c_str(), HexAddress(m_powerMeterAddress).c_str(),
                m_deviceConfig.current_sense_range_amps, info.crs_sns, info.voltage_multiplier);
    m_powerMeters.push_back({id, mux.parent_bus, adapters[index].channel, m_powerMeterAddress,
                             adapters[index], std::move(device), nullptr});
  }
}

void Acs37800PowerMeterNode::ValidatePowerMeterIds(std::size_t discovered_meter_count) const
{
  if (m_powerMeterIds.size() != discovered_meter_count)
  {
    throw std::invalid_argument(
        "power_meter_ids contains " + std::to_string(m_powerMeterIds.size()) + " IDs, but " +
        std::to_string(discovered_meter_count) + " power meters were discovered");
  }

  std::set<std::string> seen_ids;
  for (std::size_t index = 0; index < m_powerMeterIds.size(); ++index)
  {
    const std::string& id = m_powerMeterIds[index];
    if (id.empty())
    {
      throw std::invalid_argument("power_meter_ids contains an empty ID at index " +
                                  std::to_string(index));
    }
    if (!seen_ids.insert(id).second)
      throw std::invalid_argument("power_meter_ids contains duplicate ID '" + id + "'");
  }
}

void Acs37800PowerMeterNode::CreatePublishers()
{
  for (PowerMeterInstance& meter : m_powerMeters)
  {
    meter.publisher =
        create_publisher<oasis_msgs::msg::PowerMeter>(meter.id, rclcpp::SensorDataQoS());
  }
}

void Acs37800PowerMeterNode::PublishMeasurements()
{
  // The ACS37800 has no hardware timestamp, so all meters receive the shared
  // host acquisition timestamp for this callback
  const rclcpp::Time timestamp = now();
  for (PowerMeterInstance& meter : m_powerMeters)
  {
    Sample sample;
    try
    {
      sample = meter.device->ReadSample();
      if (meter.consecutive_failures > 0)
      {
        RCLCPP_INFO(get_logger(), "Power meter '%s' recovered after %u failed reads%s",
                    meter.id.c_str(), meter.consecutive_failures,
                    meter.was_disconnected ? " from disconnected state" : "");
      }
      meter.consecutive_failures = 0;
      meter.was_disconnected = false;
      RCLCPP_DEBUG(get_logger(),
                   "Power meter '%s' adapter %s mux channel %d: "
                   "reg 0x2A=0x%08x voltage=%.9g V current=%.9g A, "
                   "reg 0x2C=0x%08x power=%.9g W, reg 0x2D=0x%08x fault=%d",
                   meter.id.c_str(), meter.adapter.device_path.c_str(), meter.mux_channel,
                   sample.raw_voltage_current_register, sample.voltage, sample.current,
                   sample.raw_power_register, sample.power, sample.raw_fault_register,
                   sample.overcurrent);
    }
    catch (const std::exception& error)
    {
      ++meter.consecutive_failures;
      const bool disconnected = meter.consecutive_failures >= m_disconnectAfterFailures;
      if (disconnected && !meter.was_disconnected)
      {
        RCLCPP_ERROR(get_logger(),
                     "Power meter '%s' disconnected after %u consecutive failed reads",
                     meter.id.c_str(), meter.consecutive_failures);
      }
      meter.was_disconnected = disconnected;
      sample.voltage = std::numeric_limits<double>::quiet_NaN();
      sample.current = std::numeric_limits<double>::quiet_NaN();
      sample.power = std::numeric_limits<double>::quiet_NaN();
      sample.voltage_variance = 0.0;
      sample.current_variance = 0.0;
      sample.power_variance = 0.0;
      sample.overcurrent = false;
      sample.status =
          disconnected ? OASIS::PowerMeter::Status::Disconnected : OASIS::PowerMeter::Status::Error;
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
                            "Power meter '%s' read failed (%u consecutive): %s", meter.id.c_str(),
                            meter.consecutive_failures, error.what());
    }

    oasis_msgs::msg::PowerMeter message;
    message.header.stamp = timestamp;
    message.header.frame_id = meter.id;
    message.voltage = sample.voltage;
    message.voltage_variance = sample.voltage_variance;
    message.current = sample.current;
    message.current_variance = sample.current_variance;
    message.power = sample.power;
    message.power_variance = sample.power_variance;
    message.overcurrent = sample.overcurrent;
    message.status = ToMessageStatus(sample.status);
    meter.publisher->publish(message);
  }
}

RCLCPP_COMPONENTS_REGISTER_NODE(OASIS::ROS::Acs37800PowerMeterNode)
