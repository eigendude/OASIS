/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "Ens160Bme280Node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>

#include <rclcpp/qos.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <std_msgs/msg/header.hpp>

using OASIS::Environment::Bme280Config;
using OASIS::Environment::Bme280Sample;
using OASIS::Environment::Ens160Config;
using OASIS::Environment::Ens160Device;
using OASIS::Environment::Ens160Sample;
using OASIS::Environment::Ens160Status;
using OASIS::Environment::Ens160Validity;
using OASIS::ROS::Ens160Bme280Node;

namespace
{
constexpr const char* kNodeName = "ens160_bme280_driver";
constexpr unsigned int kRecoveryFailureThreshold = 5;
constexpr std::int64_t kLogThrottleMs = 10000;
} // namespace

Ens160Bme280Node::Ens160Bme280Node(const rclcpp::NodeOptions& options)
  : rclcpp::Node(kNodeName, options),
    m_config(ReadConfig()),
    m_aqiVariance(m_config.variance_window_samples),
    m_eco2Variance(m_config.variance_window_samples),
    m_tvocVariance(m_config.variance_window_samples),
    m_temperatureVariance(m_config.variance_window_samples),
    m_humidityVariance(m_config.variance_window_samples),
    m_pressureVariance(m_config.variance_window_samples)
{
  const rclcpp::SensorDataQoS qos;
  m_aqiPublisher = create_publisher<oasis_msgs::msg::AirQualityIndex>("air_quality_index", qos);
  m_eco2Publisher = create_publisher<oasis_msgs::msg::GasConcentration>("equivalent_co2", qos);
  m_tvocPublisher = create_publisher<oasis_msgs::msg::GasConcentration>("tvoc", qos);
  m_temperaturePublisher = create_publisher<sensor_msgs::msg::Temperature>("temperature", qos);
  m_humidityPublisher =
      create_publisher<sensor_msgs::msg::RelativeHumidity>("relative_humidity", qos);
  m_pressurePublisher = create_publisher<sensor_msgs::msg::FluidPressure>("pressure", qos);

  m_bme280Initialized = InitializeBme280();
  m_ens160Initialized = InitializeEns160();

  const auto period = std::chrono::duration<double>(1.0 / m_config.sample_rate_hz);
  m_timer = create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(period),
                              [this]() { Sample(); });
}

Ens160Bme280Node::Config Ens160Bme280Node::ReadConfig()
{
  declare_parameter("i2c_device", "/dev/i2c-1");
  declare_parameter("ens160_i2c_address", 0x53);
  declare_parameter("bme280_i2c_address", 0x77);
  declare_parameter("frame_id", "air_quality_sensor_link");
  declare_parameter("sample_rate_hz", 1.0);
  declare_parameter("variance_window_samples", 60);
  declare_parameter("publish_during_warmup", true);

  const std::int64_t ens160_address = get_parameter("ens160_i2c_address").as_int();
  const std::int64_t bme280_address = get_parameter("bme280_i2c_address").as_int();
  const std::int64_t variance_window = get_parameter("variance_window_samples").as_int();

  Config config;
  config.i2c_device = get_parameter("i2c_device").as_string();
  config.ens160_i2c_address = static_cast<std::uint8_t>(ens160_address);
  config.bme280_i2c_address = static_cast<std::uint8_t>(bme280_address);
  config.frame_id = get_parameter("frame_id").as_string();
  config.sample_rate_hz = get_parameter("sample_rate_hz").as_double();
  config.variance_window_samples = static_cast<std::size_t>(variance_window);
  config.publish_during_warmup = get_parameter("publish_during_warmup").as_bool();

  if (config.i2c_device.empty())
    throw std::invalid_argument("i2c_device must not be empty");
  if (ens160_address != 0x52 && ens160_address != 0x53)
    throw std::invalid_argument("ens160_i2c_address must be 0x52 or 0x53");
  if (bme280_address != 0x76 && bme280_address != 0x77)
    throw std::invalid_argument("bme280_i2c_address must be 0x76 or 0x77");
  if (!std::isfinite(config.sample_rate_hz) || config.sample_rate_hz <= 0.0 ||
      config.sample_rate_hz > 10.0)
    throw std::invalid_argument("sample_rate_hz must be finite and in (0, 10]");
  if (variance_window < 2 || variance_window > 100000)
    throw std::invalid_argument("variance_window_samples must be in [2, 100000]");
  if (config.frame_id.empty())
    throw std::invalid_argument("frame_id must not be empty");
  return config;
}

bool Ens160Bme280Node::InitializeEns160()
{
  if (!m_ens160.Initialize(Ens160Config{m_config.i2c_device, m_config.ens160_i2c_address}))
  {
    RCLCPP_ERROR(get_logger(), "%s", m_ens160.LastError().c_str());
    return false;
  }
  m_ens160Failures = 0;
  m_haveCompensation = false;
  ResetEns160Variances();
  RCLCPP_INFO(get_logger(), "Detected ENS160 at %s address 0x%02x, part ID 0x%04x",
              m_config.i2c_device.c_str(), m_config.ens160_i2c_address, m_ens160.PartId());
  return true;
}

bool Ens160Bme280Node::InitializeBme280()
{
  if (!m_bme280.Initialize(Bme280Config{m_config.i2c_device, m_config.bme280_i2c_address}))
  {
    RCLCPP_ERROR(get_logger(), "%s", m_bme280.LastError().c_str());
    return false;
  }
  m_bme280Failures = 0;
  ResetBme280Variances();
  RCLCPP_INFO(get_logger(), "Detected BME280 at %s address 0x%02x, chip ID 0x60",
              m_config.i2c_device.c_str(), m_config.bme280_i2c_address);
  return true;
}

void Ens160Bme280Node::Sample()
{
  if (!m_bme280Initialized)
    m_bme280Initialized = InitializeBme280();
  if (!m_ens160Initialized)
    m_ens160Initialized = InitializeEns160();

  std_msgs::msg::Header header;
  header.stamp = now();
  header.frame_id = m_config.frame_id;

  Bme280Sample bme_sample;
  bool have_bme_sample = false;
  if (m_bme280Initialized)
  {
    if (m_bme280.ReadSample(bme_sample) && std::isfinite(bme_sample.temperature_c) &&
        std::isfinite(bme_sample.pressure_pa) &&
        std::isfinite(bme_sample.relative_humidity_percent) && bme_sample.pressure_pa > 0.0 &&
        bme_sample.relative_humidity_percent >= -0.01 &&
        bme_sample.relative_humidity_percent <= 100.01)
    {
      bme_sample.relative_humidity_percent =
          std::clamp(bme_sample.relative_humidity_percent, 0.0, 100.0);
      have_bme_sample = true;
      m_bme280Failures = 0;
      PublishBme280(bme_sample, header);
    }
    else
    {
      HandleBme280Failure();
    }
  }

  if (!m_ens160Initialized)
    return;

  if (have_bme_sample)
  {
    if (!m_ens160.WriteEnvironmentalCompensation(bme_sample.temperature_c,
                                                 bme_sample.relative_humidity_percent))
    {
      HandleEns160Failure();
      return;
    }
    m_haveCompensation = true;
  }
  else if (!m_haveCompensation)
  {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), kLogThrottleMs,
                         "No valid BME280 sample yet; ENS160 is using reset-default "
                         "environmental compensation");
  }

  Ens160Status status;
  Ens160Sample ens_sample;
  if (!m_ens160.ReadStatus(status))
  {
    HandleEns160Failure();
    return;
  }
  if (status.error != 0)
  {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), kLogThrottleMs,
                          "ENS160 DEVICE_STATUS reports error code %u", status.error);
    HandleEns160Failure();
    return;
  }
  if (!status.new_data)
    return;
  if (!m_ens160.ReadSample(ens_sample))
  {
    HandleEns160Failure();
    return;
  }
  m_ens160Failures = 0;

  if (status.validity == Ens160Validity::Invalid)
  {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), kLogThrottleMs,
                          "ENS160 reports invalid air-quality output; skipping sample");
    return;
  }
  if (!Ens160Device::IsSampleValid(status, ens_sample))
  {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), kLogThrottleMs,
                         "ENS160 sample is outside datasheet ranges; skipping sample");
    return;
  }
  if (status.validity == Ens160Validity::Warmup)
  {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), kLogThrottleMs,
                         "ENS160 is warming up (typically 3 minutes)");
    if (!m_config.publish_during_warmup)
      return;
  }
  if (status.validity == Ens160Validity::InitialStartup)
  {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), kLogThrottleMs,
                         "ENS160 first-use conditioning is underway and may take up to 1 hour");
    if (!m_config.publish_during_warmup)
      return;
  }
  PublishEns160(ens_sample, header);
}

void Ens160Bme280Node::HandleBme280Failure()
{
  ++m_bme280Failures;
  RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), kLogThrottleMs, "%s",
                       m_bme280.LastError().c_str());
  if (m_bme280Failures >= kRecoveryFailureThreshold)
  {
    RCLCPP_ERROR(get_logger(), "BME280 failed %u consecutive cycles; reinitializing device",
                 m_bme280Failures);
    m_bme280Initialized = false;
    m_bme280Failures = 0;
    ResetBme280Variances();
  }
}

void Ens160Bme280Node::HandleEns160Failure()
{
  ++m_ens160Failures;
  RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), kLogThrottleMs, "%s",
                       m_ens160.LastError().c_str());
  if (m_ens160Failures >= kRecoveryFailureThreshold)
  {
    RCLCPP_ERROR(get_logger(), "ENS160 failed %u consecutive cycles; reinitializing device",
                 m_ens160Failures);
    m_ens160Initialized = false;
    m_ens160Failures = 0;
    ResetEns160Variances();
  }
}

void Ens160Bme280Node::PublishBme280(const Bme280Sample& sample,
                                     const std_msgs::msg::Header& header)
{
  const double humidity_ratio = sample.relative_humidity_percent / 100.0;
  m_temperatureVariance.Add(sample.temperature_c);
  m_humidityVariance.Add(humidity_ratio);
  m_pressureVariance.Add(sample.pressure_pa);

  sensor_msgs::msg::Temperature temperature;
  temperature.header = header;
  temperature.temperature = sample.temperature_c;
  temperature.variance = m_temperatureVariance.Variance();
  m_temperaturePublisher->publish(temperature);

  sensor_msgs::msg::RelativeHumidity humidity;
  humidity.header = header;
  humidity.relative_humidity = humidity_ratio;
  humidity.variance = m_humidityVariance.Variance();
  m_humidityPublisher->publish(humidity);

  sensor_msgs::msg::FluidPressure pressure;
  pressure.header = header;
  pressure.fluid_pressure = sample.pressure_pa;
  pressure.variance = m_pressureVariance.Variance();
  m_pressurePublisher->publish(pressure);
}

void Ens160Bme280Node::PublishEns160(const Ens160Sample& sample,
                                     const std_msgs::msg::Header& header)
{
  const double aqi = sample.air_quality_index;
  const double eco2_ppm = sample.equivalent_co2_ppm;
  const double tvoc_ppm = Ens160Device::TvocPpbToPpm(sample.tvoc_ppb);
  m_aqiVariance.Add(aqi);
  m_eco2Variance.Add(eco2_ppm);
  m_tvocVariance.Add(tvoc_ppm);

  oasis_msgs::msg::AirQualityIndex aqi_message;
  aqi_message.header = header;
  aqi_message.index = aqi;
  aqi_message.variance = m_aqiVariance.Variance();
  m_aqiPublisher->publish(aqi_message);

  oasis_msgs::msg::GasConcentration eco2_message;
  eco2_message.header = header;
  eco2_message.concentration_ppm = eco2_ppm;
  eco2_message.variance = m_eco2Variance.Variance();
  m_eco2Publisher->publish(eco2_message);

  oasis_msgs::msg::GasConcentration tvoc_message;
  tvoc_message.header = header;
  tvoc_message.concentration_ppm = tvoc_ppm;
  tvoc_message.variance = m_tvocVariance.Variance();
  m_tvocPublisher->publish(tvoc_message);
}

void Ens160Bme280Node::ResetBme280Variances()
{
  m_temperatureVariance.Reset();
  m_humidityVariance.Reset();
  m_pressureVariance.Reset();
}

void Ens160Bme280Node::ResetEns160Variances()
{
  m_aqiVariance.Reset();
  m_eco2Variance.Reset();
  m_tvocVariance.Reset();
}

RCLCPP_COMPONENTS_REGISTER_NODE(OASIS::ROS::Ens160Bme280Node)
