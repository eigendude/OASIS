/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "Mmc5983maMagnetometerNode.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <stdexcept>
#include <thread>

#include <rclcpp/rclcpp.hpp>

using namespace OASIS::ROS;
using namespace std::chrono_literals;

namespace
{

constexpr const char* NODE_NAME = "mmc5983ma_magnetometer_driver";

constexpr const char* MAG_TOPIC = "magnetic_field";
constexpr const char* DEFAULT_FRAME_ID = "mag_link";

constexpr const char* DEFAULT_I2C_DEVICE = "/dev/i2c-1";
constexpr std::uint8_t DEFAULT_I2C_ADDRESS = 0x30;
constexpr double DEFAULT_PUBLISH_RATE_HZ = 50.0;
constexpr double DEFAULT_RAW_RATE_HZ = 100.0;
constexpr std::uint8_t DEFAULT_BANDWIDTH_MODE = 1;
constexpr int DEFAULT_MEASUREMENT_TIMEOUT_MS = 20;

constexpr double DEFAULT_STATIONARY_WINDOW_SEC = 2.0;
constexpr double DEFAULT_EWMA_TAU_SEC = 10.0;
constexpr double DEFAULT_PRIOR_STRENGTH_SAMPLES = 100.0;
constexpr double DEFAULT_DATASHEET_RMS_NOISE_MG = 0.0;
constexpr double DEFAULT_SET_RESET_REPEATABILITY_MG = 1.0;

constexpr double DEFAULT_STATIONARY_AXIS_STD_THRESHOLD_T = 5e-7;
constexpr double DEFAULT_STATIONARY_MAG_STD_THRESHOLD_T = 8e-7;
constexpr double DEFAULT_OFFSET_SPIKE_THRESHOLD_T = 0.0;
constexpr double DEFAULT_OFFSET_SPIKE_INFLATION_FACTOR = 3.0;
constexpr double DEFAULT_OFFSET_SPIKE_INFLATION_SEC = 2.0;

constexpr double DEFAULT_CALIBRATION_COOLDOWN_SEC = 1.0;
constexpr const char* DEFAULT_CALIBRATION_PATH = "~/.ros/oasis/mmc5983ma_covariance.yaml";

// Units: Gauss
// Meaning: default field range for sensitivity conversion
constexpr double kFieldRangeGauss = 8.0;

// Units: counts
// Meaning: 18-bit output range size
constexpr double kCountsFullScale = 262144.0;

// Units: Tesla per count
// Meaning: scale for converting raw counts to Tesla
constexpr double kTeslaPerCount = (2.0 * kFieldRangeGauss * 1e-4) / kCountsFullScale;
} // namespace

Mmc5983maMagnetometerNode::Mmc5983maMagnetometerNode() : rclcpp::Node(NODE_NAME)
{
  declare_parameter("i2c_device", std::string(DEFAULT_I2C_DEVICE));
  declare_parameter("i2c_address", static_cast<int>(DEFAULT_I2C_ADDRESS));
  declare_parameter("frame_id", std::string(DEFAULT_FRAME_ID));
  declare_parameter("publish_rate_hz", DEFAULT_PUBLISH_RATE_HZ);
  declare_parameter("raw_rate_hz", DEFAULT_RAW_RATE_HZ);
  declare_parameter("bandwidth_mode", static_cast<int>(DEFAULT_BANDWIDTH_MODE));
  declare_parameter("measurement_timeout_ms", DEFAULT_MEASUREMENT_TIMEOUT_MS);
  declare_parameter("stationary_window_sec", DEFAULT_STATIONARY_WINDOW_SEC);
  declare_parameter("ewma_tau_sec", DEFAULT_EWMA_TAU_SEC);
  declare_parameter("prior_strength_samples", DEFAULT_PRIOR_STRENGTH_SAMPLES);
  declare_parameter("datasheet_rms_noise_mg", DEFAULT_DATASHEET_RMS_NOISE_MG);
  declare_parameter("set_reset_repeatability_mg", DEFAULT_SET_RESET_REPEATABILITY_MG);
  declare_parameter("stationary_std_threshold_t", DEFAULT_STATIONARY_AXIS_STD_THRESHOLD_T);
  declare_parameter("stationary_mag_std_threshold_t", DEFAULT_STATIONARY_MAG_STD_THRESHOLD_T);
  declare_parameter("offset_spike_threshold_t", DEFAULT_OFFSET_SPIKE_THRESHOLD_T);
  declare_parameter("offset_spike_inflation_factor", DEFAULT_OFFSET_SPIKE_INFLATION_FACTOR);
  declare_parameter("offset_spike_inflation_sec", DEFAULT_OFFSET_SPIKE_INFLATION_SEC);
  declare_parameter("calibration_mode", false);
  declare_parameter("continuous_calibration", false);
  declare_parameter("calibration_write_cooldown_sec", DEFAULT_CALIBRATION_COOLDOWN_SEC);
  declare_parameter("calibration_yaml_path", std::string(DEFAULT_CALIBRATION_PATH));

  m_i2cDevice = get_parameter("i2c_device").as_string();
  m_i2cAddress = static_cast<std::uint8_t>(get_parameter("i2c_address").as_int());
  m_frameId = get_parameter("frame_id").as_string();

  const double publishRateHz = get_parameter("publish_rate_hz").as_double();
  const double rawRateHz = get_parameter("raw_rate_hz").as_double();

  const double clampedPublishRate = std::max(publishRateHz, 0.1);
  const double clampedRawRate = std::max(rawRateHz, clampedPublishRate);
  m_publishPeriod = std::chrono::duration<double>(1.0 / clampedPublishRate);

  if (rawRateHz < publishRateHz)
  {
    RCLCPP_WARN(get_logger(),
                "raw_rate_hz (%.2f) is below publish_rate_hz (%.2f); clamping raw rate to"
                " match publish rate",
                rawRateHz, publishRateHz);
  }

  m_rawRateHz = static_cast<std::uint16_t>(std::round(clampedRawRate));
  m_bandwidthMode = static_cast<std::uint8_t>(get_parameter("bandwidth_mode").as_int());
  m_measurementTimeoutMs = get_parameter("measurement_timeout_ms").as_int();

  const bool continuousCalibration = get_parameter("continuous_calibration").as_bool();
  m_calibrationMode = get_parameter("calibration_mode").as_bool();

  const double stationaryWindowSec = get_parameter("stationary_window_sec").as_double();
  const double ewmaTauSec = get_parameter("ewma_tau_sec").as_double();
  const double priorStrengthSamples = get_parameter("prior_strength_samples").as_double();
  const double setResetRepeatabilityMg = get_parameter("set_reset_repeatability_mg").as_double();
  const double stationaryAxisStdThresholdT =
      get_parameter("stationary_std_threshold_t").as_double();
  const double stationaryMagnitudeStdThresholdT =
      get_parameter("stationary_mag_std_threshold_t").as_double();
  const double offsetSpikeThresholdT = get_parameter("offset_spike_threshold_t").as_double();
  const double offsetSpikeInflationFactor =
      get_parameter("offset_spike_inflation_factor").as_double();
  const double offsetSpikeInflationSec = get_parameter("offset_spike_inflation_sec").as_double();

  const double calibrationCooldownSec = get_parameter("calibration_write_cooldown_sec").as_double();
  const std::string calibrationPathParam = get_parameter("calibration_yaml_path").as_string();
  std::filesystem::path calibrationPath = calibrationPathParam;

  if (!calibrationPathParam.empty() && calibrationPathParam[0] == '~')
  {
    const char* home = std::getenv("HOME");
    if (home == nullptr)
    {
      RCLCPP_WARN(get_logger(),
                  "Could not resolve HOME for calibration path %s; disabling calibration",
                  calibrationPathParam.c_str());
      calibrationPath.clear();
    }
    else
    {
      calibrationPath = std::filesystem::path(home) / calibrationPathParam.substr(1);
    }
  }

  const std::size_t windowSamples =
      static_cast<std::size_t>(std::max(2.0, stationaryWindowSec * clampedPublishRate));

  Magnetometer::MagnetometerSamplingConfig samplingConfig;
  samplingConfig.publish_period_sec = m_publishPeriod.count();

  Magnetometer::MagnetometerProcessingConfig processingConfig;
  processingConfig.stationary_window_size = windowSamples;
  processingConfig.prior_strength_samples = priorStrengthSamples;
  processingConfig.ewma_tau_sec = ewmaTauSec;
  processingConfig.stationary_axis_std_threshold_t = stationaryAxisStdThresholdT;
  processingConfig.stationary_magnitude_std_threshold_t = stationaryMagnitudeStdThresholdT;
  processingConfig.datasheet_rms_noise_mg = ResolveDatasheetNoiseMg(m_bandwidthMode);
  processingConfig.set_reset_repeatability_mg = setResetRepeatabilityMg;
  processingConfig.offset_spike_threshold_t = offsetSpikeThresholdT;
  processingConfig.offset_spike_inflation_factor = offsetSpikeInflationFactor;
  processingConfig.offset_spike_inflation_sec = offsetSpikeInflationSec;

  Magnetometer::MagnetometerCalibrationConfig calibrationConfig;
  calibrationConfig.continuous_calibration = continuousCalibration;
  calibrationConfig.write_cooldown_sec = calibrationCooldownSec;
  calibrationConfig.calibration_path = calibrationPath;
  calibrationConfig.bandwidth_mode = m_bandwidthMode;
  calibrationConfig.raw_rate_hz = m_rawRateHz;

  m_sampler = std::make_unique<Magnetometer::Mmc5983maPairSampler>(m_device);
  m_processor = std::make_unique<Magnetometer::MagnetometerPairProcessor>(
      *m_sampler, samplingConfig, processingConfig, calibrationConfig);

  if (!m_processor->Initialize())
  {
    throw std::runtime_error("Failed to initialize covariance estimator");
  }
}

bool Mmc5983maMagnetometerNode::Initialize()
{
  if (!ConfigureDevice())
    return false;

  std::uint8_t productId = 0;
  if (!m_device.ReadProductId(productId))
  {
    RCLCPP_WARN(get_logger(), "Failed to read product ID");
  }
  else
  {
    RCLCPP_INFO(get_logger(), "MMC5983MA product ID: 0x%02X", productId);
  }

  m_publisher = create_publisher<sensor_msgs::msg::MagneticField>(MAG_TOPIC, rclcpp::QoS{1});

  m_running = true;
  m_samplerThread = std::thread([this]() { SamplerLoop(); });

  m_timer = create_wall_timer(m_publishPeriod,
                              std::bind(&Mmc5983maMagnetometerNode::PublishLatest, this));

  const char* modeName = m_calibrationMode ? "Calibration" : "Driver";
  RCLCPP_INFO(get_logger(), "MMC5983MA mode: %s", modeName);

  return true;
}

void Mmc5983maMagnetometerNode::Deinitialize()
{
  m_running = false;

  if (m_samplerThread.joinable())
    m_samplerThread.join();

  m_processor.reset();
  m_sampler.reset();
}

void Mmc5983maMagnetometerNode::SamplerLoop()
{
  if (m_processor == nullptr)
    return;

  auto nextTick = std::chrono::steady_clock::now();

  while (rclcpp::ok() && m_running)
  {
    Magnetometer::MagnetometerTickOutput output;
    if (!m_processor->Tick(output))
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "%s", output.warning.c_str());
    }
    else
    {
      if (!output.warning.empty())
      {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "%s", output.warning.c_str());
      }

      if (output.covariance_updated)
        m_gateReady = true;

      SampleSnapshot snapshot;
      snapshot.stamp = get_clock()->now();
      snapshot.output = output;
      snapshot.has_sample = true;
      snapshot.gate_ready = m_gateReady;

      std::lock_guard<std::mutex> lock(m_sampleMutex);
      m_latestSample = snapshot;
    }

    nextTick += m_publishPeriod;
    std::this_thread::sleep_until(nextTick);
  }
}

void Mmc5983maMagnetometerNode::PublishLatest()
{
  SampleSnapshot snapshot;
  {
    std::lock_guard<std::mutex> lock(m_sampleMutex);
    snapshot = m_latestSample;
  }

  if (!snapshot.has_sample)
    return;

  const double ageSeconds = (get_clock()->now() - snapshot.stamp).seconds();
  if (ageSeconds > 2.0 * m_publishPeriod.count())
  {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                         "Sample age %.2f s exceeds %.2f s (sampler behind)", ageSeconds,
                         2.0 * m_publishPeriod.count());
  }

  PublishSample(snapshot.stamp, snapshot.output);

  if (snapshot.gate_ready && !snapshot.output.is_stationary)
  {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                         "Stationary gating rejected magnetometer window");
  }
}

void Mmc5983maMagnetometerNode::PublishSample(const rclcpp::Time& stamp,
                                              const Magnetometer::MagnetometerTickOutput& output)
{
  sensor_msgs::msg::MagneticField msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = m_frameId;
  msg.magnetic_field.x = output.field_t.x();
  msg.magnetic_field.y = output.field_t.y();
  msg.magnetic_field.z = output.field_t.z();

  msg.magnetic_field_covariance = {
      output.covariance_t2(0, 0), output.covariance_t2(0, 1), output.covariance_t2(0, 2),
      output.covariance_t2(1, 0), output.covariance_t2(1, 1), output.covariance_t2(1, 2),
      output.covariance_t2(2, 0), output.covariance_t2(2, 1), output.covariance_t2(2, 2)};

  if (!output.covariance_finite)
  {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                         "Non-finite covariance detected in MMC5983MA driver");
  }

  m_publisher->publish(msg);
}

bool Mmc5983maMagnetometerNode::ConfigureDevice()
{
  Magnetometer::Mmc5983maConfig config;
  config.i2c_device = m_i2cDevice;
  config.i2c_address = m_i2cAddress;
  config.raw_rate_code = EncodeRawRate(m_rawRateHz);
  config.bandwidth_mode = m_bandwidthMode;
  config.tesla_per_count = kTeslaPerCount;
  config.measurement_timeout_ms = m_measurementTimeoutMs;

  if (!m_device.Initialize(config))
  {
    RCLCPP_ERROR(get_logger(), "Failed to initialize MMC5983MA");
    return false;
  }

  return true;
}

std::uint8_t Mmc5983maMagnetometerNode::EncodeRawRate(std::uint16_t raw_rate_hz) const
{
  switch (raw_rate_hz)
  {
    case 10:
      return 0x01;
    case 20:
      return 0x02;
    case 50:
      return 0x04;
    case 100:
      return 0x05;
    case 200:
      return 0x06;
    case 1000:
      return 0x07;
    default:
      break;
  }

  RCLCPP_WARN(get_logger(), "Unsupported raw_rate_hz=%u, defaulting to 100 Hz",
              static_cast<unsigned>(raw_rate_hz));
  return 0x05;
}

double Mmc5983maMagnetometerNode::ResolveDatasheetNoiseMg(std::uint8_t bandwidth_mode) const
{
  const double configuredNoise = get_parameter("datasheet_rms_noise_mg").as_double();
  if (configuredNoise > 0.0)
    return configuredNoise;

  switch (bandwidth_mode)
  {
    case 0:
      return 0.4;
    case 1:
      return 0.6;
    case 2:
      return 0.8;
    case 3:
      return 1.2;
    default:
      break;
  }

  return 0.6;
}
