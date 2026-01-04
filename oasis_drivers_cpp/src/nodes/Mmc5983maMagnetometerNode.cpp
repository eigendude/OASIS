/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "Mmc5983maMagnetometerNode.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <optional>
#include <stdexcept>

#include <rclcpp/rclcpp.hpp>

using namespace OASIS::ROS;
using namespace std::chrono_literals;

namespace
{

constexpr const char* NODE_NAME = "mmc5983ma_magnetometer_driver";

constexpr const char* MAG_TOPIC = "mag";
constexpr const char* DEFAULT_FRAME_ID = "mag_link";

constexpr const char* DEFAULT_I2C_DEVICE = "/dev/i2c-1";
constexpr std::uint8_t DEFAULT_I2C_ADDRESS = 0x30;
constexpr double DEFAULT_PUBLISH_RATE_HZ = 50.0;
constexpr double DEFAULT_RAW_RATE_HZ = 100.0;
constexpr std::uint8_t DEFAULT_BANDWIDTH_MODE = 1;

constexpr double DEFAULT_STATIONARY_WINDOW_SEC = 2.0;
constexpr double DEFAULT_EWMA_TAU_SEC = 10.0;
constexpr double DEFAULT_PRIOR_STRENGTH_SAMPLES = 100.0;
constexpr double DEFAULT_DATASHEET_RMS_NOISE_MG = 0.0;
constexpr double DEFAULT_SET_RESET_REPEATABILITY_MG = 1.0;

constexpr double DEFAULT_STATIONARY_AXIS_STD_THRESHOLD_T = 5e-7;
constexpr double DEFAULT_STATIONARY_MAG_STD_THRESHOLD_T = 8e-7;

constexpr double DEFAULT_CALIBRATION_COOLDOWN_SEC = 1.0;
constexpr const char* DEFAULT_CALIBRATION_PATH =
    "~/.ros/oasis/mmc5983ma_covariance.yaml";

// Units: Gauss
// Meaning: default field range for sensitivity conversion
constexpr double kFieldRangeGauss = 8.0;

// Units: counts
// Meaning: 18-bit output range size
constexpr double kCountsFullScale = 262144.0;

// Units: Tesla per count
// Meaning: scale for converting raw counts to Tesla
constexpr double kTeslaPerCount =
    (2.0 * kFieldRangeGauss * 1e-4) / kCountsFullScale;
} // namespace

Mmc5983maMagnetometerNode::Mmc5983maMagnetometerNode() : rclcpp::Node(NODE_NAME)
{
  declare_parameter("i2c_device", std::string(DEFAULT_I2C_DEVICE));
  declare_parameter("i2c_address", static_cast<int>(DEFAULT_I2C_ADDRESS));
  declare_parameter("frame_id", std::string(DEFAULT_FRAME_ID));
  declare_parameter("publish_rate_hz", DEFAULT_PUBLISH_RATE_HZ);
  declare_parameter("raw_rate_hz", DEFAULT_RAW_RATE_HZ);
  declare_parameter("bandwidth_mode", static_cast<int>(DEFAULT_BANDWIDTH_MODE));
  declare_parameter("stationary_window_sec", DEFAULT_STATIONARY_WINDOW_SEC);
  declare_parameter("ewma_tau_sec", DEFAULT_EWMA_TAU_SEC);
  declare_parameter("prior_strength_samples", DEFAULT_PRIOR_STRENGTH_SAMPLES);
  declare_parameter("datasheet_rms_noise_mg", DEFAULT_DATASHEET_RMS_NOISE_MG);
  declare_parameter("set_reset_repeatability_mg", DEFAULT_SET_RESET_REPEATABILITY_MG);
  declare_parameter("stationary_std_threshold_t", DEFAULT_STATIONARY_AXIS_STD_THRESHOLD_T);
  declare_parameter("stationary_mag_std_threshold_t", DEFAULT_STATIONARY_MAG_STD_THRESHOLD_T);
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
  m_rawPeriod = std::chrono::duration<double>(1.0 / clampedRawRate);

  m_rawRateHz = static_cast<std::uint16_t>(std::round(clampedRawRate));
  m_bandwidthMode = static_cast<std::uint8_t>(get_parameter("bandwidth_mode").as_int());

  if (clampedRawRate < 2.0 * clampedPublishRate)
  {
    RCLCPP_WARN(get_logger(),
                "raw_rate_hz should be at least 2x publish_rate_hz for "
                "SET/RESET pairing");
  }

  const double stationaryWindowSec = get_parameter("stationary_window_sec").as_double();
  m_ewmaTauSec = get_parameter("ewma_tau_sec").as_double();
  m_priorStrengthSamples = get_parameter("prior_strength_samples").as_double();
  m_datasheetNoiseMg = get_parameter("datasheet_rms_noise_mg").as_double();
  m_setResetRepeatabilityMg = get_parameter("set_reset_repeatability_mg").as_double();
  m_stationaryAxisStdThresholdT = get_parameter("stationary_std_threshold_t").as_double();
  m_stationaryMagnitudeStdThresholdT =
      get_parameter("stationary_mag_std_threshold_t").as_double();

  m_calibrationMode = get_parameter("calibration_mode").as_bool();
  m_continuousCalibration = get_parameter("continuous_calibration").as_bool();
  m_calibrationCooldown =
      std::chrono::duration<double>(get_parameter("calibration_write_cooldown_sec").as_double());

  const std::string calibrationPathParam = get_parameter("calibration_yaml_path").as_string();
  if (!calibrationPathParam.empty() && calibrationPathParam.front() == '~')
  {
    const char* home = std::getenv("HOME");
    const std::filesystem::path homePath =
        home ? std::filesystem::path(home) : std::filesystem::path(".");
    m_calibrationPath = homePath / calibrationPathParam.substr(2);
  }
  else
  {
    m_calibrationPath = calibrationPathParam;
  }

  const std::size_t windowSamples =
      static_cast<std::size_t>(std::max(2.0, stationaryWindowSec * clampedPublishRate));

  Magnetometer::CovarianceEstimatorConfig covarianceConfig;
  covarianceConfig.window_size = windowSamples;
  covarianceConfig.prior_strength_samples = m_priorStrengthSamples;
  covarianceConfig.ewma_tau_sec = m_ewmaTauSec;
  covarianceConfig.dt_sec = m_publishPeriod.count();

  covarianceConfig.gate.axis_std_threshold_t = m_stationaryAxisStdThresholdT;
  covarianceConfig.gate.magnitude_std_threshold_t = m_stationaryMagnitudeStdThresholdT;

  const double datasheetNoiseMg = ResolveDatasheetNoiseMg(m_bandwidthMode);
  const double noiseTesla = datasheetNoiseMg * 1e-7;
  const double repeatTesla = m_setResetRepeatabilityMg * 1e-7;

  // Units: Tesla
  // Meaning: raw noise standard deviation combining datasheet and set/reset
  const double sigmaRawT = std::sqrt((noiseTesla * noiseTesla) + (repeatTesla * repeatTesla));

  // Units: Tesla
  // Meaning: output noise after pairing (S - R) / 2
  const double sigmaOutT = sigmaRawT / std::sqrt(2.0);

  covarianceConfig.prior_covariance =
      Eigen::Matrix3d::Identity() * (sigmaOutT * sigmaOutT);

  if (!m_covarianceEstimator.Initialize(covarianceConfig))
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
  m_timer = create_wall_timer(m_rawPeriod, std::bind(&Mmc5983maMagnetometerNode::PollSensor, this));

  const char* modeName = m_calibrationMode ? "Calibration" : "Driver";
  RCLCPP_INFO(get_logger(), "MMC5983MA mode: %s", modeName);

  return true;
}

void Mmc5983maMagnetometerNode::Deinitialize()
{
  m_covarianceEstimator.Reset();
  m_hasSetSample = false;
  m_nextIsReset = false;
}

void Mmc5983maMagnetometerNode::PollSensor()
{
  const Magnetometer::MeasurementMode mode =
      m_nextIsReset ? Magnetometer::MeasurementMode::Reset : Magnetometer::MeasurementMode::Set;

  Eigen::Vector3d sample = Eigen::Vector3d::Zero();
  if (!m_device.TakeMeasurement(mode, sample))
  {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                         "Failed to read MMC5983MA measurement");
    return;
  }

  if (!m_nextIsReset)
  {
    m_lastSetSample = sample;
    m_hasSetSample = true;
    m_nextIsReset = true;
    return;
  }

  if (!m_hasSetSample)
    return;

  const Eigen::Vector3d field_t = (m_lastSetSample - sample) * 0.5;
  const Eigen::Vector3d offset_t = (m_lastSetSample + sample) * 0.5;

  PublishSample(field_t, offset_t);

  m_hasSetSample = false;
  m_nextIsReset = false;
}

void Mmc5983maMagnetometerNode::PublishSample(const Eigen::Vector3d& field_t,
                                              const Eigen::Vector3d& offset_t)
{
  const rclcpp::Time now = get_clock()->now();
  m_lastSampleTime = now;
  m_hasLastSampleTime = true;

  sensor_msgs::msg::MagneticField msg;
  msg.header.stamp = now;
  msg.header.frame_id = m_frameId;
  msg.magnetic_field.x = field_t.x();
  msg.magnetic_field.y = field_t.y();
  msg.magnetic_field.z = field_t.z();

  const Magnetometer::CovarianceUpdate update = m_covarianceEstimator.Update(field_t);
  const Eigen::Matrix3d covariance = update.covariance_t2;

  if (!covariance.allFinite())
  {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                         "Non-finite covariance detected in MMC5983MA driver");
  }

  msg.magnetic_field_covariance = {
      covariance(0, 0), covariance(0, 1), covariance(0, 2), covariance(1, 0), covariance(1, 1),
      covariance(1, 2), covariance(2, 0), covariance(2, 1), covariance(2, 2)};

  m_publisher->publish(msg);

  m_lastOffset = offset_t;

  if (!update.is_stationary)
  {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                         "Stationary gating rejected magnetometer window");
  }

  if (update.covariance_updated && m_continuousCalibration)
    (void)MaybeWriteCalibration(offset_t);
}

bool Mmc5983maMagnetometerNode::ConfigureDevice()
{
  Magnetometer::Mmc5983maConfig config;
  config.i2c_device = m_i2cDevice;
  config.i2c_address = m_i2cAddress;
  config.raw_rate_code = EncodeRawRate(m_rawRateHz);
  config.bandwidth_mode = m_bandwidthMode;
  config.tesla_per_count = kTeslaPerCount;
  config.measurement_timeout_ms = 20;

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
      return 0x03;
    case 100:
      return 0x04;
    case 200:
      return 0x05;
    case 1000:
      return 0x06;
    default:
      break;
  }

  RCLCPP_WARN(get_logger(), "Unsupported raw_rate_hz=%u, defaulting to 100 Hz",
              static_cast<unsigned>(raw_rate_hz));
  return 0x04;
}

double Mmc5983maMagnetometerNode::ResolveDatasheetNoiseMg(std::uint8_t bandwidth_mode) const
{
  if (m_datasheetNoiseMg > 0.0)
    return m_datasheetNoiseMg;

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

bool Mmc5983maMagnetometerNode::MaybeWriteCalibration(const Eigen::Vector3d& offset_t)
{
  if (m_calibrationPath.empty())
    return false;

  const rclcpp::Time now = get_clock()->now();
  if (m_hasLastCalibrationWrite)
  {
    const double elapsed = (now - m_lastCalibrationWriteTime).seconds();
    if (elapsed < m_calibrationCooldown.count())
      return false;
  }

  Magnetometer::MagnetometerCalibrationRecord record;
  record.timestamp_s = now.seconds();
  record.bandwidth_mode = m_bandwidthMode;
  record.raw_rate_hz = m_rawRateHz;
  record.covariance_t2 = m_covarianceEstimator.GetCovariance();
  record.offset_t = offset_t;

  std::error_code error;
  std::filesystem::create_directories(m_calibrationPath.parent_path(), error);
  if (error)
  {
    RCLCPP_WARN(get_logger(), "Failed to create calibration directory: %s",
                error.message().c_str());
  }

  const bool wrote = m_calibrationFile.Write(m_calibrationPath, record);
  if (!wrote)
  {
    RCLCPP_WARN(get_logger(), "Failed to write magnetometer calibration to %s",
                m_calibrationPath.c_str());
    return false;
  }

  m_lastCalibrationWriteTime = now;
  m_hasLastCalibrationWrite = true;

  return true;
}
