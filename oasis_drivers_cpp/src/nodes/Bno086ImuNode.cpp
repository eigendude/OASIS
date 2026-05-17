/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "Bno086ImuNode.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <oasis_msgs/msg/imu_vr.hpp>
#include <rclcpp/rclcpp.hpp>

using namespace OASIS::ROS;
using namespace OASIS::IMU::BNO086;

namespace
{
constexpr const char* NODE_NAME = "bno086_imu_driver";

constexpr const char* DEFAULT_I2C_DEVICE = "/dev/i2c-1";
constexpr std::uint8_t DEFAULT_I2C_ADDRESS = 0x4B;
constexpr int DEFAULT_INT_GPIO = 23;

constexpr double DEFAULT_REPORT_RATE_HZ = 100.0;

constexpr const char* IMU_TOPIC = "imu";
constexpr const char* IMU_PREDICTED_TOPIC = "imu_predicted";
constexpr const char* IMU_VR_TOPIC = "imu_vr";
constexpr const char* GRAVITY_TOPIC = "gravity";
constexpr const char* IMU_GRAVITY_TOPIC = "imu_gravity";
constexpr const char* DEFAULT_FRAME_ID = "imu_link";

constexpr int INTERRUPT_WAIT_TIMEOUT_MS = 20;
constexpr int DEFAULT_PACKET_READ_TIMEOUT_MS = 5;
constexpr int MIN_PACKET_READ_TIMEOUT_MS = 1;
constexpr int MAX_PACKET_READ_TIMEOUT_MS = 100;
constexpr int DEFAULT_FEATURE_RESPONSE_STARTUP_DRAIN_MS = 250;
constexpr int DEFAULT_FEATURE_RESPONSE_STARTUP_MAX_PACKETS = 128;
constexpr int DEFAULT_MAX_PACKETS_PER_INTERRUPT = 1024;
constexpr int MIN_MAX_PACKETS_PER_INTERRUPT = 1;
constexpr int MAX_MAX_PACKETS_PER_INTERRUPT = 8192;
constexpr int DEFAULT_MAX_POLL_ITERATIONS_PER_INTERRUPT = 4096;
constexpr int MIN_MAX_POLL_ITERATIONS_PER_INTERRUPT = 1;
constexpr int MAX_MAX_POLL_ITERATIONS_PER_INTERRUPT = 16384;
constexpr std::uint32_t REPEATED_NO_PROGRESS_TIMEOUT_WARN_COUNT = 3;

constexpr std::uint32_t MIN_COHERENT_SAMPLE_SPAN_US = 3'000;
constexpr std::uint32_t MAX_COHERENT_SAMPLE_SPAN_US = 20'000;
constexpr double DEFAULT_PREDICTION_HORIZON_SEC = 0.0;

constexpr double DEFAULT_ROTATION_VECTOR_RATE_HZ = 100.0;
constexpr double DEFAULT_GYRO_RATE_HZ = 100.0;
constexpr double DEFAULT_ACCELEROMETER_RATE_HZ = 100.0;
constexpr double DEFAULT_LINEAR_ACCELERATION_RATE_HZ = 50.0;
constexpr double DEFAULT_GRAVITY_RATE_HZ = 25.0;
constexpr double DEFAULT_ROTATION_VECTOR_BATCH_MS = 20.0;
constexpr double DEFAULT_GYRO_BATCH_MS = 20.0;
constexpr double DEFAULT_ACCELEROMETER_BATCH_MS = 20.0;
constexpr double DEFAULT_LINEAR_ACCELERATION_BATCH_MS = 20.0;
constexpr double DEFAULT_GRAVITY_BATCH_MS = 40.0;
constexpr bool DEFAULT_ENABLE_LINEAR_ACCELERATION_REPORT = true;
constexpr bool DEFAULT_ENABLE_GRAVITY_REPORT = true;
constexpr int DEFAULT_FEATURE_SUMMARY_TIMEOUT_MS = 5'000;

// Maximum nearby orientation age accepted for imu_gravity composition
constexpr double DEFAULT_IMU_GRAVITY_MAX_ORIENTATION_AGE_MS = 25.0;

// Maximum nearby gyro age accepted for imu_gravity composition
constexpr double DEFAULT_IMU_GRAVITY_MAX_GYRO_AGE_MS = 25.0;

// Plausibility bound for gravity-included calibrated acceleration samples
constexpr double MAX_IMU_GRAVITY_ACCEL_MAGNITUDE_MPS2 = 200.0;

// Plausibility bound for calibrated gyro samples
constexpr double MAX_IMU_GRAVITY_GYRO_MAGNITUDE_RADS = 100.0;
constexpr double PI_RAD = 3.14159265358979323846;
constexpr const char* ORIENTATION_REPORT_SOURCE = "rotation_vector";
constexpr int ORIENTATION_COVARIANCE_LOG_THROTTLE_MS = 5'000;
constexpr int IMU_GRAVITY_SAMPLE_WARN_THROTTLE_MS = 5'000;
constexpr const char* DEFAULT_BNO086_DIAGNOSTICS_LOG_LEVEL = "debug";
constexpr int DEFAULT_BNO086_DIAGNOSTICS_LOG_PERIOD_MS = 5'000;
constexpr int MIN_BNO086_DIAGNOSTICS_LOG_PERIOD_MS = 1'000;
constexpr double MIN_HEALTHY_RATE_FRACTION = 0.5;

bool IsFiniteArray(const std::array<double, 9>& values)
{
  return std::all_of(values.begin(), values.end(),
                     [](double value) { return std::isfinite(value); });
}

double VectorMagnitude(double x, double y, double z)
{
  return std::sqrt(x * x + y * y + z * z);
}

std::uint32_t MillisecondsToMicroseconds(double milliseconds)
{
  const double clampedMs = std::max(milliseconds, 0.0);
  return static_cast<std::uint32_t>(std::round(clampedMs * 1000.0));
}

std::optional<std::uint32_t> RateHzToIntervalUs(double rate_hz)
{
  if (!(rate_hz > 0.0))
    return std::nullopt;

  return static_cast<std::uint32_t>(std::max(1.0, std::round(1'000'000.0 / rate_hz)));
}
} // namespace

Bno086ImuNode::Bno086ImuNode() : rclcpp::Node(NODE_NAME)
{
  declare_parameter("i2c_device", std::string(DEFAULT_I2C_DEVICE));
  declare_parameter("i2c_address", static_cast<int>(DEFAULT_I2C_ADDRESS));
  declare_parameter("int_gpio", DEFAULT_INT_GPIO);
  declare_parameter("report_rate_hz", DEFAULT_REPORT_RATE_HZ);
  declare_parameter("bno086_rotation_vector_rate_hz", DEFAULT_ROTATION_VECTOR_RATE_HZ);
  declare_parameter("bno086_gyro_rate_hz", DEFAULT_GYRO_RATE_HZ);
  declare_parameter("bno086_accelerometer_rate_hz", DEFAULT_ACCELEROMETER_RATE_HZ);
  declare_parameter("bno086_linear_acceleration_rate_hz", DEFAULT_LINEAR_ACCELERATION_RATE_HZ);
  declare_parameter("bno086_gravity_rate_hz", DEFAULT_GRAVITY_RATE_HZ);
  declare_parameter("bno086_rotation_vector_batch_ms", DEFAULT_ROTATION_VECTOR_BATCH_MS);
  declare_parameter("bno086_gyro_batch_ms", DEFAULT_GYRO_BATCH_MS);
  declare_parameter("bno086_accelerometer_batch_ms", DEFAULT_ACCELEROMETER_BATCH_MS);
  declare_parameter("bno086_linear_acceleration_batch_ms", DEFAULT_LINEAR_ACCELERATION_BATCH_MS);
  declare_parameter("bno086_gravity_batch_ms", DEFAULT_GRAVITY_BATCH_MS);
  declare_parameter("bno086_enable_linear_acceleration_report",
                    DEFAULT_ENABLE_LINEAR_ACCELERATION_REPORT);
  declare_parameter("bno086_enable_gravity_report", DEFAULT_ENABLE_GRAVITY_REPORT);
  declare_parameter("bno086_feature_summary_timeout_ms", DEFAULT_FEATURE_SUMMARY_TIMEOUT_MS);
  declare_parameter("bno086_feature_response_startup_drain_ms",
                    DEFAULT_FEATURE_RESPONSE_STARTUP_DRAIN_MS);
  declare_parameter("bno086_feature_response_startup_max_packets",
                    DEFAULT_FEATURE_RESPONSE_STARTUP_MAX_PACKETS);
  declare_parameter("bno086_packet_read_timeout_ms", DEFAULT_PACKET_READ_TIMEOUT_MS);
  declare_parameter("bno086_max_packets_per_interrupt", DEFAULT_MAX_PACKETS_PER_INTERRUPT);
  declare_parameter("bno086_max_poll_iterations_per_interrupt",
                    DEFAULT_MAX_POLL_ITERATIONS_PER_INTERRUPT);
  declare_parameter("bno086_diagnostics_log_level",
                    std::string(DEFAULT_BNO086_DIAGNOSTICS_LOG_LEVEL));
  declare_parameter("bno086_diagnostics_log_period_ms", DEFAULT_BNO086_DIAGNOSTICS_LOG_PERIOD_MS);
  declare_parameter("prediction_horizon_sec", DEFAULT_PREDICTION_HORIZON_SEC);
  declare_parameter("imu_gravity_max_orientation_age_ms",
                    DEFAULT_IMU_GRAVITY_MAX_ORIENTATION_AGE_MS);
  declare_parameter("imu_gravity_max_gyro_age_ms", DEFAULT_IMU_GRAVITY_MAX_GYRO_AGE_MS);

  declare_parameter("frame_id", std::string(DEFAULT_FRAME_ID));

  const std::string i2cDevice = get_parameter("i2c_device").as_string();
  const std::uint8_t i2cAddress = static_cast<std::uint8_t>(get_parameter("i2c_address").as_int());
  const int intGpio = get_parameter("int_gpio").as_int();
  const double reportRateHz = std::max(get_parameter("report_rate_hz").as_double(), 1.0);
  m_rotationVectorRateHz =
      std::max(get_parameter("bno086_rotation_vector_rate_hz").as_double(), 1.0);
  m_gyroRateHz = std::max(get_parameter("bno086_gyro_rate_hz").as_double(), 1.0);
  m_accelerometerRateHz = std::max(get_parameter("bno086_accelerometer_rate_hz").as_double(), 1.0);
  m_linearAccelerationRateHz =
      std::max(get_parameter("bno086_linear_acceleration_rate_hz").as_double(), 1.0);
  m_gravityRateHz = std::max(get_parameter("bno086_gravity_rate_hz").as_double(), 1.0);
  m_rotationVectorBatchIntervalUs =
      MillisecondsToMicroseconds(get_parameter("bno086_rotation_vector_batch_ms").as_double());
  m_gyroBatchIntervalUs =
      MillisecondsToMicroseconds(get_parameter("bno086_gyro_batch_ms").as_double());
  m_accelerometerBatchIntervalUs =
      MillisecondsToMicroseconds(get_parameter("bno086_accelerometer_batch_ms").as_double());
  m_linearAccelerationBatchIntervalUs =
      MillisecondsToMicroseconds(get_parameter("bno086_linear_acceleration_batch_ms").as_double());
  m_gravityBatchIntervalUs =
      MillisecondsToMicroseconds(get_parameter("bno086_gravity_batch_ms").as_double());
  m_enableLinearAccelerationReport =
      get_parameter("bno086_enable_linear_acceleration_report").as_bool();
  m_enableGravityReport = get_parameter("bno086_enable_gravity_report").as_bool();
  m_featureSummaryTimeoutMs =
      std::max(static_cast<int>(get_parameter("bno086_feature_summary_timeout_ms").as_int()), 0);
  m_featureResponseStartupDrainMs = std::max(
      static_cast<int>(get_parameter("bno086_feature_response_startup_drain_ms").as_int()), 0);
  m_featureResponseStartupMaxPackets = static_cast<std::uint32_t>(std::max(
      static_cast<int>(get_parameter("bno086_feature_response_startup_max_packets").as_int()), 0));
  m_packetReadTimeoutMs =
      std::clamp(static_cast<int>(get_parameter("bno086_packet_read_timeout_ms").as_int()),
                 MIN_PACKET_READ_TIMEOUT_MS, MAX_PACKET_READ_TIMEOUT_MS);
  m_maxPacketsPerInterrupt = static_cast<std::uint32_t>(
      std::clamp(static_cast<int>(get_parameter("bno086_max_packets_per_interrupt").as_int()),
                 MIN_MAX_PACKETS_PER_INTERRUPT, MAX_MAX_PACKETS_PER_INTERRUPT));
  m_maxPollIterationsPerInterrupt = static_cast<std::uint32_t>(std::clamp(
      static_cast<int>(get_parameter("bno086_max_poll_iterations_per_interrupt").as_int()),
      MIN_MAX_POLL_ITERATIONS_PER_INTERRUPT, MAX_MAX_POLL_ITERATIONS_PER_INTERRUPT));
  const std::string diagnosticsLogLevel = get_parameter("bno086_diagnostics_log_level").as_string();
  const std::optional<Bno086DiagnosticsLogLevel> parsedDiagnosticsLogLevel =
      ParseBno086DiagnosticsLogLevel(diagnosticsLogLevel);
  if (parsedDiagnosticsLogLevel.has_value())
    m_diagnosticsLogLevel = *parsedDiagnosticsLogLevel;
  else
    m_diagnosticsLogLevel = Bno086DiagnosticsLogLevel::Debug;

  m_diagnosticsLogPeriodMs =
      std::max(static_cast<int>(get_parameter("bno086_diagnostics_log_period_ms").as_int()),
               MIN_BNO086_DIAGNOSTICS_LOG_PERIOD_MS);
  m_predictionHorizonSec = std::max(get_parameter("prediction_horizon_sec").as_double(), 0.0);
  m_imuGravityMaxOrientationAgeMs =
      std::max(get_parameter("imu_gravity_max_orientation_age_ms").as_double(), 0.0);
  m_imuGravityMaxGyroAgeMs =
      std::max(get_parameter("imu_gravity_max_gyro_age_ms").as_double(), 0.0);
  m_reportIntervalUs = std::max<std::uint32_t>(
      1U, static_cast<std::uint32_t>(std::round(1'000'000.0 / reportRateHz)));

  m_frameId = get_parameter("frame_id").as_string();

  Bno086TransportConfig transportConfig;
  transportConfig.i2c_device = i2cDevice;
  transportConfig.i2c_address = i2cAddress;

  if (!m_transport.Open(transportConfig))
  {
    RCLCPP_ERROR(get_logger(), "Failed to open BNO086 on %s (0x%02X)", i2cDevice.c_str(),
                 static_cast<unsigned>(i2cAddress));
    throw std::runtime_error("Failed to open BNO086 transport");
  }

  Bno086GpioConfig gpioConfig;
  gpioConfig.line_offset = static_cast<unsigned>(std::max(intGpio, 0));

  if (!m_interruptGpio.Open(gpioConfig))
  {
    RCLCPP_ERROR(get_logger(), "Failed to open GPIO interrupt line %d on %s", intGpio,
                 gpioConfig.chip_device.c_str());
    throw std::runtime_error("Failed to open BNO086 GPIO interrupt");
  }

  m_shtp = std::make_unique<Bno086Shtp>(m_transport);

  Bno086ShtpConfig shtpConfig;
  shtpConfig.report_rate_hz = reportRateHz;
  shtpConfig.rotation_vector_rate_hz = m_rotationVectorRateHz;
  shtpConfig.gyro_rate_hz = m_gyroRateHz;
  shtpConfig.accelerometer_rate_hz = m_accelerometerRateHz;
  shtpConfig.linear_acceleration_rate_hz = m_linearAccelerationRateHz;
  shtpConfig.gravity_rate_hz = m_gravityRateHz;
  shtpConfig.rotation_vector_batch_interval_us = m_rotationVectorBatchIntervalUs;
  shtpConfig.gyro_batch_interval_us = m_gyroBatchIntervalUs;
  shtpConfig.accelerometer_batch_interval_us = m_accelerometerBatchIntervalUs;
  shtpConfig.linear_acceleration_batch_interval_us = m_linearAccelerationBatchIntervalUs;
  shtpConfig.gravity_batch_interval_us = m_gravityBatchIntervalUs;
  shtpConfig.enable_linear_acceleration_report = m_enableLinearAccelerationReport;
  shtpConfig.enable_gravity_report = m_enableGravityReport;

  if (!m_shtp->Configure(shtpConfig))
  {
    RCLCPP_ERROR(get_logger(), "Failed to send initial BNO086 Set Feature commands");
    throw std::runtime_error("Failed to configure BNO086 reports");
  }
  m_featureConfigurationStartedAt = std::chrono::steady_clock::now();
  const Bno086Shtp::FeatureResponseDrainResult featureDrainResult = m_shtp->DrainFeatureResponses(
      m_featureResponseStartupDrainMs, m_featureResponseStartupMaxPackets, m_packetReadTimeoutMs);
  RCLCPP_INFO(get_logger(),
              "BNO086 feature response startup drain: received=%zu expected=%zu "
              "pre_report_packets=%u sensor_events_ignored_or_seen=%u physical_packets=%u "
              "elapsed_ms=%u",
              featureDrainResult.received_responses, featureDrainResult.expected_responses,
              featureDrainResult.pre_report_packets, featureDrainResult.sensor_events_seen,
              featureDrainResult.physical_packets, featureDrainResult.elapsed_ms);
  MaybeLogFeatureResponses();

  if (intGpio == DEFAULT_INT_GPIO)
  {
    RCLCPP_INFO(get_logger(), "BNO086 INT uses GPIO%d (Raspberry Pi header pin 16), active low",
                intGpio);
  }

  RCLCPP_INFO(get_logger(),
              "BNO086 opened on %s (0x%02X), int_gpio=%d active_low, "
              "fallback_report_rate_hz=%.1f",
              i2cDevice.c_str(), static_cast<unsigned>(i2cAddress), intGpio, reportRateHz);
  RCLCPP_INFO(get_logger(),
              "BNO086 static report rates:\n"
              "  rotation_vector=%.1f batch_ms=%.0f enabled=true\n"
              "  gyro=%.1f batch_ms=%.0f enabled=true\n"
              "  accelerometer=%.1f batch_ms=%.0f enabled=true\n"
              "  linear_acceleration=%.1f batch_ms=%.0f enabled=%s\n"
              "  gravity=%.1f batch_ms=%.0f enabled=%s",
              m_rotationVectorRateHz, static_cast<double>(m_rotationVectorBatchIntervalUs) / 1000.0,
              m_gyroRateHz, static_cast<double>(m_gyroBatchIntervalUs) / 1000.0,
              m_accelerometerRateHz, static_cast<double>(m_accelerometerBatchIntervalUs) / 1000.0,
              m_linearAccelerationRateHz,
              m_enableLinearAccelerationReport
                  ? static_cast<double>(m_linearAccelerationBatchIntervalUs) / 1000.0
                  : 0.0,
              m_enableLinearAccelerationReport ? "true" : "false", m_gravityRateHz,
              m_enableGravityReport ? static_cast<double>(m_gravityBatchIntervalUs) / 1000.0 : 0.0,
              m_enableGravityReport ? "true" : "false");
  RCLCPP_INFO(get_logger(), "ROS publication is interrupt-driven from GPIO packet drains");
  RCLCPP_INFO(get_logger(), "BNO086 diagnostics log level=%s period_ms=%d",
              parsedDiagnosticsLogLevel.has_value() ? diagnosticsLogLevel.c_str() : "debug",
              m_diagnosticsLogPeriodMs);
  RCLCPP_INFO(get_logger(), "Predicted orientation output uses %s with prediction_horizon_sec=%.4f",
              m_predictionSource.c_str(), m_predictionHorizonSec);
  RCLCPP_INFO(get_logger(),
              "BNO086 imu_gravity uses calibrated acceleration cadence with "
              "orientation_max_age_ms=%.1f gyro_max_age_ms=%.1f",
              m_imuGravityMaxOrientationAgeMs, m_imuGravityMaxGyroAgeMs);
}

bool Bno086ImuNode::Initialize()
{
  m_imuPublisher = create_publisher<sensor_msgs::msg::Imu>(IMU_TOPIC, rclcpp::SensorDataQoS{});
  m_imuPredictedPublisher =
      create_publisher<sensor_msgs::msg::Imu>(IMU_PREDICTED_TOPIC, rclcpp::SensorDataQoS{});
  m_imuVrPublisher =
      create_publisher<oasis_msgs::msg::ImuVr>(IMU_VR_TOPIC, rclcpp::SensorDataQoS{});
  m_gravityPublisher = create_publisher<geometry_msgs::msg::AccelWithCovarianceStamped>(
      GRAVITY_TOPIC, rclcpp::SensorDataQoS{});
  m_imuGravityPublisher =
      create_publisher<sensor_msgs::msg::Imu>(IMU_GRAVITY_TOPIC, rclcpp::SensorDataQoS{});

  m_running.store(true);
  m_interruptThread = std::thread(&Bno086ImuNode::InterruptLoop, this);

  return true;
}

void Bno086ImuNode::Deinitialize()
{
  m_running.store(false);

  if (m_interruptThread.joinable())
    m_interruptThread.join();

  m_shtp.reset();
  m_interruptGpio.Close();
  m_transport.Close();
}

void Bno086ImuNode::InterruptLoop()
{
  while (m_running.load() && rclcpp::ok())
  {
    if (m_shtp == nullptr)
      return;

    std::chrono::steady_clock::time_point interruptSteadyAt;
    const Bno086Gpio::WaitResult waitResult =
        m_interruptGpio.WaitForAssertedLow(INTERRUPT_WAIT_TIMEOUT_MS, interruptSteadyAt);

    if (waitResult == Bno086Gpio::WaitResult::Timeout)
      continue;

    if (waitResult == Bno086Gpio::WaitResult::Error)
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "GPIO wait error while waiting for BNO086 interrupt");
      continue;
    }

    const auto nowSteady = std::chrono::steady_clock::now();
    const rclcpp::Time nowRos = get_clock()->now();
    const auto interruptDeltaNs =
        std::chrono::duration_cast<std::chrono::nanoseconds>(nowSteady - interruptSteadyAt).count();
    const rclcpp::Time interruptRosAt = nowRos - rclcpp::Duration(0, interruptDeltaNs);

    DrainPacketsForInterrupt(interruptSteadyAt, interruptRosAt);

    const Bno086Shtp::StartupStatus& startup = m_shtp->GetStartupStatus();

    if (startup.communication_established && !m_loggedCommEstablished)
    {
      RCLCPP_INFO(get_logger(), "BNO086 communication established");
      m_loggedCommEstablished = true;
    }

    if (startup.set_feature_sent && !m_loggedSetFeature)
    {
      RCLCPP_INFO(get_logger(), "BNO086 Set Feature commands sent");
      m_loggedSetFeature = true;
    }

    MaybeLogFeatureResponses();
  }
}

void Bno086ImuNode::DrainPacketsForInterrupt(
    const std::chrono::steady_clock::time_point& /*interrupt_steady_at*/,
    const rclcpp::Time& interrupt_ros_at)
{
  Bno086DrainCounters drainCounters;
  Bno086DrainLimits drainLimits;
  drainLimits.max_physical_packets_per_interrupt = m_maxPacketsPerInterrupt;
  drainLimits.max_poll_iterations_per_interrupt = m_maxPollIterationsPerInterrupt;
  const rclcpp::Time drainReceiveAnchor = interrupt_ros_at;

  while (m_running.load())
  {
    const bool hintnAssertedBeforePoll = m_interruptGpio.IsAssertedLow();
    const Bno086DrainDecision beforePollDecision =
        Bno086DrainBeforePoll(drainLimits, drainCounters, hintnAssertedBeforePoll);
    if (beforePollDecision.action == Bno086DrainAction::PollIterationCap)
    {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "BNO086 interrupt drain hit poll iteration cap: "
          "physical_packets_this_drain=%u sensor_events_this_drain=%u "
          "pending_events_this_drain=%u control_packets_this_drain=%u "
          "poll_iterations=%u poll_iteration_cap=%u hintn_asserted=%s",
          drainCounters.physical_packets_this_drain, drainCounters.sensor_events_this_drain,
          drainCounters.pending_events_this_drain, drainCounters.control_packets_this_drain,
          drainCounters.poll_iterations, drainLimits.max_poll_iterations_per_interrupt,
          beforePollDecision.hintn_asserted ? "true" : "false");
      break;
    }

    const Bno086Shtp::PollResult pollResult = m_shtp->Poll(m_packetReadTimeoutMs);
    const bool hintnAssertedAfterPoll = m_interruptGpio.IsAssertedLow();
    const Bno086DrainDecision afterPollDecision =
        Bno086DrainAfterPoll(pollResult, drainLimits, drainCounters, hintnAssertedAfterPoll);

    if (afterPollDecision.action == Bno086DrainAction::Complete)
    {
      m_repeatedNoProgressTimeouts = 0;
      break;
    }

    if (afterPollDecision.action == Bno086DrainAction::WarnNoProgressTimeout)
    {
      ++m_repeatedNoProgressTimeouts;
      if (m_repeatedNoProgressTimeouts >= REPEATED_NO_PROGRESS_TIMEOUT_WARN_COUNT)
      {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                             "BNO086 interrupt asserted but packet reads repeatedly timed out");
      }
      break;
    }

    m_repeatedNoProgressTimeouts = 0;

    if (afterPollDecision.action == Bno086DrainAction::TransportError)
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "BNO086 transport error while draining interrupt data");
      break;
    }

    MaybeLogFeatureResponses();

    if (pollResult.status == Bno086Shtp::PollStatus::SensorEvent && pollResult.event.has_value())
    {
      // For batched SHTP reports, host drain time is transport latency. Use
      // one stable receive anchor for the interrupt drain so reports in a
      // batch are not timestamped later merely because they were read later
      // over I2C.
      const rclcpp::Time eventStamp = EstimateEventStamp(*pollResult.event, drainReceiveAnchor);
      const auto normalizerIndex = static_cast<std::size_t>(pollResult.event->report_id);
      const std::optional<std::uint32_t> expectedIntervalUs =
          EffectiveReportIntervalUs(pollResult.event->report_id);
      const std::optional<int64_t> expectedIntervalNs =
          expectedIntervalUs.has_value()
              ? std::make_optional(static_cast<int64_t>(*expectedIntervalUs) * 1'000)
              : std::nullopt;

      // Units: ns. Allows mapped batched samples to land just ahead of the
      // stable host drain anchor; chosen above the 40 ms max batch interval.
      constexpr int64_t kMappedTimestampFutureSlopNs = 50'000'000;

      const TimestampNormalizationResult normalizedStamp =
          m_timestampNormalizers[normalizerIndex].Normalize(
              TimestampSample{pollResult.event->sequence, eventStamp.nanoseconds(),
                              drainReceiveAnchor.nanoseconds(),
                              pollResult.event->has_base_timestamp
                                  ? std::make_optional(kMappedTimestampFutureSlopNs)
                                  : std::nullopt},
              expectedIntervalNs);

      if (normalizedStamp.reconstruction_reset)
        ++m_imuGravityDiagnostics.timestamp_reconstruction_reset_count;

      const bool repairedToInterval = normalizedStamp.repaired_duplicate_to_interval ||
                                      normalizedStamp.repaired_nonmonotonic_to_interval ||
                                      normalizedStamp.repaired_sequence_gap_to_interval;
      if (repairedToInterval && expectedIntervalUs.has_value())
      {
        m_imuGravityDiagnostics.latest_timestamp_repair_interval_us = *expectedIntervalUs;

        if (normalizedStamp.repaired_duplicate_to_interval)
          ++m_imuGravityDiagnostics.timestamp_repaired_duplicate_to_interval;

        if (normalizedStamp.repaired_nonmonotonic_to_interval)
          ++m_imuGravityDiagnostics.timestamp_repaired_nonmonotonic_to_interval;

        if (normalizedStamp.repaired_sequence_gap_to_interval)
          ++m_imuGravityDiagnostics.timestamp_repaired_sequence_gap_to_interval;

        if (normalizedStamp.interval_repair_clamped_to_host)
          ++m_imuGravityDiagnostics.timestamp_interval_repair_clamped_to_host;

        if (normalizedStamp.interval_repair_bounded_to_legacy)
          ++m_imuGravityDiagnostics.timestamp_interval_repair_bounded_to_legacy;

        if (!m_loggedTimestampIntervalRepair)
        {
          const char* repairReason =
              normalizedStamp.repaired_sequence_gap_to_interval ? "sequence_gap"
              : normalizedStamp.repaired_duplicate_to_interval  ? "duplicate_stamp"
                                                                : "nonmonotonic_stamp";
          RCLCPP_DEBUG(get_logger(),
                       "BNO086 timestamp repair using report interval: report=%s "
                       "expected_interval_us=%u reason=%s",
                       ReportName(pollResult.event->report_id), *expectedIntervalUs, repairReason);
          m_loggedTimestampIntervalRepair = true;
        }
      }

      if (pollResult.event->report_id == ReportId::Accelerometer)
      {
        if (normalizedStamp.repaired_nonmonotonic)
          ++m_imuGravityDiagnostics.timestamp_repaired_nonmonotonic_accel;

        if (normalizedStamp.sequence_gap)
          ++m_imuGravityDiagnostics.accel_sequence_gap_count;
      }

      if (!normalizedStamp.duplicate)
      {
        const rclcpp::Time normalizedEventStamp(normalizedStamp.stamp_ns, RCL_ROS_TIME);
        CountDecodedReport(*pollResult.event);
        ApplyEvent(*pollResult.event, normalizedEventStamp);
        MaybePublishOnLinearAcceleration(*pollResult.event);
        MaybePublishImuGravityOnAccelerometer(*pollResult.event);
        MaybePublishGravityOnGravityReport(*pollResult.event);
      }
    }

    if (afterPollDecision.action == Bno086DrainAction::PhysicalPacketCap)
    {
      if (afterPollDecision.hintn_asserted)
      {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "BNO086 interrupt drain hit physical packet cap while H_INTN "
            "remained asserted: physical_packets_this_drain=%u "
            "sensor_events_this_drain=%u pending_events_this_drain=%u "
            "control_packets_this_drain=%u poll_iterations=%u packet_cap=%u "
            "hintn_asserted=true",
            drainCounters.physical_packets_this_drain, drainCounters.sensor_events_this_drain,
            drainCounters.pending_events_this_drain, drainCounters.control_packets_this_drain,
            drainCounters.poll_iterations, drainLimits.max_physical_packets_per_interrupt);
      }
      break;
    }
  }
}

void Bno086ImuNode::MaybePublishOnLinearAcceleration(const SensorEvent& event)
{
  // LinearAcceleration is the cadence anchor for IMU publication once the
  // orientation + gyro + linear-accel trio is coherent
  if (event.report_id != ReportId::LinearAcceleration)
    return;

  ++m_imuGravityDiagnostics.linear_accel_events_seen;
  m_imuGravityDiagnostics.latest_orientation_stamp_ns =
      m_orientationState.has_sample ? m_orientationState.stamp.nanoseconds() : 0;
  m_imuGravityDiagnostics.latest_gyro_stamp_ns =
      m_gyroState.has_sample ? m_gyroState.stamp.nanoseconds() : 0;
  m_imuGravityDiagnostics.latest_linear_accel_stamp_ns =
      m_linearAccelState.has_sample ? m_linearAccelState.stamp.nanoseconds() : 0;

  if (!(m_latestFrame.has_orientation && m_latestFrame.has_gyro &&
        m_latestFrame.has_linear_accel) ||
      !(m_orientationState.has_sample && m_gyroState.has_sample && m_linearAccelState.has_sample))
  {
    ++m_imuGravityDiagnostics.imu_skipped_missing_core_frame;
    MaybeLogImuGravityDiagnostics();
    return;
  }

  const int64_t orientationNs = m_orientationState.stamp.nanoseconds();
  const int64_t gyroNs = m_gyroState.stamp.nanoseconds();
  const int64_t linearAccelNs = m_linearAccelState.stamp.nanoseconds();
  const int64_t oldestNs = std::min({orientationNs, gyroNs, linearAccelNs});
  const int64_t newestNs = std::max({orientationNs, gyroNs, linearAccelNs});
  const int64_t spanNs = newestNs - oldestNs;
  m_imuGravityDiagnostics.latest_orientation_stamp_ns = orientationNs;
  m_imuGravityDiagnostics.latest_gyro_stamp_ns = gyroNs;
  m_imuGravityDiagnostics.latest_linear_accel_stamp_ns = linearAccelNs;
  m_imuGravityDiagnostics.latest_core_span_ms = static_cast<double>(spanNs) / 1.0e6;

  if (spanNs > DurationFromUs(CoreCoherenceToleranceUs()).nanoseconds())
  {
    ++m_imuGravityDiagnostics.imu_skipped_incoherent_core_frame;
    MaybeLogImuGravityDiagnostics();
    return;
  }

  const CoreFrameSignature signature = LatestCoreSignature();
  if (m_lastPublishedCoreSignature.has_value() &&
      signature.orientation_sequence == m_lastPublishedCoreSignature->orientation_sequence &&
      signature.gyro_sequence == m_lastPublishedCoreSignature->gyro_sequence &&
      signature.linear_accel_sequence == m_lastPublishedCoreSignature->linear_accel_sequence &&
      signature.orientation_stamp_ns == m_lastPublishedCoreSignature->orientation_stamp_ns &&
      signature.gyro_stamp_ns == m_lastPublishedCoreSignature->gyro_stamp_ns &&
      signature.linear_accel_stamp_ns == m_lastPublishedCoreSignature->linear_accel_stamp_ns)
  {
    ++m_imuGravityDiagnostics.imu_skipped_duplicate_core_signature;
    MaybeLogImuGravityDiagnostics();
    return;
  }

  PublishLatestFrame(LatestCoreStamp());
  m_lastPublishedCoreSignature = signature;
  ++m_imuGravityDiagnostics.imu_published;
}

void Bno086ImuNode::MaybePublishImuGravityOnAccelerometer(const SensorEvent& event)
{
  if (event.report_id != ReportId::Accelerometer)
    return;

  ++m_imuGravityDiagnostics.calibrated_accel_reports_received;
  m_imuGravityDiagnostics.latest_accel_stamp_ns =
      m_imuGravityState.has_sample ? m_imuGravityState.stamp.nanoseconds() : 0;
  m_imuGravityDiagnostics.latest_orientation_stamp_ns =
      m_orientationState.has_sample ? m_orientationState.stamp.nanoseconds() : 0;
  m_imuGravityDiagnostics.latest_gyro_stamp_ns =
      m_gyroState.has_sample ? m_gyroState.stamp.nanoseconds() : 0;

  if (!m_orientationState.has_sample || !m_latestFrame.has_orientation)
  {
    ++m_imuGravityDiagnostics.imu_gravity_skipped_missing_orientation;
    MaybeLogImuGravityDiagnostics();
    return;
  }

  if (!m_gyroState.has_sample || !m_latestFrame.has_gyro)
  {
    ++m_imuGravityDiagnostics.imu_gravity_skipped_missing_gyro;
    MaybeLogImuGravityDiagnostics();
    return;
  }

  if (!m_imuGravityState.has_sample || !m_latestFrame.has_accel)
  {
    MaybeLogImuGravityDiagnostics();
    return;
  }

  const int64_t accelStampNs = m_imuGravityState.stamp.nanoseconds();
  const int64_t orientationAgeNs = accelStampNs - m_orientationState.stamp.nanoseconds();
  const int64_t gyroAgeNs = accelStampNs - m_gyroState.stamp.nanoseconds();
  m_imuGravityDiagnostics.latest_orientation_age_ms = static_cast<double>(orientationAgeNs) / 1.0e6;
  m_imuGravityDiagnostics.latest_gyro_age_ms = static_cast<double>(gyroAgeNs) / 1.0e6;

  if (std::abs(m_imuGravityDiagnostics.latest_orientation_age_ms) > m_imuGravityMaxOrientationAgeMs)
  {
    ++m_imuGravityDiagnostics.imu_gravity_skipped_stale_orientation;
    MaybeLogImuGravityDiagnostics();
    return;
  }

  if (std::abs(m_imuGravityDiagnostics.latest_gyro_age_ms) > m_imuGravityMaxGyroAgeMs)
  {
    ++m_imuGravityDiagnostics.imu_gravity_skipped_stale_gyro;
    MaybeLogImuGravityDiagnostics();
    return;
  }

  if (m_lastPublishedImuGravityAccelStampNs.has_value() &&
      accelStampNs <= *m_lastPublishedImuGravityAccelStampNs)
  {
    ++m_imuGravityDiagnostics.imu_gravity_skipped_duplicate_stamp;
    MaybeLogImuGravityDiagnostics();
    return;
  }

  // `imu_gravity` publishes a full sensor_msgs/Imu at calibrated
  // acceleration cadence. Its linear_acceleration field intentionally
  // contains calibrated acceleration including gravity. This is the stream
  // used by monocular-inertial SLAM.
  //
  // BNO08X calibrated acceleration includes gravity, while the linear
  // acceleration report used by `imu` has gravity removed.
  const sensor_msgs::msg::Imu imuGravityMsg = BuildImuGravityMessage(m_imuGravityState.stamp);
  std::string invalidReason;
  if (!IsImuGravitySampleValid(imuGravityMsg, invalidReason))
  {
    ++m_imuGravityDiagnostics.imu_gravity_skipped_nonfinite;
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), IMU_GRAVITY_SAMPLE_WARN_THROTTLE_MS,
                         "Skipping BNO086 imu_gravity sample: %s", invalidReason.c_str());
    MaybeLogImuGravityDiagnostics();
    return;
  }

  m_imuGravityPublisher->publish(imuGravityMsg);
  m_lastPublishedImuGravityAccelStampNs = accelStampNs;
  ++m_imuGravityDiagnostics.imu_gravity_published;
  MaybeLogImuGravityDiagnostics();
}

void Bno086ImuNode::MaybePublishGravityOnGravityReport(const SensorEvent& event)
{
  if (event.report_id != ReportId::Gravity)
    return;

  if (!m_latestFrame.has_gravity)
    return;

  const geometry_msgs::msg::AccelWithCovarianceStamped gravityMsg =
      BuildGravityMessage(m_gravityState.stamp);
  m_gravityPublisher->publish(gravityMsg);
}

void Bno086ImuNode::PublishLatestFrame(const rclcpp::Time& stamp)
{
  if (!(m_latestFrame.has_orientation && m_latestFrame.has_gyro && m_latestFrame.has_linear_accel))
  {
    if (!m_warnedMissingImuFields)
    {
      RCLCPP_WARN(get_logger(), "Waiting for complete BNO086 IMU fields "
                                "(orientation+gyro+linear_accel)");
      m_warnedMissingImuFields = true;
    }
    return;
  }

  m_warnedMissingImuFields = false;

  // `imu` keeps gravity-removed linear acceleration
  const sensor_msgs::msg::Imu imuMsg = BuildPresentImuMessage(stamp);
  const sensor_msgs::msg::Imu predictedImuMsg = BuildPredictedImuMessage(imuMsg);
  const oasis_msgs::msg::ImuVr imuVrMsg = BuildPredictedVrMessage(imuMsg, predictedImuMsg);

  m_imuPublisher->publish(imuMsg);
  m_imuPredictedPublisher->publish(predictedImuMsg);
  m_imuVrPublisher->publish(imuVrMsg);
}

sensor_msgs::msg::Imu Bno086ImuNode::BuildPresentImuMessage(const rclcpp::Time& stamp) const
{
  sensor_msgs::msg::Imu imuMsg;
  imuMsg.header.stamp = stamp;
  imuMsg.header.frame_id = m_frameId;

  imuMsg.orientation.x = m_latestFrame.orientation_xyzw[0];
  imuMsg.orientation.y = m_latestFrame.orientation_xyzw[1];
  imuMsg.orientation.z = m_latestFrame.orientation_xyzw[2];
  imuMsg.orientation.w = m_latestFrame.orientation_xyzw[3];

  imuMsg.angular_velocity.x = m_latestFrame.gyro_rads[0];
  imuMsg.angular_velocity.y = m_latestFrame.gyro_rads[1];
  imuMsg.angular_velocity.z = m_latestFrame.gyro_rads[2];

  imuMsg.linear_acceleration.x = m_latestFrame.linear_accel_mps2[0];
  imuMsg.linear_acceleration.y = m_latestFrame.linear_accel_mps2[1];
  imuMsg.linear_acceleration.z = m_latestFrame.linear_accel_mps2[2];

  imuMsg.orientation_covariance.fill(0.0);
  imuMsg.angular_velocity_covariance.fill(0.0);
  imuMsg.linear_acceleration_covariance.fill(0.0);

  if (m_latestFrame.has_orientation_covariance)
    SetCovariance(imuMsg.orientation_covariance, m_latestFrame.orientation_cov_rad2);

  if (m_latestFrame.has_gyro_covariance)
    SetCovariance(imuMsg.angular_velocity_covariance, m_latestFrame.gyro_cov_rads2_2);

  if (m_latestFrame.has_linear_accel_covariance)
  {
    SetCovariance(imuMsg.linear_acceleration_covariance, m_latestFrame.linear_accel_cov_mps2_2);
  }

  return imuMsg;
}

sensor_msgs::msg::Imu Bno086ImuNode::BuildImuGravityMessage(const rclcpp::Time& stamp) const
{
  sensor_msgs::msg::Imu imuGravityMsg;
  imuGravityMsg.header.stamp = stamp;
  imuGravityMsg.header.frame_id = m_frameId;

  imuGravityMsg.orientation.x = m_latestFrame.orientation_xyzw[0];
  imuGravityMsg.orientation.y = m_latestFrame.orientation_xyzw[1];
  imuGravityMsg.orientation.z = m_latestFrame.orientation_xyzw[2];
  imuGravityMsg.orientation.w = m_latestFrame.orientation_xyzw[3];

  imuGravityMsg.angular_velocity.x = m_latestFrame.gyro_rads[0];
  imuGravityMsg.angular_velocity.y = m_latestFrame.gyro_rads[1];
  imuGravityMsg.angular_velocity.z = m_latestFrame.gyro_rads[2];

  imuGravityMsg.linear_acceleration.x = m_latestFrame.accel_mps2[0];
  imuGravityMsg.linear_acceleration.y = m_latestFrame.accel_mps2[1];
  imuGravityMsg.linear_acceleration.z = m_latestFrame.accel_mps2[2];

  imuGravityMsg.orientation_covariance.fill(0.0);
  imuGravityMsg.angular_velocity_covariance.fill(0.0);
  imuGravityMsg.linear_acceleration_covariance.fill(0.0);

  if (m_latestFrame.has_orientation_covariance)
    SetCovariance(imuGravityMsg.orientation_covariance, m_latestFrame.orientation_cov_rad2);

  if (m_latestFrame.has_gyro_covariance)
    SetCovariance(imuGravityMsg.angular_velocity_covariance, m_latestFrame.gyro_cov_rads2_2);

  if (m_latestFrame.has_accel_covariance)
  {
    SetCovariance(imuGravityMsg.linear_acceleration_covariance, m_latestFrame.accel_cov_mps2_2);
  }

  return imuGravityMsg;
}

geometry_msgs::msg::AccelWithCovarianceStamped Bno086ImuNode::BuildGravityMessage(
    const rclcpp::Time& stamp) const
{
  std::optional<OASIS::IMU::Mat3> gravityCovariance;
  if (m_latestFrame.has_gravity_covariance)
    gravityCovariance = m_latestFrame.gravity_cov_mps2_2;

  const PublishedGravityMeasurement gravityMeasurement =
      MakePublishedGravityMeasurement(m_latestFrame.gravity_mps2, gravityCovariance);

  geometry_msgs::msg::AccelWithCovarianceStamped gravityMsg;
  gravityMsg.header.stamp = stamp;
  gravityMsg.header.frame_id = m_frameId;
  gravityMsg.accel.accel.linear.x = gravityMeasurement.gravity_mps2[0];
  gravityMsg.accel.accel.linear.y = gravityMeasurement.gravity_mps2[1];
  gravityMsg.accel.accel.linear.z = gravityMeasurement.gravity_mps2[2];

  gravityMsg.accel.accel.angular.x = 0.0;
  gravityMsg.accel.accel.angular.y = 0.0;
  gravityMsg.accel.accel.angular.z = 0.0;

  // `gravity` publishes the canonical OASIS gravity vector in
  // `accel.accel.linear`: expressed in `imu_link`, pointing down, and near
  // 9.81 m/s^2 at rest. This is a physical gravity vector, not an "up"
  // vector and not a normalized direction-only unit vector.
  //
  // The rotational covariance block in geometry_msgs/AccelWithCovariance is
  // reserved for angular acceleration. The BNO086 does not estimate angular
  // acceleration on this topic, so covariance[21] remains -1.0 by policy.
  gravityMsg.accel.covariance = gravityMeasurement.covariance;

  return gravityMsg;
}

sensor_msgs::msg::Imu Bno086ImuNode::BuildPredictedImuMessage(
    const sensor_msgs::msg::Imu& present_imu) const
{
  sensor_msgs::msg::Imu predictedImuMsg = present_imu;
  const std::array<double, 4> predictedOrientation = PredictOrientation(
      m_latestFrame.orientation_xyzw, m_latestFrame.gyro_rads, m_predictionHorizonSec);

  predictedImuMsg.orientation.x = predictedOrientation[0];
  predictedImuMsg.orientation.y = predictedOrientation[1];
  predictedImuMsg.orientation.z = predictedOrientation[2];
  predictedImuMsg.orientation.w = predictedOrientation[3];

  double sigmaNoiseRad = 0.0;
  double sigmaRmsRad = 0.0;
  double sigmaBoundRad = 0.0;

  // This covariance is a driver heuristic: SH-2 provides only a coarse
  // accuracy bucket, so the predicted orientation covariance inherits the
  // present-time estimate and only adds host-side horizon growth.
  predictedImuMsg.orientation_covariance =
      PredictedCovarianceFromPresent(present_imu.orientation_covariance, m_predictionHorizonSec,
                                     sigmaNoiseRad, sigmaRmsRad, sigmaBoundRad);

  return predictedImuMsg;
}

oasis_msgs::msg::ImuVr Bno086ImuNode::BuildPredictedVrMessage(
    const sensor_msgs::msg::Imu& present_imu, const sensor_msgs::msg::Imu& predicted_imu) const
{
  oasis_msgs::msg::ImuVr vrMsg;
  const std::uint8_t predictionAccuracy =
      std::min(m_orientationState.accuracy, m_gyroState.accuracy);
  vrMsg.header = present_imu.header;
  vrMsg.valid = m_latestFrame.has_orientation && m_latestFrame.has_gyro;
  vrMsg.source = m_predictionSource;
  vrMsg.prediction_horizon_sec = m_predictionHorizonSec;
  vrMsg.orientation = predicted_imu.orientation;
  vrMsg.orientation_covariance = predicted_imu.orientation_covariance;
  vrMsg.accuracy_status = predictionAccuracy;
  vrMsg.covariance_is_prediction_model_based = m_predictionHorizonSec > 0.0;

  double sigmaNoiseRad = 0.0;
  double sigmaRmsRad = 0.0;
  double sigmaBoundRad = 0.0;
  PredictedCovarianceFromPresent(present_imu.orientation_covariance, m_predictionHorizonSec,
                                 sigmaNoiseRad, sigmaRmsRad, sigmaBoundRad);
  vrMsg.sigma_noise_rad = sigmaNoiseRad;
  vrMsg.sigma_rms_rad = sigmaRmsRad;
  vrMsg.sigma_bound_rad = sigmaBoundRad;

  vrMsg.angular_velocity = present_imu.angular_velocity;
  vrMsg.angular_velocity_covariance = present_imu.angular_velocity_covariance;
  vrMsg.linear_acceleration = present_imu.linear_acceleration;
  vrMsg.linear_acceleration_covariance = present_imu.linear_acceleration_covariance;
  return vrMsg;
}

void Bno086ImuNode::MaybeLogFeatureResponses()
{
  if (m_shtp == nullptr)
    return;

  const std::vector<FeatureResponse> featureResponses = m_shtp->TakeFeatureResponses();
  for (const FeatureResponse& featureResponse : featureResponses)
  {
    const auto it = std::find_if(m_latestFeatureResponses.begin(), m_latestFeatureResponses.end(),
                                 [&featureResponse](const FeatureResponse& response)
                                 { return response.report_id == featureResponse.report_id; });
    if (it == m_latestFeatureResponses.end())
      m_latestFeatureResponses.emplace_back(featureResponse);
    else
      *it = featureResponse;

    LogFeatureResponse(featureResponse);
  }

  MaybeLogFeatureSummary();
}

void Bno086ImuNode::LogFeatureResponse(const FeatureResponse& response)
{
  const std::optional<std::uint32_t> requestedIntervalUs =
      RequestedFeatureIntervalUs(response.report_id);
  const std::optional<std::uint32_t> requestedBatchUs =
      RequestedFeatureBatchIntervalUs(response.report_id);
  const std::uint32_t requestedUs = requestedIntervalUs.value_or(0U);
  const std::uint32_t requestedBatch = requestedBatchUs.value_or(0U);
  const double actualRateHz = response.report_interval_us > 0
                                  ? 1'000'000.0 / static_cast<double>(response.report_interval_us)
                                  : 0.0;

  const std::optional<std::size_t> reportIndex = DiagnosticReportIndex(response.report_id);
  if (reportIndex.has_value())
    m_imuGravityDiagnostics.latest_feature_rate_hz[*reportIndex] = actualRateHz;

  RCLCPP_INFO(get_logger(),
              "BNO086 Get Feature Response: report=%s id=0x%02X requested_interval_us=%u "
              "actual_interval_us=%u actual_rate_hz=%.2f requested_batch_us=%u "
              "actual_batch_us=%u batching_active=%s feature_flags=0x%02X "
              "sensor_specific_config=0x%08X",
              ReportName(response.report_id), static_cast<unsigned>(response.report_id),
              requestedUs, response.report_interval_us, actualRateHz, requestedBatch,
              response.batch_interval_us, IsFeatureBatchingActive(response) ? "true" : "false",
              static_cast<unsigned>(response.feature_flags), response.sensor_specific_config);
}

void Bno086ImuNode::MaybeLogFeatureSummary()
{
  if (m_loggedFeatureSummary || m_shtp == nullptr)
    return;

  const std::vector<FeatureConfiguration>& configurations = m_shtp->GetFeatureConfigurations();
  if (configurations.empty())
    return;

  const bool receivedAll = m_latestFeatureResponses.size() >= configurations.size();
  bool timedOut = false;
  if (!receivedAll && m_featureConfigurationStartedAt.time_since_epoch().count() != 0)
  {
    const auto elapsedMs = std::chrono::duration_cast<std::chrono::milliseconds>(
                               std::chrono::steady_clock::now() - m_featureConfigurationStartedAt)
                               .count();
    timedOut = elapsedMs >= m_featureSummaryTimeoutMs;
  }

  if (!receivedAll && !timedOut)
    return;

  LogFeatureSummary();
  m_loggedFeatureSummary = true;
}

void Bno086ImuNode::LogFeatureSummary() const
{
  if (m_shtp == nullptr)
    return;

  const std::vector<FeatureConfiguration>& configurations = m_shtp->GetFeatureConfigurations();
  std::ostringstream oss;

  if (m_latestFeatureResponses.size() >= configurations.size())
  {
    oss << "BNO086 feature acceptance summary:";
  }
  else
  {
    oss << "BNO086 feature acceptance summary incomplete: received="
        << m_latestFeatureResponses.size() << " expected=" << configurations.size();
  }

  for (const FeatureConfiguration& configuration : configurations)
  {
    oss << "\n  "
        << BuildFeatureSummaryLine(configuration, LatestFeatureResponse(configuration.report_id));
  }

  RCLCPP_INFO(get_logger(), "%s", oss.str().c_str());
}

std::string Bno086ImuNode::BuildFeatureSummaryLine(
    const FeatureConfiguration& configuration, const std::optional<FeatureResponse>& response) const
{
  std::ostringstream oss;
  oss << ReportName(configuration.report_id)
      << " requested_interval_us=" << configuration.requested_interval_us
      << " requested_batch_us=" << configuration.requested_batch_interval_us;

  if (!response.has_value())
  {
    oss << " status=missing_response";
    return oss.str();
  }

  oss << " actual_interval_us=" << response->report_interval_us
      << " actual_batch_us=" << response->batch_interval_us
      << " batching_active=" << (IsFeatureBatchingActive(*response) ? "true" : "false");
  return oss.str();
}

void Bno086ImuNode::MaybeLogImuGravityDiagnostics()
{
  const auto now = std::chrono::steady_clock::now();
  if (m_imuGravityDiagnostics.last_log_at.time_since_epoch().count() != 0)
  {
    const auto elapsedMs = std::chrono::duration_cast<std::chrono::milliseconds>(
                               now - m_imuGravityDiagnostics.last_log_at)
                               .count();
    if (elapsedMs < m_diagnosticsLogPeriodMs)
      return;
  }

  UpdateImuGravityDiagnosticsRates(now);
  MaybeEmitImuGravityDiagnosticsLog();
  m_imuGravityDiagnostics.last_log_at = now;
}

void Bno086ImuNode::UpdateImuGravityDiagnosticsRates(
    const std::chrono::steady_clock::time_point& now)
{
  if (m_imuGravityDiagnostics.last_log_at.time_since_epoch().count() == 0)
  {
    m_imuGravityDiagnostics.latest_skipped_stale_orientation_delta = 0;
    m_imuGravityDiagnostics.latest_skipped_stale_gyro_delta = 0;
    m_imuGravityDiagnostics.latest_accel_sequence_gap_delta = 0;
    m_imuGravityDiagnostics.latest_timestamp_reconstruction_reset_delta = 0;
  }
  else
  {
    const auto elapsedMs = std::chrono::duration_cast<std::chrono::milliseconds>(
                               now - m_imuGravityDiagnostics.last_log_at)
                               .count();
    if (elapsedMs > 0)
    {
      const std::uint64_t accelReportDelta =
          m_imuGravityDiagnostics.calibrated_accel_reports_received -
          m_imuGravityDiagnostics.last_rate_accel_reports;
      const std::uint64_t imuGravityPublishedDelta =
          m_imuGravityDiagnostics.imu_gravity_published -
          m_imuGravityDiagnostics.last_rate_imu_gravity_published;
      m_imuGravityDiagnostics.latest_calibrated_accel_rate_hz =
          static_cast<double>(accelReportDelta) * 1000.0 / static_cast<double>(elapsedMs);
      m_imuGravityDiagnostics.latest_imu_gravity_rate_hz =
          static_cast<double>(imuGravityPublishedDelta) * 1000.0 / static_cast<double>(elapsedMs);

      for (std::size_t i = 0; i < m_imuGravityDiagnostics.latest_decoded_rate_hz.size(); ++i)
      {
        const std::uint64_t decodedDelta = m_imuGravityDiagnostics.decoded_reports_received[i] -
                                           m_imuGravityDiagnostics.last_rate_decoded_reports[i];
        m_imuGravityDiagnostics.latest_decoded_rate_hz[i] =
            static_cast<double>(decodedDelta) * 1000.0 / static_cast<double>(elapsedMs);
      }
    }

    m_imuGravityDiagnostics.latest_skipped_stale_orientation_delta =
        m_imuGravityDiagnostics.imu_gravity_skipped_stale_orientation -
        m_imuGravityDiagnostics.last_rate_skipped_stale_orientation;
    m_imuGravityDiagnostics.latest_skipped_stale_gyro_delta =
        m_imuGravityDiagnostics.imu_gravity_skipped_stale_gyro -
        m_imuGravityDiagnostics.last_rate_skipped_stale_gyro;
    m_imuGravityDiagnostics.latest_accel_sequence_gap_delta =
        m_imuGravityDiagnostics.accel_sequence_gap_count -
        m_imuGravityDiagnostics.last_rate_accel_sequence_gap_count;
    m_imuGravityDiagnostics.latest_timestamp_reconstruction_reset_delta =
        m_imuGravityDiagnostics.timestamp_reconstruction_reset_count -
        m_imuGravityDiagnostics.last_rate_timestamp_reconstruction_reset_count;
  }

  m_imuGravityDiagnostics.last_rate_accel_reports =
      m_imuGravityDiagnostics.calibrated_accel_reports_received;
  m_imuGravityDiagnostics.last_rate_imu_gravity_published =
      m_imuGravityDiagnostics.imu_gravity_published;
  m_imuGravityDiagnostics.last_rate_decoded_reports =
      m_imuGravityDiagnostics.decoded_reports_received;
  m_imuGravityDiagnostics.last_rate_skipped_stale_orientation =
      m_imuGravityDiagnostics.imu_gravity_skipped_stale_orientation;
  m_imuGravityDiagnostics.last_rate_skipped_stale_gyro =
      m_imuGravityDiagnostics.imu_gravity_skipped_stale_gyro;
  m_imuGravityDiagnostics.last_rate_accel_sequence_gap_count =
      m_imuGravityDiagnostics.accel_sequence_gap_count;
  m_imuGravityDiagnostics.last_rate_timestamp_reconstruction_reset_count =
      m_imuGravityDiagnostics.timestamp_reconstruction_reset_count;
}

void Bno086ImuNode::MaybeEmitImuGravityDiagnosticsLog()
{
  const bool unhealthy = IsBno086DiagnosticsUnhealthy();
  if (!ShouldEmitBno086Diagnostics(m_diagnosticsLogLevel, unhealthy))
  {
    if (m_diagnosticsLogLevel != Bno086DiagnosticsLogLevel::Off && m_diagnosticsWasUnhealthy &&
        !unhealthy)
    {
      RCLCPP_INFO(get_logger(),
                  "BNO086 imu_gravity recovered: imu_gravity_rate_hz=%.2f "
                  "accel_rate_hz=%.2f",
                  m_imuGravityDiagnostics.latest_imu_gravity_rate_hz,
                  m_imuGravityDiagnostics.latest_calibrated_accel_rate_hz);
    }

    m_diagnosticsWasUnhealthy = unhealthy;
    return;
  }

  const std::string message = BuildImuGravityDiagnosticsLogMessage();
  switch (m_diagnosticsLogLevel)
  {
    case Bno086DiagnosticsLogLevel::Debug:
      RCLCPP_DEBUG(get_logger(), "%s", message.c_str());
      break;
    case Bno086DiagnosticsLogLevel::Info:
      RCLCPP_INFO(get_logger(), "%s", message.c_str());
      break;
    case Bno086DiagnosticsLogLevel::Warn:
      RCLCPP_WARN(get_logger(), "%s", message.c_str());
      break;
    case Bno086DiagnosticsLogLevel::Off:
      break;
    default:
      break;
  }

  if (m_diagnosticsWasUnhealthy && !unhealthy)
  {
    RCLCPP_INFO(get_logger(),
                "BNO086 imu_gravity recovered: imu_gravity_rate_hz=%.2f "
                "accel_rate_hz=%.2f",
                m_imuGravityDiagnostics.latest_imu_gravity_rate_hz,
                m_imuGravityDiagnostics.latest_calibrated_accel_rate_hz);
  }

  m_diagnosticsWasUnhealthy = unhealthy;
}

std::string Bno086ImuNode::BuildImuGravityDiagnosticsLogMessage() const
{
  std::ostringstream oss;
  oss << "BNO086 imu_gravity diagnostics: " << "accel_events_seen="
      << m_imuGravityDiagnostics.calibrated_accel_reports_received << " "
      << "calibrated_accel_reports_received="
      << m_imuGravityDiagnostics.calibrated_accel_reports_received << " "
      << "imu_gravity_published=" << m_imuGravityDiagnostics.imu_gravity_published << " "
      << "skipped_missing_orientation="
      << m_imuGravityDiagnostics.imu_gravity_skipped_missing_orientation << " "
      << "skipped_missing_gyro=" << m_imuGravityDiagnostics.imu_gravity_skipped_missing_gyro << " "
      << "skipped_stale_orientation="
      << m_imuGravityDiagnostics.imu_gravity_skipped_stale_orientation << " "
      << "skipped_stale_gyro=" << m_imuGravityDiagnostics.imu_gravity_skipped_stale_gyro << " "
      << "skipped_duplicate_stamp=" << m_imuGravityDiagnostics.imu_gravity_skipped_duplicate_stamp
      << " " << "skipped_nonfinite=" << m_imuGravityDiagnostics.imu_gravity_skipped_nonfinite << " "
      << "timestamp_repaired_nonmonotonic_accel="
      << m_imuGravityDiagnostics.timestamp_repaired_nonmonotonic_accel << " "
      << "timestamp_repaired_duplicate_to_interval="
      << m_imuGravityDiagnostics.timestamp_repaired_duplicate_to_interval << " "
      << "timestamp_repaired_nonmonotonic_to_interval="
      << m_imuGravityDiagnostics.timestamp_repaired_nonmonotonic_to_interval << " "
      << "timestamp_repaired_sequence_gap_to_interval="
      << m_imuGravityDiagnostics.timestamp_repaired_sequence_gap_to_interval << " "
      << "timestamp_interval_repair_clamped_to_host="
      << m_imuGravityDiagnostics.timestamp_interval_repair_clamped_to_host << " "
      << "timestamp_interval_repair_bounded_to_legacy="
      << m_imuGravityDiagnostics.timestamp_interval_repair_bounded_to_legacy << " "
      << "accel_sequence_gap_count=" << m_imuGravityDiagnostics.accel_sequence_gap_count << " "
      << "timestamp_reconstruction_reset_count="
      << m_imuGravityDiagnostics.timestamp_reconstruction_reset_count << " "
      << "latest_timestamp_repair_interval_us="
      << m_imuGravityDiagnostics.latest_timestamp_repair_interval_us << " "
      << "latest_accel_stamp_ns=" << m_imuGravityDiagnostics.latest_accel_stamp_ns << " "
      << "latest_orientation_stamp_ns=" << m_imuGravityDiagnostics.latest_orientation_stamp_ns
      << " " << "latest_gyro_stamp_ns=" << m_imuGravityDiagnostics.latest_gyro_stamp_ns << " "
      << "latest_orientation_age_ms=" << m_imuGravityDiagnostics.latest_orientation_age_ms << " "
      << "latest_gyro_age_ms=" << m_imuGravityDiagnostics.latest_gyro_age_ms << " "
      << "linear_accel_events_seen=" << m_imuGravityDiagnostics.linear_accel_events_seen << " "
      << "imu_published=" << m_imuGravityDiagnostics.imu_published << " "
      << "imu_skipped_missing_core_frame=" << m_imuGravityDiagnostics.imu_skipped_missing_core_frame
      << " " << "imu_skipped_incoherent_core_frame="
      << m_imuGravityDiagnostics.imu_skipped_incoherent_core_frame << " "
      << "imu_skipped_duplicate_core_signature="
      << m_imuGravityDiagnostics.imu_skipped_duplicate_core_signature << " "
      << "latest_core_span_ms=" << m_imuGravityDiagnostics.latest_core_span_ms << " "
      << "latest_linear_accel_stamp_ns=" << m_imuGravityDiagnostics.latest_linear_accel_stamp_ns
      << " " << "latest_calibrated_accel_rate_hz="
      << m_imuGravityDiagnostics.latest_calibrated_accel_rate_hz << " "
      << "latest_imu_gravity_rate_hz=" << m_imuGravityDiagnostics.latest_imu_gravity_rate_hz << " "
      << "decoded_accel_rate_hz=" << m_imuGravityDiagnostics.latest_decoded_rate_hz[0] << " "
      << "decoded_gyro_rate_hz=" << m_imuGravityDiagnostics.latest_decoded_rate_hz[1] << " "
      << "decoded_rotation_rate_hz=" << m_imuGravityDiagnostics.latest_decoded_rate_hz[2] << " "
      << "decoded_linear_rate_hz=" << m_imuGravityDiagnostics.latest_decoded_rate_hz[3] << " "
      << "decoded_gravity_rate_hz=" << m_imuGravityDiagnostics.latest_decoded_rate_hz[4];
  return oss.str();
}

bool Bno086ImuNode::IsImuGravitySampleValid(const sensor_msgs::msg::Imu& message,
                                            std::string& invalid_reason) const
{
  if (message.header.stamp.sec == 0 && message.header.stamp.nanosec == 0)
  {
    invalid_reason = "zero header stamp";
    return false;
  }

  const bool orientationFinite =
      std::isfinite(message.orientation.x) && std::isfinite(message.orientation.y) &&
      std::isfinite(message.orientation.z) && std::isfinite(message.orientation.w);
  if (!orientationFinite)
  {
    invalid_reason = "nonfinite orientation quaternion";
    return false;
  }

  const bool gyroFinite = std::isfinite(message.angular_velocity.x) &&
                          std::isfinite(message.angular_velocity.y) &&
                          std::isfinite(message.angular_velocity.z);
  if (!gyroFinite)
  {
    invalid_reason = "nonfinite angular velocity";
    return false;
  }

  const bool accelFinite = std::isfinite(message.linear_acceleration.x) &&
                           std::isfinite(message.linear_acceleration.y) &&
                           std::isfinite(message.linear_acceleration.z);
  if (!accelFinite)
  {
    invalid_reason = "nonfinite calibrated acceleration";
    return false;
  }

  if (!IsFiniteArray(message.orientation_covariance) ||
      !IsFiniteArray(message.angular_velocity_covariance) ||
      !IsFiniteArray(message.linear_acceleration_covariance))
  {
    invalid_reason = "nonfinite covariance";
    return false;
  }

  const double accelMagnitude = VectorMagnitude(
      message.linear_acceleration.x, message.linear_acceleration.y, message.linear_acceleration.z);
  if (accelMagnitude >= MAX_IMU_GRAVITY_ACCEL_MAGNITUDE_MPS2)
  {
    invalid_reason = "implausible calibrated acceleration magnitude";
    return false;
  }

  const double gyroMagnitude = VectorMagnitude(
      message.angular_velocity.x, message.angular_velocity.y, message.angular_velocity.z);
  if (gyroMagnitude >= MAX_IMU_GRAVITY_GYRO_MAGNITUDE_RADS)
  {
    invalid_reason = "implausible angular velocity magnitude";
    return false;
  }

  return true;
}

std::optional<std::uint32_t> Bno086ImuNode::RequestedFeatureIntervalUs(ReportId report_id) const
{
  if (m_shtp == nullptr)
    return std::nullopt;

  const std::vector<FeatureConfiguration>& featureConfigurations =
      m_shtp->GetFeatureConfigurations();
  const auto it = std::find_if(featureConfigurations.begin(), featureConfigurations.end(),
                               [report_id](const FeatureConfiguration& configuration)
                               { return configuration.report_id == report_id; });

  if (it == featureConfigurations.end())
    return std::nullopt;

  return it->requested_interval_us;
}

std::optional<std::uint32_t> Bno086ImuNode::RequestedFeatureBatchIntervalUs(
    ReportId report_id) const
{
  if (m_shtp == nullptr)
    return std::nullopt;

  const std::vector<FeatureConfiguration>& featureConfigurations =
      m_shtp->GetFeatureConfigurations();
  const auto it = std::find_if(featureConfigurations.begin(), featureConfigurations.end(),
                               [report_id](const FeatureConfiguration& configuration)
                               { return configuration.report_id == report_id; });

  if (it == featureConfigurations.end())
    return std::nullopt;

  return it->requested_batch_interval_us;
}

std::optional<std::uint32_t> Bno086ImuNode::EffectiveReportIntervalUs(ReportId report_id) const
{
  const std::optional<FeatureResponse> response = LatestFeatureResponse(report_id);
  if (response.has_value() && response->report_interval_us > 0)
    return response->report_interval_us;

  const std::optional<std::uint32_t> requestedIntervalUs = RequestedFeatureIntervalUs(report_id);
  if (requestedIntervalUs.has_value() && *requestedIntervalUs > 0)
    return requestedIntervalUs;

  switch (report_id)
  {
    case ReportId::Accelerometer:
      return RateHzToIntervalUs(m_accelerometerRateHz);
    case ReportId::GyroscopeCalibrated:
      return RateHzToIntervalUs(m_gyroRateHz);
    case ReportId::RotationVector:
      return RateHzToIntervalUs(m_rotationVectorRateHz);
    case ReportId::LinearAcceleration:
      return RateHzToIntervalUs(m_linearAccelerationRateHz);
    case ReportId::Gravity:
      return RateHzToIntervalUs(m_gravityRateHz);
    default:
      break;
  }

  return std::nullopt;
}

std::optional<FeatureResponse> Bno086ImuNode::LatestFeatureResponse(ReportId report_id) const
{
  const auto it = std::find_if(m_latestFeatureResponses.begin(), m_latestFeatureResponses.end(),
                               [report_id](const FeatureResponse& response)
                               { return response.report_id == report_id; });

  if (it == m_latestFeatureResponses.end())
    return std::nullopt;

  return *it;
}

bool Bno086ImuNode::IsBno086RateUnhealthy() const
{
  if (m_imuGravityDiagnostics.latest_calibrated_accel_rate_hz > 0.0 &&
      m_imuGravityDiagnostics.latest_calibrated_accel_rate_hz <
          m_accelerometerRateHz * MIN_HEALTHY_RATE_FRACTION)
  {
    return true;
  }

  if (m_imuGravityDiagnostics.latest_imu_gravity_rate_hz > 0.0 &&
      m_imuGravityDiagnostics.latest_imu_gravity_rate_hz <
          m_accelerometerRateHz * MIN_HEALTHY_RATE_FRACTION)
  {
    return true;
  }

  return false;
}

bool Bno086ImuNode::IsBno086DiagnosticsUnhealthy() const
{
  if (IsBno086RateUnhealthy())
    return true;

  if (m_imuGravityDiagnostics.latest_skipped_stale_orientation_delta > 0)
    return true;

  if (m_imuGravityDiagnostics.latest_skipped_stale_gyro_delta > 0)
    return true;

  if (m_imuGravityDiagnostics.latest_accel_sequence_gap_delta > 0)
    return true;

  if (m_imuGravityDiagnostics.latest_timestamp_reconstruction_reset_delta > 0)
    return true;

  return false;
}

void Bno086ImuNode::CountDecodedReport(const SensorEvent& event)
{
  const std::optional<std::size_t> reportIndex = DiagnosticReportIndex(event.report_id);
  if (reportIndex.has_value())
    ++m_imuGravityDiagnostics.decoded_reports_received[*reportIndex];
}

std::uint32_t Bno086ImuNode::CoreCoherenceToleranceUs() const
{
  const std::uint32_t scaledToleranceUs = m_reportIntervalUs + (m_reportIntervalUs / 2);
  return std::clamp(scaledToleranceUs, MIN_COHERENT_SAMPLE_SPAN_US, MAX_COHERENT_SAMPLE_SPAN_US);
}

bool Bno086ImuNode::HasPublishableCoreFrame() const
{
  if (!(m_latestFrame.has_orientation && m_latestFrame.has_gyro && m_latestFrame.has_linear_accel))
    return false;

  if (!(m_orientationState.has_sample && m_gyroState.has_sample && m_linearAccelState.has_sample))
    return false;

  const int64_t orientationNs = m_orientationState.stamp.nanoseconds();
  const int64_t gyroNs = m_gyroState.stamp.nanoseconds();
  const int64_t linearAccelNs = m_linearAccelState.stamp.nanoseconds();

  const int64_t oldestNs = std::min({orientationNs, gyroNs, linearAccelNs});
  const int64_t newestNs = std::max({orientationNs, gyroNs, linearAccelNs});
  const int64_t spanNs = newestNs - oldestNs;
  return spanNs <= DurationFromUs(CoreCoherenceToleranceUs()).nanoseconds();
}

Bno086ImuNode::CoreFrameSignature Bno086ImuNode::LatestCoreSignature() const
{
  CoreFrameSignature signature;
  signature.orientation_sequence = m_orientationState.sequence;
  signature.gyro_sequence = m_gyroState.sequence;
  signature.linear_accel_sequence = m_linearAccelState.sequence;
  signature.orientation_stamp_ns = m_orientationState.stamp.nanoseconds();
  signature.gyro_stamp_ns = m_gyroState.stamp.nanoseconds();
  signature.linear_accel_stamp_ns = m_linearAccelState.stamp.nanoseconds();
  return signature;
}

rclcpp::Time Bno086ImuNode::LatestCoreStamp() const
{
  const int64_t orientationNs = m_orientationState.stamp.nanoseconds();
  const int64_t gyroNs = m_gyroState.stamp.nanoseconds();
  const int64_t linearAccelNs = m_linearAccelState.stamp.nanoseconds();
  return rclcpp::Time(std::max({orientationNs, gyroNs, linearAccelNs}), RCL_ROS_TIME);
}

void Bno086ImuNode::ApplyEvent(const SensorEvent& event, const rclcpp::Time& sample_stamp)
{
  switch (event.report_id)
  {
    case ReportId::RotationVector:
    {
      const auto accuracyEstimateIndex =
          static_cast<std::size_t>(RotationVectorValueIndex::AccuracyEstimate);
      const OrientationCovariancePolicyResult covariancePolicy =
          ResolveOrientationCovariancePolicy(event.accuracy, event.values[accuracyEstimateIndex]);

      m_latestFrame.orientation_xyzw[0] = QToDouble(
          event.values[static_cast<std::size_t>(RotationVectorValueIndex::QuaternionI)], 14);
      m_latestFrame.orientation_xyzw[1] = QToDouble(
          event.values[static_cast<std::size_t>(RotationVectorValueIndex::QuaternionJ)], 14);
      m_latestFrame.orientation_xyzw[2] = QToDouble(
          event.values[static_cast<std::size_t>(RotationVectorValueIndex::QuaternionK)], 14);
      m_latestFrame.orientation_xyzw[3] = QToDouble(
          event.values[static_cast<std::size_t>(RotationVectorValueIndex::QuaternionReal)], 14);
      NormalizeQuaternion(m_latestFrame.orientation_xyzw);
      m_latestFrame.has_orientation = true;
      m_orientationState.has_sample = true;
      m_orientationState.stamp = sample_stamp;
      m_orientationState.sequence = event.sequence;
      m_orientationState.accuracy = event.accuracy;

      // The policy module owns the driver-boundary heuristic that turns the
      // SH-2 Rotation Vector report into the published ROS orientation
      // covariance contract:
      //
      // - prefer the report's own estimated accuracy field when it is present
      //   and sane
      // - otherwise fall back to a documented, intentionally tighter SH-2
      //   accuracy-bucket heuristic for usable fused-attitude uncertainty
      // - publish an axis-aligned covariance because SH-2 does not expose a
      //   full 3x3 orientation covariance
      //
      // The node only applies that result and reports which source was used.
      // Downstream AHRS preserves and rotates the published covariance.
      m_latestFrame.orientation_cov_rad2 = covariancePolicy.covariance_rad2;
      m_latestFrame.has_orientation_covariance = true;
      m_orientationCovarianceDebug.accuracy_bucket = covariancePolicy.accuracy_bucket;
      m_orientationCovarianceDebug.raw_accuracy_estimate_q12 =
          covariancePolicy.raw_accuracy_estimate_q12;
      m_orientationCovarianceDebug.sigma_rad = covariancePolicy.sigma_rad;
      m_orientationCovarianceDebug.source = covariancePolicy.source;
      m_orientationCovarianceDebug.has_accuracy_estimate = covariancePolicy.has_accuracy_estimate;
      m_orientationCovarianceDebug.accuracy_estimate_rad = covariancePolicy.accuracy_estimate_rad;
      m_orientationCovarianceDebug.rejection_reason = covariancePolicy.rejection_reason;
      MaybeLogOrientationCovariancePolicy();
      break;
    }

    case ReportId::GyroscopeCalibrated:
      m_latestFrame.gyro_rads[0] = QToDouble(event.values[0], 9);
      m_latestFrame.gyro_rads[1] = QToDouble(event.values[1], 9);
      m_latestFrame.gyro_rads[2] = QToDouble(event.values[2], 9);
      m_latestFrame.has_gyro = true;
      m_gyroState.has_sample = true;
      m_gyroState.stamp = sample_stamp;
      m_gyroState.sequence = event.sequence;
      m_gyroState.accuracy = event.accuracy;

      m_latestFrame.gyro_cov_rads2_2 =
          Bno086ImuNode::CovarianceFromAccuracyBucket(event.accuracy, 1.2, 0.5, 0.18, 0.06);
      m_latestFrame.has_gyro_covariance = true;
      break;

    case ReportId::LinearAcceleration:
      m_latestFrame.linear_accel_mps2[0] = QToDouble(event.values[0], 8);
      m_latestFrame.linear_accel_mps2[1] = QToDouble(event.values[1], 8);
      m_latestFrame.linear_accel_mps2[2] = QToDouble(event.values[2], 8);
      m_latestFrame.has_linear_accel = true;
      m_linearAccelState.has_sample = true;
      m_linearAccelState.stamp = sample_stamp;
      m_linearAccelState.sequence = event.sequence;
      m_linearAccelState.accuracy = event.accuracy;

      m_latestFrame.linear_accel_cov_mps2_2 =
          Bno086ImuNode::CovarianceFromAccuracyBucket(event.accuracy, 4.0, 2.0, 0.8, 0.25);
      m_latestFrame.has_linear_accel_covariance = true;
      break;

    case ReportId::Accelerometer:
      m_latestFrame.accel_mps2[0] = QToDouble(event.values[0], 8);
      m_latestFrame.accel_mps2[1] = QToDouble(event.values[1], 8);
      m_latestFrame.accel_mps2[2] = QToDouble(event.values[2], 8);
      m_latestFrame.has_accel = true;
      m_imuGravityState.has_sample = true;
      m_imuGravityState.stamp = sample_stamp;
      m_imuGravityState.sequence = event.sequence;
      m_imuGravityState.accuracy = event.accuracy;

      m_latestFrame.accel_cov_mps2_2 =
          Bno086ImuNode::CovarianceFromAccuracyBucket(event.accuracy, 4.0, 2.0, 0.8, 0.25);
      m_latestFrame.has_accel_covariance = true;
      break;

    case ReportId::Gravity:
    {
      const OASIS::IMU::Vec3 rawGravityMps2{
          QToDouble(event.values[0], 8),
          QToDouble(event.values[1], 8),
          QToDouble(event.values[2], 8),
      };

      // SH-2 gravity is ingested here and converted immediately into the
      // canonical OASIS gravity convention. The public `gravity` topic must
      // carry a gravity vector in `imu_link` that points down in the direction
      // of gravitational acceleration, so the raw BNO sign is flipped once at
      // ingestion and the rest of the stack can treat gravity consistently.
      m_latestFrame.gravity_mps2 = CanonicalizeGravityVector(rawGravityMps2);
      m_latestFrame.gravity_cov_mps2_2 =
          Bno086ImuNode::CovarianceFromAccuracyBucket(event.accuracy, 1.0, 0.5, 0.2, 0.08);
      m_latestFrame.has_gravity = true;
      m_latestFrame.has_gravity_covariance = true;
      m_gravityState.has_sample = true;
      m_gravityState.stamp = sample_stamp;
      m_gravityState.sequence = event.sequence;
      m_gravityState.accuracy = event.accuracy;
      break;
    }

    default:
      break;
  }
}

rclcpp::Time Bno086ImuNode::EstimateEventStamp(const SensorEvent& event,
                                               const rclcpp::Time& interrupt_ros_at)
{
  if (!event.has_base_timestamp)
  {
    rclcpp::Time fallbackStamp = interrupt_ros_at;
    if (event.has_delay)
      fallbackStamp = fallbackStamp - DurationFromUs(event.delay_us);

    return fallbackStamp;
  }

  const TimestampMappingResult mappedStamp = m_timestampMapper.Map(TimestampMappingInput{
      event.has_base_timestamp,
      event.base_timestamp_us,
      event.has_delay,
      event.delay_us,
      event.sequence,
      interrupt_ros_at.nanoseconds(),
  });

  if (mappedStamp.initialized_offset || mappedStamp.reanchored_offset ||
      mappedStamp.detected_wrap_or_reset || mappedStamp.rejected_implausible_mapping)
  {
    RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 2000,
                          "BNO086 timestamp mapper state: initialized=%s reanchored=%s "
                          "wrap_or_reset=%s rejected_implausible=%s",
                          mappedStamp.initialized_offset ? "true" : "false",
                          mappedStamp.reanchored_offset ? "true" : "false",
                          mappedStamp.detected_wrap_or_reset ? "true" : "false",
                          mappedStamp.rejected_implausible_mapping ? "true" : "false");
  }

  return rclcpp::Time(mappedStamp.stamp_ns, RCL_ROS_TIME);
}

std::array<double, 9> Bno086ImuNode::PredictedCovarianceFromPresent(
    const std::array<double, 9>& present_orientation_covariance,
    double prediction_horizon_sec,
    double& sigma_noise_rad,
    double& sigma_rms_rad,
    double& sigma_bound_rad)
{
  // Present orientation covariance comes from the driver-owned SH-2 policy:
  // Rotation Vector estimated accuracy when available, otherwise the fallback
  // accuracy bucket table. Host prediction should never be more confident than
  // that estimate, so start from the present-time diagonal and add only
  // nonnegative growth.
  //
  // The growth term models additional small-angle variance that accumulates
  // while integrating gyro forward in time:
  //
  //   sigma_growth = k_prediction_sigma_rate_rad_per_sec * horizon
  //   variance_growth = sigma_growth^2
  //
  // This keeps Sigma_pred(h=0) == Sigma_present and makes the diagonal grow
  // monotonically for h > 0.
  constexpr double kPredictionSigmaRateRadPerSec = 0.05;

  std::array<double, 9> predictedCovariance{};
  predictedCovariance.fill(0.0);

  const double clampedHorizonSec = std::max(prediction_horizon_sec, 0.0);
  sigma_noise_rad = kPredictionSigmaRateRadPerSec * clampedHorizonSec;

  const double growthVarianceRad2 = sigma_noise_rad * sigma_noise_rad;
  double maxPredictedVarianceRad2 = 0.0;

  for (std::size_t axis = 0; axis < 3; ++axis)
  {
    const std::size_t diagonalIndex = (axis * 3) + axis;
    const double presentVarianceRad2 = std::max(present_orientation_covariance[diagonalIndex], 0.0);
    const double predictedVarianceRad2 = presentVarianceRad2 + growthVarianceRad2;
    predictedCovariance[diagonalIndex] = predictedVarianceRad2;
    maxPredictedVarianceRad2 = std::max(maxPredictedVarianceRad2, predictedVarianceRad2);
  }

  sigma_rms_rad = std::sqrt(maxPredictedVarianceRad2);
  sigma_bound_rad = sigma_rms_rad;
  return predictedCovariance;
}

rclcpp::Duration Bno086ImuNode::DurationFromUs(std::uint32_t microseconds)
{
  return rclcpp::Duration(0, static_cast<int64_t>(microseconds) * 1000);
}

OASIS::IMU::Mat3 Bno086ImuNode::CovarianceFromAccuracyBucket(std::uint8_t accuracy,
                                                             double sigma_unreliable,
                                                             double sigma_low,
                                                             double sigma_medium,
                                                             double sigma_high)
{
  const double sigma = (accuracy >= 3)   ? sigma_high
                       : (accuracy == 2) ? sigma_medium
                       : (accuracy == 1) ? sigma_low
                                         : sigma_unreliable;

  const double variance = sigma * sigma;

  OASIS::IMU::Mat3 covariance{};
  covariance[0][0] = variance;
  covariance[0][1] = 0.0;
  covariance[0][2] = 0.0;

  covariance[1][0] = 0.0;
  covariance[1][1] = variance;
  covariance[1][2] = 0.0;

  covariance[2][0] = 0.0;
  covariance[2][1] = 0.0;
  covariance[2][2] = variance;
  return covariance;
}

const char* Bno086ImuNode::ReportName(ReportId report_id)
{
  switch (report_id)
  {
    case ReportId::Accelerometer:
      return "accelerometer";
    case ReportId::GyroscopeCalibrated:
      return "gyro";
    case ReportId::LinearAcceleration:
      return "linear_acceleration";
    case ReportId::RotationVector:
      return "rotation_vector";
    case ReportId::Gravity:
      return "gravity";
    default:
      break;
  }

  return "unknown";
}

double Bno086ImuNode::QToDouble(std::int16_t value, unsigned q_point)
{
  return static_cast<double>(value) / static_cast<double>(1U << q_point);
}

void Bno086ImuNode::NormalizeQuaternion(std::array<double, 4>& q)
{
  const double norm = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);

  if (norm <= 0.0)
  {
    q = {0.0, 0.0, 0.0, 1.0};
    return;
  }

  q[0] /= norm;
  q[1] /= norm;
  q[2] /= norm;
  q[3] /= norm;
}

std::array<double, 4> Bno086ImuNode::MultiplyQuaternion(const std::array<double, 4>& lhs,
                                                        const std::array<double, 4>& rhs)
{
  return {
      lhs[3] * rhs[0] + lhs[0] * rhs[3] + lhs[1] * rhs[2] - lhs[2] * rhs[1],
      lhs[3] * rhs[1] - lhs[0] * rhs[2] + lhs[1] * rhs[3] + lhs[2] * rhs[0],
      lhs[3] * rhs[2] + lhs[0] * rhs[1] - lhs[1] * rhs[0] + lhs[2] * rhs[3],
      lhs[3] * rhs[3] - lhs[0] * rhs[0] - lhs[1] * rhs[1] - lhs[2] * rhs[2],
  };
}

std::array<double, 4> Bno086ImuNode::PredictOrientation(
    const std::array<double, 4>& orientation_xyzw,
    const OASIS::IMU::Vec3& gyro_rads,
    double prediction_horizon_sec)
{
  if (prediction_horizon_sec <= 0.0)
    return orientation_xyzw;

  const double angularSpeed = std::sqrt(gyro_rads[0] * gyro_rads[0] + gyro_rads[1] * gyro_rads[1] +
                                        gyro_rads[2] * gyro_rads[2]);
  if (angularSpeed <= 1e-9)
    return orientation_xyzw;

  const double halfAngle = 0.5 * angularSpeed * prediction_horizon_sec;
  const double sinHalfAngle = std::sin(halfAngle);
  const double invAngularSpeed = 1.0 / angularSpeed;
  std::array<double, 4> deltaQuaternion{
      gyro_rads[0] * invAngularSpeed * sinHalfAngle,
      gyro_rads[1] * invAngularSpeed * sinHalfAngle,
      gyro_rads[2] * invAngularSpeed * sinHalfAngle,
      std::cos(halfAngle),
  };

  std::array<double, 4> predictedOrientation =
      MultiplyQuaternion(orientation_xyzw, deltaQuaternion);
  NormalizeQuaternion(predictedOrientation);
  return predictedOrientation;
}

void Bno086ImuNode::SetCovariance(std::array<double, 9>& dst, const OASIS::IMU::Mat3& src)
{
  dst[0] = src[0][0];
  dst[1] = src[0][1];
  dst[2] = src[0][2];

  dst[3] = src[1][0];
  dst[4] = src[1][1];
  dst[5] = src[1][2];

  dst[6] = src[2][0];
  dst[7] = src[2][1];
  dst[8] = src[2][2];
}

std::optional<std::size_t> Bno086ImuNode::DiagnosticReportIndex(ReportId report_id)
{
  switch (report_id)
  {
    case ReportId::Accelerometer:
      return 0;
    case ReportId::GyroscopeCalibrated:
      return 1;
    case ReportId::RotationVector:
      return 2;
    case ReportId::LinearAcceleration:
      return 3;
    case ReportId::Gravity:
      return 4;
    default:
      break;
  }

  return std::nullopt;
}

void Bno086ImuNode::MaybeLogOrientationCovariancePolicy()
{
  const bool sourceChanged =
      !m_loggedOrientationCovarianceSource ||
      m_orientationCovarianceDebug.source != m_lastOrientationCovarianceSource;
  const bool bucketChanged =
      !m_loggedOrientationCovarianceSource ||
      m_orientationCovarianceDebug.accuracy_bucket != m_lastOrientationAccuracyBucket;

  if (sourceChanged || bucketChanged)
  {
    RCLCPP_INFO(get_logger(),
                "BNO086 orientation covariance policy: report=%s bucket=%u source=%s "
                "raw_estimate_q12=%d estimate_rad=%.4f rejection=%s sigma_rad=%.4f "
                "sigma_deg=%.2f",
                ORIENTATION_REPORT_SOURCE,
                static_cast<unsigned>(m_orientationCovarianceDebug.accuracy_bucket),
                OrientationCovarianceSourceName(m_orientationCovarianceDebug.source),
                static_cast<int>(m_orientationCovarianceDebug.raw_accuracy_estimate_q12),
                m_orientationCovarianceDebug.accuracy_estimate_rad,
                OrientationCovarianceEstimateRejectionReasonName(
                    m_orientationCovarianceDebug.rejection_reason),
                m_orientationCovarianceDebug.sigma_rad,
                m_orientationCovarianceDebug.sigma_rad * 180.0 / PI_RAD);

    m_loggedOrientationCovarianceSource = true;
    m_lastOrientationCovarianceSource = m_orientationCovarianceDebug.source;
    m_lastOrientationAccuracyBucket = m_orientationCovarianceDebug.accuracy_bucket;
  }

  RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), ORIENTATION_COVARIANCE_LOG_THROTTLE_MS,
                        "BNO086 orientation covariance status: report=%s bucket=%u source=%s "
                        "raw_estimate_q12=%d estimate_rad=%.4f rejection=%s sigma_rad=%.4f "
                        "sigma_deg=%.2f variance_rad2=%.6f",
                        ORIENTATION_REPORT_SOURCE,
                        static_cast<unsigned>(m_orientationCovarianceDebug.accuracy_bucket),
                        OrientationCovarianceSourceName(m_orientationCovarianceDebug.source),
                        static_cast<int>(m_orientationCovarianceDebug.raw_accuracy_estimate_q12),
                        m_orientationCovarianceDebug.accuracy_estimate_rad,
                        OrientationCovarianceEstimateRejectionReasonName(
                            m_orientationCovarianceDebug.rejection_reason),
                        m_orientationCovarianceDebug.sigma_rad,
                        m_orientationCovarianceDebug.sigma_rad * 180.0 / PI_RAD,
                        m_latestFrame.orientation_cov_rad2[0][0]);
}
