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
constexpr int DEFAULT_MAX_NO_PROGRESS_POLLS_PER_INTERRUPT = 64;
constexpr int MIN_MAX_NO_PROGRESS_POLLS_PER_INTERRUPT = 1;
constexpr int MAX_MAX_NO_PROGRESS_POLLS_PER_INTERRUPT = 1024;
constexpr int DEFAULT_MAX_DRAIN_DURATION_MS = 100;
constexpr int MIN_MAX_DRAIN_DURATION_MS = 1;
constexpr int MAX_MAX_DRAIN_DURATION_MS = 1000;
constexpr int DEFAULT_MAX_SENSOR_EVENTS_PER_DRAIN = 4096;
constexpr int MIN_MAX_SENSOR_EVENTS_PER_DRAIN = 1;
constexpr int MAX_MAX_SENSOR_EVENTS_PER_DRAIN = 16384;
constexpr int DEFAULT_MAX_PENDING_EVENTS_FLUSH_PER_DRAIN = 1024;
constexpr int MIN_MAX_PENDING_EVENTS_FLUSH_PER_DRAIN = 1;
constexpr int MAX_MAX_PENDING_EVENTS_FLUSH_PER_DRAIN = 16384;
constexpr std::uint32_t REPEATED_NO_PROGRESS_TIMEOUT_WARN_COUNT = 3;

constexpr std::uint32_t MIN_COHERENT_SAMPLE_SPAN_US = 50'000;
constexpr double DEFAULT_PREDICTION_HORIZON_SEC = 0.0;

constexpr double DEFAULT_ROTATION_VECTOR_RATE_HZ = 50.0;
constexpr double DEFAULT_GYRO_RATE_HZ = 50.0;
constexpr double DEFAULT_ACCELEROMETER_RATE_HZ = 100.0;
constexpr double DEFAULT_LINEAR_ACCELERATION_RATE_HZ = 50.0;
constexpr double DEFAULT_GRAVITY_RATE_HZ = 25.0;
constexpr double DEFAULT_ROTATION_VECTOR_BATCH_MS = 50.0;
constexpr double DEFAULT_GYRO_BATCH_MS = 50.0;
constexpr double DEFAULT_ACCELEROMETER_BATCH_MS = 50.0;
constexpr double DEFAULT_LINEAR_ACCELERATION_BATCH_MS = 50.0;
constexpr double DEFAULT_GRAVITY_BATCH_MS = 100.0;
constexpr bool DEFAULT_ENABLE_LINEAR_ACCELERATION_REPORT = true;
constexpr bool DEFAULT_ENABLE_GRAVITY_REPORT = true;
constexpr int DEFAULT_FEATURE_SUMMARY_TIMEOUT_MS = 5'000;
constexpr int DEFAULT_BNO086_TIMESTAMP_TRACE_COUNT = 0;
constexpr int MAX_BNO086_TIMESTAMP_TRACE_COUNT = 1'000;

// Maximum nearby orientation age accepted for imu_gravity composition
constexpr double DEFAULT_IMU_GRAVITY_MAX_ORIENTATION_AGE_MS = 50.0;

// Maximum nearby gyro age accepted for imu_gravity composition
constexpr double DEFAULT_IMU_GRAVITY_MAX_GYRO_AGE_MS = 50.0;

// Plausibility bound for gravity-included calibrated acceleration samples
constexpr double MAX_IMU_GRAVITY_ACCEL_MAGNITUDE_MPS2 = 200.0;

// Plausibility bound for calibrated gyro samples
constexpr double MAX_IMU_GRAVITY_GYRO_MAGNITUDE_RADS = 100.0;
constexpr double PI_RAD = 3.14159265358979323846;
constexpr const char* ORIENTATION_REPORT_SOURCE = "rotation_vector";
constexpr int ORIENTATION_COVARIANCE_LOG_THROTTLE_MS = 5'000;
constexpr int IMU_GRAVITY_SAMPLE_WARN_THROTTLE_MS = 5'000;
constexpr int DEFAULT_BNO086_DIAGNOSTICS_LOG_PERIOD_MS = 5'000;
constexpr int MIN_BNO086_DIAGNOSTICS_LOG_PERIOD_MS = 1'000;
constexpr double MIN_HEALTHY_RATE_FRACTION = 0.5;

// Units: ns. SH-2 delay values above this are not plausible sample latency
constexpr int64_t MAX_PLAUSIBLE_SAMPLE_DELAY_NS = 50'000'000;

// QoS parameters
constexpr std::size_t RAW_IMU_GRAVITY_QOS_DEPTH = 256;
constexpr std::size_t RAW_IMU_QOS_DEPTH = 10;
constexpr std::size_t RAW_GRAVITY_QOS_DEPTH = 32;

rclcpp::QoS BestEffortSensorQos(std::size_t depth)
{
  rclcpp::QoS qos{rclcpp::KeepLast(depth)};
  qos.best_effort();
  qos.durability_volatile();
  return qos;
}

rclcpp::QoS ReliableSensorQos(std::size_t depth)
{
  rclcpp::QoS qos{rclcpp::KeepLast(depth)};
  qos.reliable();
  qos.durability_volatile();
  return qos;
}

bool IsFiniteArray(const std::array<double, 9>& values)
{
  return std::all_of(values.begin(), values.end(),
                     [](double value) { return std::isfinite(value); });
}

int64_t HostAnchorNs(const SensorEvent& event, int64_t packet_host_stamp_ns)
{
  if (!event.has_delay)
    return packet_host_stamp_ns;

  const int64_t delayNs = static_cast<int64_t>(event.delay_us) * 1'000;
  if (delayNs < 0 || delayNs > MAX_PLAUSIBLE_SAMPLE_DELAY_NS)
    return packet_host_stamp_ns;

  return packet_host_stamp_ns - delayNs;
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
  declare_parameter("bno086_max_no_progress_polls_per_interrupt",
                    DEFAULT_MAX_NO_PROGRESS_POLLS_PER_INTERRUPT);
  declare_parameter("bno086_max_drain_duration_ms", DEFAULT_MAX_DRAIN_DURATION_MS);
  declare_parameter("bno086_max_sensor_events_per_drain", DEFAULT_MAX_SENSOR_EVENTS_PER_DRAIN);
  declare_parameter("bno086_max_pending_events_flush_per_drain",
                    DEFAULT_MAX_PENDING_EVENTS_FLUSH_PER_DRAIN);
  declare_parameter("bno086_diagnostics_log_period_ms", DEFAULT_BNO086_DIAGNOSTICS_LOG_PERIOD_MS);
  declare_parameter("bno086_timestamp_trace_count", DEFAULT_BNO086_TIMESTAMP_TRACE_COUNT);
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
  m_maxNoProgressPollsPerInterrupt = static_cast<std::uint32_t>(std::clamp(
      static_cast<int>(get_parameter("bno086_max_no_progress_polls_per_interrupt").as_int()),
      MIN_MAX_NO_PROGRESS_POLLS_PER_INTERRUPT, MAX_MAX_NO_PROGRESS_POLLS_PER_INTERRUPT));
  m_maxDrainDurationMs =
      std::clamp(static_cast<int>(get_parameter("bno086_max_drain_duration_ms").as_int()),
                 MIN_MAX_DRAIN_DURATION_MS, MAX_MAX_DRAIN_DURATION_MS);
  m_maxSensorEventsPerDrain = static_cast<std::uint32_t>(
      std::clamp(static_cast<int>(get_parameter("bno086_max_sensor_events_per_drain").as_int()),
                 MIN_MAX_SENSOR_EVENTS_PER_DRAIN, MAX_MAX_SENSOR_EVENTS_PER_DRAIN));
  m_maxPendingEventsFlushPerDrain = static_cast<std::uint32_t>(std::clamp(
      static_cast<int>(get_parameter("bno086_max_pending_events_flush_per_drain").as_int()),
      MIN_MAX_PENDING_EVENTS_FLUSH_PER_DRAIN, MAX_MAX_PENDING_EVENTS_FLUSH_PER_DRAIN));
  m_diagnosticsLogPeriodMs =
      std::max(static_cast<int>(get_parameter("bno086_diagnostics_log_period_ms").as_int()),
               MIN_BNO086_DIAGNOSTICS_LOG_PERIOD_MS);
  m_timestampTraceCount = static_cast<std::uint32_t>(
      std::clamp(static_cast<int>(get_parameter("bno086_timestamp_trace_count").as_int()), 0,
                 MAX_BNO086_TIMESTAMP_TRACE_COUNT));
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
  RCLCPP_INFO(get_logger(), "BNO086 diagnostics period_ms=%d", m_diagnosticsLogPeriodMs);
  RCLCPP_INFO(get_logger(), "Predicted orientation output uses %s with prediction_horizon_sec=%.4f",
              m_predictionSource.c_str(), m_predictionHorizonSec);
  RCLCPP_INFO(get_logger(),
              "BNO086 imu_gravity uses calibrated acceleration cadence with "
              "orientation_max_age_ms=%.1f gyro_max_age_ms=%.1f",
              m_imuGravityMaxOrientationAgeMs, m_imuGravityMaxGyroAgeMs);
}

bool Bno086ImuNode::Initialize()
{
  for (Bno086ReportTimestampTracker& tracker : m_timestampTrackers)
    tracker.Reset();
  m_bnoCadenceEpochNs.reset();
  m_lastEmittedTimestampNs.fill(std::nullopt);
  m_lastPublishedCoreSignature.reset();
  m_lastPublishedImuGravityAccelStampNs.reset();
  m_drainDiagnostics = DrainThroughputDiagnostics{};

  m_imuPublisher =
      create_publisher<sensor_msgs::msg::Imu>(IMU_TOPIC, BestEffortSensorQos(RAW_IMU_QOS_DEPTH));
  m_imuPredictedPublisher = create_publisher<sensor_msgs::msg::Imu>(
      IMU_PREDICTED_TOPIC, BestEffortSensorQos(RAW_IMU_QOS_DEPTH));
  m_imuVrPublisher = create_publisher<oasis_msgs::msg::ImuVr>(
      IMU_VR_TOPIC, BestEffortSensorQos(RAW_IMU_QOS_DEPTH));
  m_gravityPublisher = create_publisher<geometry_msgs::msg::AccelWithCovarianceStamped>(
      GRAVITY_TOPIC, ReliableSensorQos(RAW_GRAVITY_QOS_DEPTH));
  m_imuGravityPublisher = create_publisher<sensor_msgs::msg::Imu>(
      IMU_GRAVITY_TOPIC, ReliableSensorQos(RAW_IMU_GRAVITY_QOS_DEPTH));

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
  drainLimits.max_no_progress_polls_per_interrupt = m_maxNoProgressPollsPerInterrupt;
  drainLimits.max_sensor_events_per_drain = m_maxSensorEventsPerDrain;
  drainLimits.max_pending_events_flush_per_drain = m_maxPendingEventsFlushPerDrain;
  const rclcpp::Time drainReceiveAnchor = interrupt_ros_at;
  const auto drainStartedAt = std::chrono::steady_clock::now();
  Bno086DrainAction drainExitAction = Bno086DrainAction::Complete;

  while (m_running.load())
  {
    const bool hintnAssertedBeforePoll = m_interruptGpio.IsAssertedLow();
    const Bno086DrainDecision beforePollDecision =
        Bno086DrainBeforePoll(drainLimits, drainCounters, hintnAssertedBeforePoll);
    if (beforePollDecision.action == Bno086DrainAction::PollIterationCap)
    {
      const auto drainDurationUs =
          static_cast<std::uint32_t>(std::chrono::duration_cast<std::chrono::microseconds>(
                                         std::chrono::steady_clock::now() - drainStartedAt)
                                         .count());
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "BNO086 interrupt drain hit poll iteration cap: "
          "physical_packets_this_drain=%u sensor_events_this_drain=%u "
          "pending_events_this_drain=%u control_packets_this_drain=%u "
          "poll_iterations=%u consecutive_no_progress_polls=%u "
          "packet_cap=%u poll_iteration_cap=%u no_progress_budget=%u "
          "drain_duration_ms=%.3f hintn_asserted=%s",
          drainCounters.physical_packets_this_drain, drainCounters.sensor_events_this_drain,
          drainCounters.pending_events_this_drain, drainCounters.control_packets_this_drain,
          drainCounters.poll_iterations, drainCounters.consecutive_no_progress_polls,
          drainLimits.max_physical_packets_per_interrupt,
          drainLimits.max_poll_iterations_per_interrupt,
          drainLimits.max_no_progress_polls_per_interrupt,
          static_cast<double>(drainDurationUs) / 1e3,
          beforePollDecision.hintn_asserted ? "true" : "false");
      drainExitAction = beforePollDecision.action;
      break;
    }

    const auto elapsedBeforePollMs = std::chrono::duration_cast<std::chrono::milliseconds>(
                                         std::chrono::steady_clock::now() - drainStartedAt)
                                         .count();
    const std::size_t pendingEventsBeforePoll = m_shtp->PendingEventCount();
    if (pendingEventsBeforePoll == 0 && drainCounters.poll_iterations > 0 &&
        elapsedBeforePollMs >= m_maxDrainDurationMs)
    {
      RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 2000,
                            "BNO086 interrupt drain hit duration budget: "
                            "physical_packets_this_drain=%u sensor_events_this_drain=%u "
                            "pending_events_this_drain=%u poll_iterations=%u "
                            "drain_duration_ms=%lld duration_budget_ms=%d hintn_asserted=%s",
                            drainCounters.physical_packets_this_drain,
                            drainCounters.sensor_events_this_drain,
                            drainCounters.pending_events_this_drain, drainCounters.poll_iterations,
                            static_cast<long long>(elapsedBeforePollMs), m_maxDrainDurationMs,
                            hintnAssertedBeforePoll ? "true" : "false");
      drainExitAction = Bno086DrainAction::DrainDurationBudget;
      break;
    }

    const Bno086Shtp::PollResult pollResult = m_shtp->Poll(m_packetReadTimeoutMs);
    const bool hintnAssertedAfterPoll = m_interruptGpio.IsAssertedLow();
    const Bno086DrainDecision afterPollDecision =
        Bno086DrainAfterPoll(pollResult, drainLimits, drainCounters, hintnAssertedAfterPoll);
    if (pollResult.packet_channel.has_value() &&
        *pollResult.packet_channel < m_drainDiagnostics.channel_packet_counts.size())
    {
      ++m_drainDiagnostics.channel_packet_counts[*pollResult.packet_channel];
    }

    if (afterPollDecision.action == Bno086DrainAction::Complete)
    {
      m_repeatedNoProgressTimeouts = 0;
      drainExitAction = afterPollDecision.action;
      break;
    }

    if (afterPollDecision.action == Bno086DrainAction::NoProgressBudget)
    {
      ++m_repeatedNoProgressTimeouts;
      const auto drainDurationUs =
          static_cast<std::uint32_t>(std::chrono::duration_cast<std::chrono::microseconds>(
                                         std::chrono::steady_clock::now() - drainStartedAt)
                                         .count());
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "BNO086 interrupt drain exhausted no-progress budget while H_INTN "
          "remained asserted: physical_packets_this_drain=%u "
          "sensor_events_this_drain=%u pending_events_this_drain=%u "
          "control_packets_this_drain=%u poll_iterations=%u "
          "consecutive_no_progress_polls=%u no_progress_budget=%u "
          "packet_cap=%u poll_iteration_cap=%u drain_duration_ms=%.3f "
          "hintn_asserted=true",
          drainCounters.physical_packets_this_drain, drainCounters.sensor_events_this_drain,
          drainCounters.pending_events_this_drain, drainCounters.control_packets_this_drain,
          drainCounters.poll_iterations, drainCounters.consecutive_no_progress_polls,
          drainLimits.max_no_progress_polls_per_interrupt,
          drainLimits.max_physical_packets_per_interrupt,
          drainLimits.max_poll_iterations_per_interrupt,
          static_cast<double>(drainDurationUs) / 1e3);
      drainExitAction = afterPollDecision.action;
      break;
    }

    if (pollResult.read_physical_packet || pollResult.event.has_value() ||
        pollResult.dequeued_pending_event || pollResult.handled_control_packet)
    {
      m_repeatedNoProgressTimeouts = 0;
    }

    if (afterPollDecision.action == Bno086DrainAction::TransportError)
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "BNO086 transport error while draining interrupt data");
      drainExitAction = afterPollDecision.action;
      break;
    }

    MaybeLogFeatureResponses();

    if (pollResult.status == Bno086Shtp::PollStatus::SensorEvent && pollResult.event.has_value())
    {
      // For batched SHTP reports, host drain time is transport latency. Use
      // one stable receive anchor for the interrupt drain so reports in a
      // batch are not timestamped later merely because they were read later
      // over I2C.
      const std::optional<std::uint32_t> expectedIntervalUs =
          EffectiveReportIntervalUs(pollResult.event->report_id);
      const std::optional<int64_t> expectedIntervalNs =
          expectedIntervalUs.has_value()
              ? std::make_optional(static_cast<int64_t>(*expectedIntervalUs) * 1'000)
              : std::nullopt;
      const EstimatedEventStamp eventStamp =
          EstimateEventStamp(*pollResult.event, drainReceiveAnchor, expectedIntervalNs);
      const TimestampNormalizationResult normalizedStamp = FinalizeEventStamp(
          *pollResult.event, eventStamp, drainReceiveAnchor.nanoseconds(), expectedIntervalNs);
      RecordTimestampTrace(*pollResult.event, eventStamp.stamp.nanoseconds(),
                           drainReceiveAnchor.nanoseconds(), normalizedStamp, eventStamp.tracker,
                           expectedIntervalUs);

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
        const auto drainDurationUs =
            static_cast<std::uint32_t>(std::chrono::duration_cast<std::chrono::microseconds>(
                                           std::chrono::steady_clock::now() - drainStartedAt)
                                           .count());
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "BNO086 interrupt drain hit physical packet cap while H_INTN "
            "remained asserted: physical_packets_this_drain=%u "
            "sensor_events_this_drain=%u pending_events_this_drain=%u "
            "control_packets_this_drain=%u poll_iterations=%u "
            "consecutive_no_progress_polls=%u packet_cap=%u "
            "poll_iteration_cap=%u no_progress_budget=%u drain_duration_ms=%.3f "
            "hintn_asserted=true",
            drainCounters.physical_packets_this_drain, drainCounters.sensor_events_this_drain,
            drainCounters.pending_events_this_drain, drainCounters.control_packets_this_drain,
            drainCounters.poll_iterations, drainCounters.consecutive_no_progress_polls,
            drainLimits.max_physical_packets_per_interrupt,
            drainLimits.max_poll_iterations_per_interrupt,
            drainLimits.max_no_progress_polls_per_interrupt,
            static_cast<double>(drainDurationUs) / 1e3);
      }
      drainExitAction = afterPollDecision.action;
      break;
    }

    if (afterPollDecision.action == Bno086DrainAction::SensorEventBudget)
    {
      RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 2000,
                            "BNO086 interrupt drain hit sensor event budget: "
                            "physical_packets_this_drain=%u sensor_events_this_drain=%u "
                            "pending_events_this_drain=%u poll_iterations=%u "
                            "sensor_event_budget=%u hintn_asserted=%s",
                            drainCounters.physical_packets_this_drain,
                            drainCounters.sensor_events_this_drain,
                            drainCounters.pending_events_this_drain, drainCounters.poll_iterations,
                            drainLimits.max_sensor_events_per_drain,
                            afterPollDecision.hintn_asserted ? "true" : "false");
      drainExitAction = afterPollDecision.action;
      break;
    }

    if (afterPollDecision.action == Bno086DrainAction::PendingEventFlushBudget)
    {
      RCLCPP_DEBUG_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "BNO086 interrupt drain hit pending event flush budget: "
          "physical_packets_this_drain=%u sensor_events_this_drain=%u "
          "pending_events_this_drain=%u poll_iterations=%u "
          "pending_flush_budget=%u pending_queue_depth=%zu hintn_asserted=%s",
          drainCounters.physical_packets_this_drain, drainCounters.sensor_events_this_drain,
          drainCounters.pending_events_this_drain, drainCounters.poll_iterations,
          drainLimits.max_pending_events_flush_per_drain, m_shtp->PendingEventCount(),
          afterPollDecision.hintn_asserted ? "true" : "false");
      drainExitAction = afterPollDecision.action;
      break;
    }

    const auto elapsedAfterPollMs = std::chrono::duration_cast<std::chrono::milliseconds>(
                                        std::chrono::steady_clock::now() - drainStartedAt)
                                        .count();
    if (m_shtp->PendingEventCount() == 0 && elapsedAfterPollMs >= m_maxDrainDurationMs)
    {
      RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 2000,
                            "BNO086 interrupt drain hit duration budget: "
                            "physical_packets_this_drain=%u sensor_events_this_drain=%u "
                            "pending_events_this_drain=%u poll_iterations=%u "
                            "drain_duration_ms=%lld duration_budget_ms=%d hintn_asserted=%s",
                            drainCounters.physical_packets_this_drain,
                            drainCounters.sensor_events_this_drain,
                            drainCounters.pending_events_this_drain, drainCounters.poll_iterations,
                            static_cast<long long>(elapsedAfterPollMs), m_maxDrainDurationMs,
                            m_interruptGpio.IsAssertedLow() ? "true" : "false");
      drainExitAction = Bno086DrainAction::DrainDurationBudget;
      break;
    }
  }

  const bool hintnAssertedAfterExit = m_interruptGpio.IsAssertedLow();
  const std::uint32_t pendingQueueDepthAtExit =
      static_cast<std::uint32_t>(std::min<std::size_t>(m_shtp->PendingEventCount(), UINT32_MAX));
  const auto drainDurationUs =
      static_cast<std::uint32_t>(std::chrono::duration_cast<std::chrono::microseconds>(
                                     std::chrono::steady_clock::now() - drainStartedAt)
                                     .count());
  RecordDrainThroughputDiagnostics(drainCounters, drainExitAction, hintnAssertedAfterExit,
                                   drainDurationUs, pendingQueueDepthAtExit);
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

  const int64_t coreThresholdNs = DurationFromUs(CoreCoherenceToleranceUs()).nanoseconds();
  if (!IsTimestampSpanCoherent(oldestNs, newestNs, coreThresholdNs))
  {
    ++m_imuGravityDiagnostics.imu_skipped_incoherent_core_frame;
    ++m_imuGravityDiagnostics.imu_skipped_incoherent_core_frame_span;
    RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 2000,
                          "Skipping BNO086 imu core frame: timestamp span too wide "
                          "latest_core_span_ms=%.3f threshold_ms=%.3f "
                          "orientation_ns=%lld gyro_ns=%lld linear_accel_ns=%lld",
                          m_imuGravityDiagnostics.latest_core_span_ms,
                          static_cast<double>(coreThresholdNs) / 1.0e6,
                          static_cast<long long>(orientationNs), static_cast<long long>(gyroNs),
                          static_cast<long long>(linearAccelNs));
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
  const int64_t maxOrientationAgeNs = ImuGravityMaxOrientationAgeNs();
  const int64_t maxGyroAgeNs = ImuGravityMaxGyroAgeNs();
  const SampleFreshnessResult orientationFreshness = EvaluateSampleFreshness(
      accelStampNs, m_orientationState.stamp.nanoseconds(), maxOrientationAgeNs,
      ReportFutureToleranceNs(ReportId::RotationVector));
  const SampleFreshnessResult gyroFreshness =
      EvaluateSampleFreshness(accelStampNs, m_gyroState.stamp.nanoseconds(), maxGyroAgeNs,
                              ReportFutureToleranceNs(ReportId::GyroscopeCalibrated));
  m_imuGravityDiagnostics.latest_orientation_age_ms =
      static_cast<double>(orientationFreshness.age_ns) / 1.0e6;
  m_imuGravityDiagnostics.latest_gyro_age_ms = static_cast<double>(gyroFreshness.age_ns) / 1.0e6;

  if (orientationFreshness.status == SampleFreshnessStatus::TooOld)
  {
    ++m_imuGravityDiagnostics.imu_gravity_skipped_stale_orientation;
    RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 2000,
                          "Skipping BNO086 imu_gravity sample: orientation older than max age "
                          "age_ms=%.3f max_age_ms=%.3f accel_ns=%lld orientation_ns=%lld",
                          m_imuGravityDiagnostics.latest_orientation_age_ms,
                          static_cast<double>(maxOrientationAgeNs) / 1.0e6,
                          static_cast<long long>(accelStampNs),
                          static_cast<long long>(m_orientationState.stamp.nanoseconds()));
    MaybeLogImuGravityDiagnostics();
    return;
  }

  if (orientationFreshness.status == SampleFreshnessStatus::TooFuture)
  {
    ++m_imuGravityDiagnostics.imu_gravity_skipped_future_orientation;
    RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 2000,
                          "Skipping BNO086 imu_gravity sample: orientation too far after accel "
                          "age_ms=%.3f future_tolerance_ms=%.3f accel_ns=%lld orientation_ns=%lld",
                          m_imuGravityDiagnostics.latest_orientation_age_ms,
                          static_cast<double>(ReportFutureToleranceNs(ReportId::RotationVector)) /
                              1.0e6,
                          static_cast<long long>(accelStampNs),
                          static_cast<long long>(m_orientationState.stamp.nanoseconds()));
    MaybeLogImuGravityDiagnostics();
    return;
  }

  if (gyroFreshness.status == SampleFreshnessStatus::TooOld)
  {
    ++m_imuGravityDiagnostics.imu_gravity_skipped_stale_gyro;
    RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 2000,
                          "Skipping BNO086 imu_gravity sample: gyro older than max age "
                          "age_ms=%.3f max_age_ms=%.3f accel_ns=%lld gyro_ns=%lld",
                          m_imuGravityDiagnostics.latest_gyro_age_ms,
                          static_cast<double>(maxGyroAgeNs) / 1.0e6,
                          static_cast<long long>(accelStampNs),
                          static_cast<long long>(m_gyroState.stamp.nanoseconds()));
    MaybeLogImuGravityDiagnostics();
    return;
  }

  if (gyroFreshness.status == SampleFreshnessStatus::TooFuture)
  {
    ++m_imuGravityDiagnostics.imu_gravity_skipped_future_gyro;
    RCLCPP_DEBUG_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Skipping BNO086 imu_gravity sample: gyro too far after accel "
        "age_ms=%.3f future_tolerance_ms=%.3f accel_ns=%lld gyro_ns=%lld",
        m_imuGravityDiagnostics.latest_gyro_age_ms,
        static_cast<double>(ReportFutureToleranceNs(ReportId::GyroscopeCalibrated)) / 1.0e6,
        static_cast<long long>(accelStampNs),
        static_cast<long long>(m_gyroState.stamp.nanoseconds()));
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
  MaybeEmitTimestampSummaries();
  m_imuGravityDiagnostics.last_log_at = now;
}

void Bno086ImuNode::RecordDrainThroughputDiagnostics(const Bno086DrainCounters& counters,
                                                     Bno086DrainAction exit_action,
                                                     bool hintn_asserted_after_exit,
                                                     std::uint32_t drain_duration_us,
                                                     std::uint32_t pending_queue_depth_at_exit)
{
  ++m_drainDiagnostics.drains;

  m_drainDiagnostics.physical_packets_sum += counters.physical_packets_this_drain;
  m_drainDiagnostics.physical_packets_max =
      std::max(m_drainDiagnostics.physical_packets_max, counters.physical_packets_this_drain);
  if (!m_drainDiagnostics.physical_packets_min.has_value() ||
      counters.physical_packets_this_drain < *m_drainDiagnostics.physical_packets_min)
  {
    m_drainDiagnostics.physical_packets_min = counters.physical_packets_this_drain;
  }

  m_drainDiagnostics.sensor_events_sum += counters.sensor_events_this_drain;
  m_drainDiagnostics.sensor_events_max =
      std::max(m_drainDiagnostics.sensor_events_max, counters.sensor_events_this_drain);
  if (!m_drainDiagnostics.sensor_events_min.has_value() ||
      counters.sensor_events_this_drain < *m_drainDiagnostics.sensor_events_min)
  {
    m_drainDiagnostics.sensor_events_min = counters.sensor_events_this_drain;
  }

  m_drainDiagnostics.pending_events_sum += counters.pending_events_this_drain;
  m_drainDiagnostics.pending_events_max =
      std::max(m_drainDiagnostics.pending_events_max, counters.pending_events_this_drain);
  m_drainDiagnostics.pending_queue_depth_at_exit = pending_queue_depth_at_exit;
  m_drainDiagnostics.pending_queue_depth_max =
      std::max(m_drainDiagnostics.pending_queue_depth_max, pending_queue_depth_at_exit);

  m_drainDiagnostics.drain_duration_sum_us += drain_duration_us;
  m_drainDiagnostics.drain_duration_max_us =
      std::max(m_drainDiagnostics.drain_duration_max_us, drain_duration_us);
  if (!m_drainDiagnostics.drain_duration_min_us.has_value() ||
      drain_duration_us < *m_drainDiagnostics.drain_duration_min_us)
  {
    m_drainDiagnostics.drain_duration_min_us = drain_duration_us;
  }

  if (exit_action == Bno086DrainAction::PhysicalPacketCap)
  {
    ++m_drainDiagnostics.physical_packet_cap_hit_count;
    ++m_drainDiagnostics.exit_packet_cap_count;
  }

  if (exit_action == Bno086DrainAction::PollIterationCap)
  {
    ++m_drainDiagnostics.poll_iteration_cap_hit_count;
    ++m_drainDiagnostics.exit_poll_iteration_cap_count;
  }

  if (exit_action == Bno086DrainAction::DrainDurationBudget)
  {
    ++m_drainDiagnostics.drain_duration_budget_hit_count;
    ++m_drainDiagnostics.exit_drain_duration_budget_count;
    if (pending_queue_depth_at_exit > 0)
      ++m_drainDiagnostics.exit_duration_budget_with_pending_count;
    else
      ++m_drainDiagnostics.exit_duration_budget_no_pending_count;
  }

  if (exit_action == Bno086DrainAction::SensorEventBudget)
  {
    ++m_drainDiagnostics.sensor_event_budget_hit_count;
    ++m_drainDiagnostics.exit_sensor_event_budget_count;
  }

  if (exit_action == Bno086DrainAction::PendingEventFlushBudget)
  {
    ++m_drainDiagnostics.pending_flush_budget_hit_count;
    ++m_drainDiagnostics.exit_pending_event_flush_budget_count;
  }

  if (exit_action == Bno086DrainAction::NoProgressBudget)
  {
    ++m_drainDiagnostics.no_progress_drain_count;
    ++m_drainDiagnostics.exit_no_progress_budget_count;
  }

  if (exit_action == Bno086DrainAction::TransportError)
  {
    ++m_drainDiagnostics.transport_error_count;
    ++m_drainDiagnostics.exit_transport_error_count;
  }

  if (exit_action == Bno086DrainAction::Complete && !hintn_asserted_after_exit)
    ++m_drainDiagnostics.complete_int_deasserted_count;

  if (hintn_asserted_after_exit)
  {
    ++m_drainDiagnostics.hintn_asserted_after_exit_count;
    if (exit_action == Bno086DrainAction::DrainDurationBudget ||
        exit_action == Bno086DrainAction::SensorEventBudget ||
        exit_action == Bno086DrainAction::PendingEventFlushBudget)
    {
      ++m_drainDiagnostics.hintn_asserted_after_cooperative_budget_exit_count;
    }
    else
    {
      ++m_drainDiagnostics.hintn_asserted_after_safety_or_error_exit_count;
    }
  }
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
  const std::string message = BuildImuGravityDiagnosticsLogMessage();
  RCLCPP_DEBUG(get_logger(), "%s", message.c_str());

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
  const double drainCount = static_cast<double>(m_drainDiagnostics.drains);
  const double physicalPacketsMean =
      m_drainDiagnostics.drains > 0
          ? static_cast<double>(m_drainDiagnostics.physical_packets_sum) / drainCount
          : 0.0;
  const double sensorEventsMean =
      m_drainDiagnostics.drains > 0
          ? static_cast<double>(m_drainDiagnostics.sensor_events_sum) / drainCount
          : 0.0;
  const double pendingEventsMean =
      m_drainDiagnostics.drains > 0
          ? static_cast<double>(m_drainDiagnostics.pending_events_sum) / drainCount
          : 0.0;
  const double drainDurationMeanMs =
      m_drainDiagnostics.drains > 0
          ? static_cast<double>(m_drainDiagnostics.drain_duration_sum_us) / drainCount / 1.0e3
          : 0.0;
  const Bno086TransportStats transportStats = m_transport.GetStats();

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
      << "skipped_future_orientation="
      << m_imuGravityDiagnostics.imu_gravity_skipped_future_orientation << " "
      << "skipped_future_gyro=" << m_imuGravityDiagnostics.imu_gravity_skipped_future_gyro << " "
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
      << "imu_skipped_incoherent_core_frame_span="
      << m_imuGravityDiagnostics.imu_skipped_incoherent_core_frame_span << " "
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
      << "decoded_gravity_rate_hz=" << m_imuGravityDiagnostics.latest_decoded_rate_hz[4] << " "
      << "drains=" << m_drainDiagnostics.drains << " "
      << "physical_packets_per_drain_min=" << m_drainDiagnostics.physical_packets_min.value_or(0)
      << " " << "physical_packets_per_drain_mean=" << physicalPacketsMean << " "
      << "physical_packets_per_drain_max=" << m_drainDiagnostics.physical_packets_max << " "
      << "sensor_events_per_drain_min=" << m_drainDiagnostics.sensor_events_min.value_or(0) << " "
      << "sensor_events_per_drain_mean=" << sensorEventsMean << " "
      << "sensor_events_per_drain_max=" << m_drainDiagnostics.sensor_events_max << " "
      << "pending_events_per_drain_mean=" << pendingEventsMean << " "
      << "pending_events_per_drain_max=" << m_drainDiagnostics.pending_events_max << " "
      << "pending_queue_depth_at_exit=" << m_drainDiagnostics.pending_queue_depth_at_exit << " "
      << "pending_queue_depth_max=" << m_drainDiagnostics.pending_queue_depth_max << " "
      << "drain_duration_ms_min="
      << static_cast<double>(m_drainDiagnostics.drain_duration_min_us.value_or(0)) / 1.0e3 << " "
      << "drain_duration_ms_mean=" << drainDurationMeanMs << " " << "drain_duration_ms_max="
      << static_cast<double>(m_drainDiagnostics.drain_duration_max_us) / 1.0e3 << " "
      << "physical_packet_cap_hit_count=" << m_drainDiagnostics.physical_packet_cap_hit_count << " "
      << "poll_iteration_cap_hit_count=" << m_drainDiagnostics.poll_iteration_cap_hit_count << " "
      << "drain_duration_budget_hit_count=" << m_drainDiagnostics.drain_duration_budget_hit_count
      << " " << "sensor_event_budget_hit_count=" << m_drainDiagnostics.sensor_event_budget_hit_count
      << " "
      << "pending_flush_budget_hit_count=" << m_drainDiagnostics.pending_flush_budget_hit_count
      << " " << "exit_duration_budget_with_pending_count="
      << m_drainDiagnostics.exit_duration_budget_with_pending_count << " "
      << "exit_duration_budget_no_pending_count="
      << m_drainDiagnostics.exit_duration_budget_no_pending_count << " "
      << "hintn_asserted_after_drain_exit_count="
      << m_drainDiagnostics.hintn_asserted_after_exit_count << " "
      << "hintn_asserted_after_cooperative_budget_exit_count="
      << m_drainDiagnostics.hintn_asserted_after_cooperative_budget_exit_count << " "
      << "hintn_asserted_after_safety_or_error_exit_count="
      << m_drainDiagnostics.hintn_asserted_after_safety_or_error_exit_count << " "
      << "no_progress_drain_count=" << m_drainDiagnostics.no_progress_drain_count << " "
      << "drain_transport_error_count=" << m_drainDiagnostics.transport_error_count << " "
      << "complete_int_deasserted_count=" << m_drainDiagnostics.complete_int_deasserted_count << " "
      << "exit_no_progress_budget_count=" << m_drainDiagnostics.exit_no_progress_budget_count << " "
      << "exit_transport_error_count=" << m_drainDiagnostics.exit_transport_error_count << " "
      << "exit_packet_cap_count=" << m_drainDiagnostics.exit_packet_cap_count << " "
      << "exit_poll_iteration_cap_count=" << m_drainDiagnostics.exit_poll_iteration_cap_count << " "
      << "exit_drain_duration_budget_count=" << m_drainDiagnostics.exit_drain_duration_budget_count
      << " "
      << "exit_sensor_event_budget_count=" << m_drainDiagnostics.exit_sensor_event_budget_count
      << " " << "exit_pending_event_flush_budget_count="
      << m_drainDiagnostics.exit_pending_event_flush_budget_count << " "
      << "i2c_read_error_count=" << transportStats.i2c_read_error_count << " "
      << "i2c_read_timeout_count=" << transportStats.i2c_read_timeout_count << " "
      << "invalid_header_count=" << transportStats.invalid_header_count << " "
      << "all_zero_header_count=" << transportStats.all_zero_header_count << " "
      << "invalid_full_packet_count=" << transportStats.invalid_full_packet_count << " "
      << "channel0_packets=" << m_drainDiagnostics.channel_packet_counts[0] << " "
      << "channel1_packets=" << m_drainDiagnostics.channel_packet_counts[1] << " "
      << "channel2_packets=" << m_drainDiagnostics.channel_packet_counts[2] << " "
      << "channel3_packets=" << m_drainDiagnostics.channel_packet_counts[3] << " "
      << "channel4_packets=" << m_drainDiagnostics.channel_packet_counts[4] << " "
      << "channel5_packets=" << m_drainDiagnostics.channel_packet_counts[5];
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

void Bno086ImuNode::RecordTimestampTrace(const SensorEvent& event,
                                         int64_t raw_stamp_ns,
                                         int64_t packet_host_stamp_ns,
                                         const TimestampNormalizationResult& normalized,
                                         const ReportTimestampTrackerResult& tracker,
                                         std::optional<std::uint32_t> expected_interval_us)
{
  const std::optional<std::size_t> reportIndex = DiagnosticReportIndex(event.report_id);
  if (!reportIndex.has_value())
    return;

  TimestampTraceStats& stats = m_timestampTraceStats[*reportIndex];
  ++stats.events;

  if (event.has_base_timestamp)
    ++stats.has_base_count;

  if (event.has_delay)
    ++stats.has_delay_count;

  if (normalized.sequence_gap)
    ++stats.sequence_gap_count;

  if (tracker.gap_detected)
    ++stats.cadence_tracker_gap_count;

  if (tracker.reanchored)
    ++stats.cadence_tracker_reanchor_count;

  if (tracker.duplicate_sequence)
    ++stats.duplicate_sequence_count;

  if (normalized.repaired_nonmonotonic)
    ++stats.monotonic_guard_count;

  std::optional<int64_t> previousRawDeltaNs;
  if (stats.last_raw_stamp_ns.has_value())
  {
    previousRawDeltaNs = raw_stamp_ns - *stats.last_raw_stamp_ns;
    ++stats.raw_delta_count;
    stats.raw_delta_sum_ns += *previousRawDeltaNs;
    if (!stats.raw_delta_min_ns.has_value() || *previousRawDeltaNs < *stats.raw_delta_min_ns)
      stats.raw_delta_min_ns = *previousRawDeltaNs;

    if (*previousRawDeltaNs == 0)
      ++stats.duplicate_raw_stamp_count;

    if (expected_interval_us.has_value() && *previousRawDeltaNs > 0 &&
        *previousRawDeltaNs < static_cast<int64_t>(*expected_interval_us) * 500)
    {
      ++stats.tiny_raw_delta_count;
    }
  }

  std::optional<int64_t> previousNormalizedDeltaNs;
  if (stats.last_normalized_stamp_ns.has_value())
  {
    previousNormalizedDeltaNs = normalized.stamp_ns - *stats.last_normalized_stamp_ns;
    ++stats.normalized_delta_count;
    stats.normalized_delta_sum_ns += *previousNormalizedDeltaNs;
    if (!stats.normalized_delta_min_ns.has_value() ||
        *previousNormalizedDeltaNs < *stats.normalized_delta_min_ns)
    {
      stats.normalized_delta_min_ns = *previousNormalizedDeltaNs;
    }

    if (*previousNormalizedDeltaNs == 0)
      ++stats.duplicate_normalized_stamp_count;

    if (*previousNormalizedDeltaNs == 1)
      ++stats.legacy_plus_one_repair_count;
  }

  const bool repairedToInterval = normalized.repaired_duplicate_to_interval ||
                                  normalized.repaired_nonmonotonic_to_interval ||
                                  normalized.repaired_sequence_gap_to_interval;
  if (repairedToInterval)
    ++stats.interval_repair_count;

  if (normalized.interval_repair_clamped_to_host)
    ++stats.host_clamp_count;

  if (normalized.interval_repair_bounded_to_legacy)
    ++stats.bounded_to_legacy_count;

  MaybeLogTimestampTraceLine(event, raw_stamp_ns, packet_host_stamp_ns, normalized,
                             expected_interval_us, previousRawDeltaNs, previousNormalizedDeltaNs);

  stats.last_raw_stamp_ns = raw_stamp_ns;
  stats.last_normalized_stamp_ns = normalized.stamp_ns;
}

void Bno086ImuNode::MaybeLogTimestampTraceLine(const SensorEvent& event,
                                               int64_t raw_stamp_ns,
                                               int64_t packet_host_stamp_ns,
                                               const TimestampNormalizationResult& normalized,
                                               std::optional<std::uint32_t> expected_interval_us,
                                               std::optional<int64_t> previous_raw_delta_ns,
                                               std::optional<int64_t> previous_normalized_delta_ns)
{
  m_timestampTraceCount = static_cast<std::uint32_t>(
      std::clamp(static_cast<int>(get_parameter("bno086_timestamp_trace_count").as_int()), 0,
                 MAX_BNO086_TIMESTAMP_TRACE_COUNT));

  if (m_timestampTraceLogged >= m_timestampTraceCount)
    return;

  ++m_timestampTraceLogged;

  RCLCPP_INFO(get_logger(),
              "BNO086 timestamp trace: report=%s sequence=%u channel=%u "
              "payload_offset=%zu payload_record_bytes=%zu "
              "has_base_timestamp=%s base_timestamp_us=%u has_delay=%s delay_us=%u "
              "raw_reconstructed_stamp_ns=%lld packet_host_stamp_ns=%lld "
              "normalized_stamp_ns=%lld normalizer_duplicate=%s sequence_gap=%s "
              "repaired_nonmonotonic=%s repaired_duplicate_to_interval=%s "
              "repaired_nonmonotonic_to_interval=%s "
              "repaired_sequence_gap_to_interval=%s "
              "interval_repair_clamped_to_host=%s "
              "interval_repair_allowed_by_future_slop=%s "
              "interval_repair_bounded_to_legacy=%s expected_interval_us=%u "
              "previous_normalized_delta_ns=%lld previous_raw_delta_ns=%lld",
              ReportName(event.report_id), event.sequence, event.channel, event.report_offset,
              event.bytes_consumed, event.has_base_timestamp ? "true" : "false",
              event.base_timestamp_us, event.has_delay ? "true" : "false", event.delay_us,
              static_cast<long long>(raw_stamp_ns), static_cast<long long>(packet_host_stamp_ns),
              static_cast<long long>(normalized.stamp_ns), normalized.duplicate ? "true" : "false",
              normalized.sequence_gap ? "true" : "false",
              normalized.repaired_nonmonotonic ? "true" : "false",
              normalized.repaired_duplicate_to_interval ? "true" : "false",
              normalized.repaired_nonmonotonic_to_interval ? "true" : "false",
              normalized.repaired_sequence_gap_to_interval ? "true" : "false",
              normalized.interval_repair_clamped_to_host ? "true" : "false",
              normalized.interval_repair_allowed_by_future_slop ? "true" : "false",
              normalized.interval_repair_bounded_to_legacy ? "true" : "false",
              expected_interval_us.value_or(0),
              static_cast<long long>(previous_normalized_delta_ns.value_or(0)),
              static_cast<long long>(previous_raw_delta_ns.value_or(0)));
}

void Bno086ImuNode::MaybeEmitTimestampSummaries() const
{
  constexpr std::array<ReportId, 5> kSummaryReports = {
      ReportId::Accelerometer,  ReportId::GyroscopeCalibrated,
      ReportId::RotationVector, ReportId::LinearAcceleration,
      ReportId::Gravity,
  };

  for (const ReportId reportId : kSummaryReports)
  {
    const std::optional<std::size_t> reportIndex = DiagnosticReportIndex(reportId);
    if (!reportIndex.has_value())
      continue;

    const TimestampTraceStats& stats = m_timestampTraceStats[*reportIndex];
    if (stats.events == 0)
      continue;

    const double rawDeltaMinMs = stats.raw_delta_min_ns.has_value()
                                     ? static_cast<double>(*stats.raw_delta_min_ns) / 1e6
                                     : 0.0;
    const double rawDeltaMeanMs = stats.raw_delta_count > 0
                                      ? static_cast<double>(stats.raw_delta_sum_ns) /
                                            static_cast<double>(stats.raw_delta_count) / 1e6
                                      : 0.0;
    const double normalizedDeltaMinMs =
        stats.normalized_delta_min_ns.has_value()
            ? static_cast<double>(*stats.normalized_delta_min_ns) / 1e6
            : 0.0;
    const double normalizedDeltaMeanMs =
        stats.normalized_delta_count > 0
            ? static_cast<double>(stats.normalized_delta_sum_ns) /
                  static_cast<double>(stats.normalized_delta_count) / 1e6
            : 0.0;

    RCLCPP_INFO(get_logger(),
                "BNO086 timestamp summary: report=%s events=%llu "
                "has_base_count=%llu has_delay_count=%llu sequence_gap_count=%llu "
                "cadence_tracker_gap_count=%llu cadence_tracker_reanchor_count=%llu "
                "monotonic_guard_count=%llu duplicate_sequence_count=%llu "
                "duplicate_raw_stamp_count=%llu tiny_raw_delta_count=%llu "
                "duplicate_normalized_stamp_count=%llu legacy_plus_one_repair_count=%llu "
                "interval_repair_count=%llu host_clamp_count=%llu "
                "bounded_to_legacy_count=%llu raw_delta_min_ms=%.6f "
                "raw_delta_mean_ms=%.6f normalized_delta_min_ms=%.6f "
                "normalized_delta_mean_ms=%.6f",
                ReportName(reportId), static_cast<unsigned long long>(stats.events),
                static_cast<unsigned long long>(stats.has_base_count),
                static_cast<unsigned long long>(stats.has_delay_count),
                static_cast<unsigned long long>(stats.sequence_gap_count),
                static_cast<unsigned long long>(stats.cadence_tracker_gap_count),
                static_cast<unsigned long long>(stats.cadence_tracker_reanchor_count),
                static_cast<unsigned long long>(stats.monotonic_guard_count),
                static_cast<unsigned long long>(stats.duplicate_sequence_count),
                static_cast<unsigned long long>(stats.duplicate_raw_stamp_count),
                static_cast<unsigned long long>(stats.tiny_raw_delta_count),
                static_cast<unsigned long long>(stats.duplicate_normalized_stamp_count),
                static_cast<unsigned long long>(stats.legacy_plus_one_repair_count),
                static_cast<unsigned long long>(stats.interval_repair_count),
                static_cast<unsigned long long>(stats.host_clamp_count),
                static_cast<unsigned long long>(stats.bounded_to_legacy_count), rawDeltaMinMs,
                rawDeltaMeanMs, normalizedDeltaMinMs, normalizedDeltaMeanMs);
  }
}

std::uint32_t Bno086ImuNode::CoreCoherenceToleranceUs() const
{
  const std::optional<std::uint32_t> linearIntervalUs =
      EffectiveReportIntervalUs(ReportId::LinearAcceleration);
  const std::optional<std::uint32_t> gyroIntervalUs =
      EffectiveReportIntervalUs(ReportId::GyroscopeCalibrated);
  const std::optional<std::uint32_t> orientationIntervalUs =
      EffectiveReportIntervalUs(ReportId::RotationVector);
  const int64_t toleranceNs = EffectiveCoreSpanToleranceNs(
      static_cast<int64_t>(linearIntervalUs.value_or(m_reportIntervalUs)) * 1'000,
      static_cast<int64_t>(gyroIntervalUs.value_or(m_reportIntervalUs)) * 1'000,
      static_cast<int64_t>(orientationIntervalUs.value_or(m_reportIntervalUs)) * 1'000,
      static_cast<int64_t>(MIN_COHERENT_SAMPLE_SPAN_US) * 1'000);

  return static_cast<std::uint32_t>(toleranceNs / 1'000);
}

int64_t Bno086ImuNode::ImuGravityMaxOrientationAgeNs() const
{
  const std::optional<std::uint32_t> intervalUs =
      EffectiveReportIntervalUs(ReportId::RotationVector);
  return EffectiveMaxPastAgeNs(
      static_cast<int64_t>(m_imuGravityMaxOrientationAgeMs * 1.0e6),
      static_cast<int64_t>(intervalUs.value_or(m_reportIntervalUs)) * 1'000, 50'000'000);
}

int64_t Bno086ImuNode::ImuGravityMaxGyroAgeNs() const
{
  const std::optional<std::uint32_t> intervalUs =
      EffectiveReportIntervalUs(ReportId::GyroscopeCalibrated);
  return EffectiveMaxPastAgeNs(
      static_cast<int64_t>(m_imuGravityMaxGyroAgeMs * 1.0e6),
      static_cast<int64_t>(intervalUs.value_or(m_reportIntervalUs)) * 1'000, 50'000'000);
}

int64_t Bno086ImuNode::ReportFutureToleranceNs(ReportId report_id) const
{
  const std::optional<std::uint32_t> intervalUs = EffectiveReportIntervalUs(report_id);
  if (intervalUs.has_value() && *intervalUs > 0)
    return static_cast<int64_t>(*intervalUs) * 1'000;

  return DurationFromUs(CoreCoherenceToleranceUs()).nanoseconds();
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
  return IsTimestampSpanCoherent(oldestNs, newestNs,
                                 DurationFromUs(CoreCoherenceToleranceUs()).nanoseconds());
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

auto Bno086ImuNode::EstimateEventStamp(const SensorEvent& event,
                                       const rclcpp::Time& interrupt_ros_at,
                                       std::optional<int64_t> expected_interval_ns)
    -> EstimatedEventStamp
{
  const auto trackerIndex = static_cast<std::size_t>(event.report_id);
  if (!m_bnoCadenceEpochNs.has_value())
    m_bnoCadenceEpochNs = HostAnchorNs(event, interrupt_ros_at.nanoseconds());

  const ReportTimestampTrackerResult trackedStamp =
      m_timestampTrackers[trackerIndex].Update(ReportTimestampTrackerInput{
          event.sequence, expected_interval_ns, interrupt_ros_at.nanoseconds(), event.has_delay,
          event.delay_us, m_bnoCadenceEpochNs});

  if (trackedStamp.reanchored || trackedStamp.gap_detected)
  {
    RCLCPP_DEBUG_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "BNO086 timestamp tracker state: report=%s initialized=%s "
        "reanchored=%s gap_detected=%s duplicate_sequence=%s "
        "sequence_delta=%u used_interval_cadence=%s "
        "used_host_anchor=%s used_shared_epoch_anchor=%s "
        "shared_epoch_ns=%lld",
        ReportName(event.report_id), trackedStamp.initialized ? "true" : "false",
        trackedStamp.reanchored ? "true" : "false", trackedStamp.gap_detected ? "true" : "false",
        trackedStamp.duplicate_sequence ? "true" : "false", trackedStamp.sequence_delta,
        trackedStamp.used_interval_cadence ? "true" : "false",
        trackedStamp.used_host_anchor ? "true" : "false",
        trackedStamp.used_shared_epoch_anchor ? "true" : "false",
        static_cast<long long>(m_bnoCadenceEpochNs.value_or(0)));
  }

  return EstimatedEventStamp{rclcpp::Time(trackedStamp.stamp_ns, RCL_ROS_TIME), trackedStamp};
}

TimestampNormalizationResult Bno086ImuNode::FinalizeEventStamp(
    const SensorEvent& event,
    const EstimatedEventStamp& estimated_stamp,
    int64_t packet_host_stamp_ns,
    std::optional<int64_t> expected_interval_ns)
{
  const auto timestampIndex = static_cast<std::size_t>(event.report_id);

  if (expected_interval_ns.has_value())
  {
    TimestampNormalizationResult result;
    result.stamp_ns = estimated_stamp.stamp.nanoseconds();
    result.sequence_gap = estimated_stamp.tracker.gap_detected;
    result.duplicate = estimated_stamp.tracker.duplicate_sequence;

    const std::optional<int64_t>& lastStampNs = m_lastEmittedTimestampNs[timestampIndex];
    if (lastStampNs.has_value() && result.stamp_ns <= *lastStampNs)
    {
      result.repaired_nonmonotonic = true;
      if (estimated_stamp.tracker.duplicate_sequence)
      {
        result.stamp_ns = *lastStampNs + 1;
      }
      else if (*expected_interval_ns > 0)
      {
        result.stamp_ns = *lastStampNs + *expected_interval_ns;
      }
      else
      {
        result.stamp_ns = *lastStampNs + 1;
      }

      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "BNO086 cadence timestamp guard repaired report=%s "
                           "sequence=%u candidate_ns=%lld last_ns=%lld final_ns=%lld "
                           "duplicate_sequence=%s sequence_delta=%u",
                           ReportName(event.report_id), event.sequence,
                           static_cast<long long>(estimated_stamp.stamp.nanoseconds()),
                           static_cast<long long>(*lastStampNs),
                           static_cast<long long>(result.stamp_ns),
                           estimated_stamp.tracker.duplicate_sequence ? "true" : "false",
                           estimated_stamp.tracker.sequence_delta);
    }

    m_lastEmittedTimestampNs[timestampIndex] = result.stamp_ns;
    return result;
  }

  // Units: ns. Allows fallback reconstructed samples to land just ahead of the
  // stable host drain anchor
  constexpr int64_t kFallbackTimestampFutureSlopNs = 50'000'000;

  TimestampNormalizationResult result = m_timestampNormalizers[timestampIndex].Normalize(
      TimestampSample{event.sequence, estimated_stamp.stamp.nanoseconds(), packet_host_stamp_ns,
                      std::make_optional(kFallbackTimestampFutureSlopNs)},
      expected_interval_ns);
  m_lastEmittedTimestampNs[timestampIndex] = result.stamp_ns;
  return result;
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
