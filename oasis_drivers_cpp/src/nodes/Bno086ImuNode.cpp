/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "Bno086ImuNode.hpp"

#include "imu/bno086/core/Bno086TimestampCadence.hpp"
#include "imu/bno086/gpio/Bno086Gpio.hpp"
#include "imu/bno086/sh2/Bno086Shtp.hpp"
#include "imu/bno086/shtp/Bno086Transport.hpp"
#include "imu/bno086/utils/Bno086CovarianceUtils.hpp"
#include "imu/bno086/utils/Bno086FeatureSummary.hpp"
#include "imu/bno086/utils/Bno086MathUtils.hpp"
#include "imu/bno086/utils/Bno086ReportUtils.hpp"
#include "imu/bno086/utils/Bno086TimingUtils.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
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
constexpr int DEFAULT_MAX_ALL_ZERO_POLLS_PER_INTERRUPT = 64;
constexpr int MIN_MAX_ALL_ZERO_POLLS_PER_INTERRUPT = 1;
constexpr int MAX_MAX_ALL_ZERO_POLLS_PER_INTERRUPT = 1024;
constexpr int DEFAULT_ALL_ZERO_BACKOFF_US = 500;
constexpr int MIN_ALL_ZERO_BACKOFF_US = 0;
constexpr int MAX_ALL_ZERO_BACKOFF_US = 10'000;
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

constexpr std::uint32_t MIN_COHERENT_SAMPLE_SPAN_US = 80'000;
constexpr double DEFAULT_PREDICTION_HORIZON_SEC = 0.0;
constexpr std::uint32_t DEFAULT_REPORT_INTERVAL_US = 10'000;

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

// Maximum nearby orientation age accepted for imu_gravity composition
constexpr double DEFAULT_IMU_GRAVITY_MAX_ORIENTATION_AGE_MS = 80.0;

// Maximum nearby gyro age accepted for imu_gravity composition
constexpr double DEFAULT_IMU_GRAVITY_MAX_GYRO_AGE_MS = 80.0;

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

// QoS parameters
//
// RELIABLE for required BNO -> AHRS pipeline streams with bounded history
constexpr std::size_t RAW_IMU_GRAVITY_QOS_DEPTH = 256;
constexpr std::size_t RAW_IMU_QOS_DEPTH = 512;
constexpr std::size_t RAW_GRAVITY_QOS_DEPTH = 256;

// BEST_EFFORT only for disposable live/debug streams
constexpr std::size_t DEBUG_IMU_QOS_DEPTH = 10;

rclcpp::QoS BestEffortSensorQos(std::size_t depth)
{
  // BEST_EFFORT is reserved for disposable live/debug sensor streams
  rclcpp::QoS qos{rclcpp::KeepLast(depth)};
  qos.best_effort();
  qos.durability_volatile();
  return qos;
}

rclcpp::QoS ReliableSensorQos(std::size_t depth)
{
  // RELIABLE is used for required pipeline and replay-facing streams
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

double VectorMagnitude(double x, double y, double z)
{
  return std::sqrt(x * x + y * y + z * z);
}

} // namespace

Bno086ImuNode::Bno086ImuNode()
  : rclcpp::Node(NODE_NAME),
    m_transport(std::make_unique<Bno086Transport>()),
    m_shtp(std::make_unique<Bno086Shtp>(*m_transport)),
    m_interruptGpio(std::make_unique<Bno086Gpio>()),
    m_timestampCadence(std::make_unique<Bno086TimestampCadence>())
{
  declare_parameter("i2c_device", std::string(DEFAULT_I2C_DEVICE));
  declare_parameter("i2c_address", static_cast<int>(DEFAULT_I2C_ADDRESS));
  declare_parameter("int_gpio", DEFAULT_INT_GPIO);
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
  declare_parameter("bno086_all_zero_backoff_us", DEFAULT_ALL_ZERO_BACKOFF_US);
  declare_parameter("bno086_diagnostics_log_period_ms", DEFAULT_BNO086_DIAGNOSTICS_LOG_PERIOD_MS);
  declare_parameter("prediction_horizon_sec", DEFAULT_PREDICTION_HORIZON_SEC);
  declare_parameter("imu_gravity_max_orientation_age_ms",
                    DEFAULT_IMU_GRAVITY_MAX_ORIENTATION_AGE_MS);
  declare_parameter("imu_gravity_max_gyro_age_ms", DEFAULT_IMU_GRAVITY_MAX_GYRO_AGE_MS);

  declare_parameter("frame_id", std::string(DEFAULT_FRAME_ID));

  const std::string i2cDevice = get_parameter("i2c_device").as_string();
  const std::uint8_t i2cAddress = static_cast<std::uint8_t>(get_parameter("i2c_address").as_int());
  const int intGpio = get_parameter("int_gpio").as_int();
  m_reports.rotation_vector_rate_hz =
      std::max(get_parameter("bno086_rotation_vector_rate_hz").as_double(), 1.0);
  m_reports.gyro_rate_hz = std::max(get_parameter("bno086_gyro_rate_hz").as_double(), 1.0);
  m_reports.accelerometer_rate_hz =
      std::max(get_parameter("bno086_accelerometer_rate_hz").as_double(), 1.0);
  m_reports.linear_acceleration_rate_hz =
      std::max(get_parameter("bno086_linear_acceleration_rate_hz").as_double(), 1.0);
  m_reports.gravity_rate_hz = std::max(get_parameter("bno086_gravity_rate_hz").as_double(), 1.0);
  m_reports.rotation_vector_batch_interval_us =
      MillisecondsToMicroseconds(get_parameter("bno086_rotation_vector_batch_ms").as_double());
  m_reports.gyro_batch_interval_us =
      MillisecondsToMicroseconds(get_parameter("bno086_gyro_batch_ms").as_double());
  m_reports.accelerometer_batch_interval_us =
      MillisecondsToMicroseconds(get_parameter("bno086_accelerometer_batch_ms").as_double());
  m_reports.linear_acceleration_batch_interval_us =
      MillisecondsToMicroseconds(get_parameter("bno086_linear_acceleration_batch_ms").as_double());
  m_reports.gravity_batch_interval_us =
      MillisecondsToMicroseconds(get_parameter("bno086_gravity_batch_ms").as_double());
  m_reports.enable_linear_acceleration_report =
      get_parameter("bno086_enable_linear_acceleration_report").as_bool();
  m_reports.enable_gravity_report = get_parameter("bno086_enable_gravity_report").as_bool();
  m_config.all_zero_backoff_us =
      std::clamp(static_cast<int>(get_parameter("bno086_all_zero_backoff_us").as_int()),
                 MIN_ALL_ZERO_BACKOFF_US, MAX_ALL_ZERO_BACKOFF_US);
  m_config.diagnostics_log_period_ms =
      std::max(static_cast<int>(get_parameter("bno086_diagnostics_log_period_ms").as_int()),
               MIN_BNO086_DIAGNOSTICS_LOG_PERIOD_MS);
  m_config.prediction_horizon_sec =
      std::max(get_parameter("prediction_horizon_sec").as_double(), 0.0);
  m_config.imu_gravity_max_orientation_age_ms =
      std::max(get_parameter("imu_gravity_max_orientation_age_ms").as_double(), 0.0);
  m_config.imu_gravity_max_gyro_age_ms =
      std::max(get_parameter("imu_gravity_max_gyro_age_ms").as_double(), 0.0);

  m_config.frame_id = get_parameter("frame_id").as_string();

  Bno086TransportConfig transportConfig;
  transportConfig.i2c_device = i2cDevice;
  transportConfig.i2c_address = i2cAddress;

  if (!m_transport->Open(transportConfig))
  {
    RCLCPP_ERROR(get_logger(), "Failed to open BNO086 on %s (0x%02X)", i2cDevice.c_str(),
                 static_cast<unsigned>(i2cAddress));
    throw std::runtime_error("Failed to open BNO086 transport");
  }

  Bno086GpioConfig gpioConfig;
  gpioConfig.line_offset = static_cast<unsigned>(std::max(intGpio, 0));

  if (!m_interruptGpio->Open(gpioConfig))
  {
    RCLCPP_ERROR(get_logger(), "Failed to open GPIO interrupt line %d on %s", intGpio,
                 gpioConfig.chip_device.c_str());
    throw std::runtime_error("Failed to open BNO086 GPIO interrupt");
  }

  Bno086ShtpConfig shtpConfig;
  shtpConfig.rotation_vector_rate_hz = m_reports.rotation_vector_rate_hz;
  shtpConfig.gyro_rate_hz = m_reports.gyro_rate_hz;
  shtpConfig.accelerometer_rate_hz = m_reports.accelerometer_rate_hz;
  shtpConfig.linear_acceleration_rate_hz = m_reports.linear_acceleration_rate_hz;
  shtpConfig.gravity_rate_hz = m_reports.gravity_rate_hz;
  shtpConfig.rotation_vector_batch_interval_us = m_reports.rotation_vector_batch_interval_us;
  shtpConfig.gyro_batch_interval_us = m_reports.gyro_batch_interval_us;
  shtpConfig.accelerometer_batch_interval_us = m_reports.accelerometer_batch_interval_us;
  shtpConfig.linear_acceleration_batch_interval_us =
      m_reports.linear_acceleration_batch_interval_us;
  shtpConfig.gravity_batch_interval_us = m_reports.gravity_batch_interval_us;
  shtpConfig.enable_linear_acceleration_report = m_reports.enable_linear_acceleration_report;
  shtpConfig.enable_gravity_report = m_reports.enable_gravity_report;

  if (!m_shtp->Configure(shtpConfig))
  {
    RCLCPP_ERROR(get_logger(), "Failed to send initial BNO086 Set Feature commands");
    throw std::runtime_error("Failed to configure BNO086 reports");
  }
  m_startup.feature_configuration_started_at = std::chrono::steady_clock::now();
  (void)m_shtp->DrainFeatureResponses(m_config.feature_response_startup_drain_ms,
                                      m_config.feature_response_startup_max_packets,
                                      m_config.packet_read_timeout_ms);
  MaybeLogFeatureResponses();

  if (intGpio == DEFAULT_INT_GPIO)
  {
    RCLCPP_INFO(get_logger(), "BNO086 INT uses GPIO%d (Raspberry Pi header pin 16), active low",
                intGpio);
  }

  RCLCPP_INFO(get_logger(), "BNO086 opened on %s (0x%02X), int_gpio=%d active_low",
              i2cDevice.c_str(), static_cast<unsigned>(i2cAddress), intGpio);
  RCLCPP_INFO(
      get_logger(),
      "BNO086 static report rates:\n"
      "  rotation_vector=%.1f batch_ms=%.0f enabled=true\n"
      "  gyro=%.1f batch_ms=%.0f enabled=true\n"
      "  accelerometer=%.1f batch_ms=%.0f enabled=true\n"
      "  linear_acceleration=%.1f batch_ms=%.0f enabled=%s\n"
      "  gravity=%.1f batch_ms=%.0f enabled=%s",
      m_reports.rotation_vector_rate_hz,
      static_cast<double>(m_reports.rotation_vector_batch_interval_us) / 1000.0,
      m_reports.gyro_rate_hz, static_cast<double>(m_reports.gyro_batch_interval_us) / 1000.0,
      m_reports.accelerometer_rate_hz,
      static_cast<double>(m_reports.accelerometer_batch_interval_us) / 1000.0,
      m_reports.linear_acceleration_rate_hz,
      m_reports.enable_linear_acceleration_report
          ? static_cast<double>(m_reports.linear_acceleration_batch_interval_us) / 1000.0
          : 0.0,
      m_reports.enable_linear_acceleration_report ? "true" : "false", m_reports.gravity_rate_hz,
      m_reports.enable_gravity_report
          ? static_cast<double>(m_reports.gravity_batch_interval_us) / 1000.0
          : 0.0,
      m_reports.enable_gravity_report ? "true" : "false");
  RCLCPP_INFO(get_logger(), "BNO086 publication cadence: imu=linear_acceleration "
                            "imu_gravity=rotation_vector gravity=gravity_report");
  if (m_config.prediction_horizon_sec > 0.0)
  {
    RCLCPP_INFO(get_logger(),
                "Predicted orientation output uses %s with prediction_horizon_sec=%.4f",
                m_config.prediction_source.c_str(), m_config.prediction_horizon_sec);
  }
  RCLCPP_INFO(get_logger(),
              "BNO086 imu_gravity uses rotation vector cadence with "
              "orientation_max_age_ms=%.1f gyro_max_age_ms=%.1f",
              m_config.imu_gravity_max_orientation_age_ms, m_config.imu_gravity_max_gyro_age_ms);
}

Bno086ImuNode::~Bno086ImuNode() = default;

bool Bno086ImuNode::Initialize()
{
  m_timestampCadence->Reset();
  m_stream.last_published_core_signature.reset();
  m_stream.last_published_imu_gravity_anchor_stamp_ns.reset();
  m_stream.imu_gravity_accel_history.Reset();
  m_diag.drain_health = Bno086DrainHealth{};
  m_diag.rate_health = Bno086RateHealth{};

  m_imuPublisher =
      create_publisher<sensor_msgs::msg::Imu>(IMU_TOPIC, ReliableSensorQos(RAW_IMU_QOS_DEPTH));
  m_imuPredictedPublisher = create_publisher<sensor_msgs::msg::Imu>(
      IMU_PREDICTED_TOPIC, BestEffortSensorQos(DEBUG_IMU_QOS_DEPTH));
  m_imuVrPublisher = create_publisher<oasis_msgs::msg::ImuVr>(
      IMU_VR_TOPIC, BestEffortSensorQos(DEBUG_IMU_QOS_DEPTH));
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

  m_interruptGpio->Close();
  m_transport->Close();
}

void Bno086ImuNode::InterruptLoop()
{
  while (m_running.load() && rclcpp::ok())
  {
    std::chrono::steady_clock::time_point interruptSteadyAt;
    const Bno086Gpio::WaitResult waitResult =
        m_interruptGpio->WaitForAssertedLow(INTERRUPT_WAIT_TIMEOUT_MS, interruptSteadyAt);

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

    if (startup.communication_established && !m_startup.logged_comm_established)
    {
      RCLCPP_INFO(get_logger(), "BNO086 communication established");
      m_startup.logged_comm_established = true;
    }

    if (startup.set_feature_sent && !m_startup.logged_set_feature)
    {
      RCLCPP_INFO(get_logger(), "BNO086 Set Feature commands sent");
      m_startup.logged_set_feature = true;
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
  drainLimits.max_physical_packets_per_interrupt = m_drain_config.max_packets_per_interrupt;
  drainLimits.max_poll_iterations_per_interrupt = m_drain_config.max_poll_iterations_per_interrupt;
  drainLimits.max_no_progress_polls_per_interrupt =
      m_drain_config.max_no_progress_polls_per_interrupt;
  drainLimits.max_all_zero_polls_per_interrupt = m_drain_config.max_all_zero_polls_per_interrupt;
  drainLimits.max_sensor_events_per_drain = m_drain_config.max_sensor_events_per_drain;
  drainLimits.max_pending_events_flush_per_drain =
      m_drain_config.max_pending_events_flush_per_drain;
  const rclcpp::Time drainReceiveAnchor = interrupt_ros_at;
  const auto drainStartedAt = std::chrono::steady_clock::now();
  Bno086DrainAction drainExitAction = Bno086DrainAction::Complete;

  while (m_running.load())
  {
    const bool hintnAssertedBeforePoll = m_interruptGpio->IsAssertedLow();
    const std::size_t pendingEventsBeforePoll = m_shtp->PendingEventCount();
    if (pendingEventsBeforePoll == 0 && !hintnAssertedBeforePoll)
    {
      drainExitAction = Bno086DrainAction::Complete;
      break;
    }

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
          "all_zero_polls_this_drain=%u consecutive_all_zero_polls=%u "
          "packet_cap=%u poll_iteration_cap=%u no_progress_budget=%u all_zero_budget=%u "
          "drain_duration_ms=%.3f hintn_asserted=%s",
          drainCounters.physical_packets_this_drain, drainCounters.sensor_events_this_drain,
          drainCounters.pending_events_this_drain, drainCounters.control_packets_this_drain,
          drainCounters.poll_iterations, drainCounters.consecutive_no_progress_polls,
          drainCounters.all_zero_polls_this_drain, drainCounters.consecutive_all_zero_polls,
          drainLimits.max_physical_packets_per_interrupt,
          drainLimits.max_poll_iterations_per_interrupt,
          drainLimits.max_no_progress_polls_per_interrupt,
          drainLimits.max_all_zero_polls_per_interrupt, static_cast<double>(drainDurationUs) / 1e3,
          beforePollDecision.hintn_asserted ? "true" : "false");
      drainExitAction = beforePollDecision.action;
      break;
    }

    const auto elapsedBeforePollMs = std::chrono::duration_cast<std::chrono::milliseconds>(
                                         std::chrono::steady_clock::now() - drainStartedAt)
                                         .count();
    if (pendingEventsBeforePoll == 0 && drainCounters.poll_iterations > 0 &&
        elapsedBeforePollMs >= m_config.max_drain_duration_ms)
    {
      drainExitAction = Bno086DrainAction::DrainDurationBudget;
      break;
    }

    const Bno086Shtp::PollResult pollResult = m_shtp->Poll(m_config.packet_read_timeout_ms);
    const bool hintnAssertedAfterPoll = m_interruptGpio->IsAssertedLow();
    const Bno086DrainDecision afterPollDecision =
        Bno086DrainAfterPoll(pollResult, drainLimits, drainCounters, hintnAssertedAfterPoll);
    if (afterPollDecision.action == Bno086DrainAction::Complete)
    {
      m_diag.repeated_no_progress_timeouts = 0;
      drainExitAction = afterPollDecision.action;
      break;
    }

    if (afterPollDecision.action == Bno086DrainAction::NoProgressBudget)
    {
      ++m_diag.repeated_no_progress_timeouts;
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
          "all_zero_polls_this_drain=%u consecutive_all_zero_polls=%u "
          "packet_cap=%u poll_iteration_cap=%u drain_duration_ms=%.3f "
          "hintn_asserted=true",
          drainCounters.physical_packets_this_drain, drainCounters.sensor_events_this_drain,
          drainCounters.pending_events_this_drain, drainCounters.control_packets_this_drain,
          drainCounters.poll_iterations, drainCounters.consecutive_no_progress_polls,
          drainLimits.max_no_progress_polls_per_interrupt, drainCounters.all_zero_polls_this_drain,
          drainCounters.consecutive_all_zero_polls, drainLimits.max_physical_packets_per_interrupt,
          drainLimits.max_poll_iterations_per_interrupt,
          static_cast<double>(drainDurationUs) / 1e3);
      drainExitAction = afterPollDecision.action;
      break;
    }

    if (afterPollDecision.action == Bno086DrainAction::AllZeroBudget)
    {
      const auto drainDurationUs =
          static_cast<std::uint32_t>(std::chrono::duration_cast<std::chrono::microseconds>(
                                         std::chrono::steady_clock::now() - drainStartedAt)
                                         .count());
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "BNO086 interrupt drain exhausted all-zero retry budget while H_INTN "
          "remained asserted: physical_packets_this_drain=%u "
          "sensor_events_this_drain=%u pending_events_this_drain=%u "
          "control_packets_this_drain=%u poll_iterations=%u "
          "all_zero_polls_this_drain=%u consecutive_all_zero_polls=%u "
          "all_zero_budget=%u all_zero_backoff_us=%d drain_duration_ms=%.3f "
          "hintn_asserted=true",
          drainCounters.physical_packets_this_drain, drainCounters.sensor_events_this_drain,
          drainCounters.pending_events_this_drain, drainCounters.control_packets_this_drain,
          drainCounters.poll_iterations, drainCounters.all_zero_polls_this_drain,
          drainCounters.consecutive_all_zero_polls, drainLimits.max_all_zero_polls_per_interrupt,
          m_config.all_zero_backoff_us, static_cast<double>(drainDurationUs) / 1e3);
      drainExitAction = afterPollDecision.action;
      break;
    }

    if (pollResult.read_physical_packet || pollResult.event.has_value() ||
        pollResult.dequeued_pending_event || pollResult.handled_control_packet)
    {
      m_diag.repeated_no_progress_timeouts = 0;
    }

    if (afterPollDecision.action == Bno086DrainAction::TransportError)
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "BNO086 transport error while draining interrupt data");
      drainExitAction = afterPollDecision.action;
      break;
    }

    if (pollResult.status == Bno086Shtp::PollStatus::AllZeroHeader &&
        afterPollDecision.action == Bno086DrainAction::Continue &&
        afterPollDecision.hintn_asserted && m_config.all_zero_backoff_us > 0)
    {
      m_diag.drain_health.CountAllZeroBackoff();
      std::this_thread::sleep_for(std::chrono::microseconds(m_config.all_zero_backoff_us));
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
      const std::optional<int64_t> finalizedStampNs =
          FinalizeEventStampNs(*pollResult.event, drainReceiveAnchor, expectedIntervalNs);

      if (finalizedStampNs.has_value())
      {
        const rclcpp::Time normalizedEventStamp(*finalizedStampNs, RCL_ROS_TIME);
        m_diag.rate_health.CountDecodedReport(pollResult.event->report_id);
        ApplyEvent(*pollResult.event, normalizedEventStamp);
        MaybePublishOnLinearAcceleration(*pollResult.event);
        MaybePublishImuGravityOnRotationVector(*pollResult.event);
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
            "consecutive_no_progress_polls=%u all_zero_polls_this_drain=%u "
            "consecutive_all_zero_polls=%u packet_cap=%u "
            "poll_iteration_cap=%u no_progress_budget=%u all_zero_budget=%u "
            "drain_duration_ms=%.3f "
            "hintn_asserted=true",
            drainCounters.physical_packets_this_drain, drainCounters.sensor_events_this_drain,
            drainCounters.pending_events_this_drain, drainCounters.control_packets_this_drain,
            drainCounters.poll_iterations, drainCounters.consecutive_no_progress_polls,
            drainCounters.all_zero_polls_this_drain, drainCounters.consecutive_all_zero_polls,
            drainLimits.max_physical_packets_per_interrupt,
            drainLimits.max_poll_iterations_per_interrupt,
            drainLimits.max_no_progress_polls_per_interrupt,
            drainLimits.max_all_zero_polls_per_interrupt,
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
    if (m_shtp->PendingEventCount() == 0 && elapsedAfterPollMs >= m_config.max_drain_duration_ms)
    {
      drainExitAction = Bno086DrainAction::DrainDurationBudget;
      break;
    }
  }

  const bool hintnAssertedAfterExit = m_interruptGpio->IsAssertedLow();
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

  if (!(m_stream.latest_frame.has_orientation && m_stream.latest_frame.has_gyro &&
        m_stream.latest_frame.has_linear_accel) ||
      !(m_stream.orientation.has_sample && m_stream.gyro.has_sample &&
        m_stream.linear_accel.has_sample))
  {
    MaybeLogImuGravityDiagnostics();
    return;
  }

  const int64_t orientationNs = m_stream.orientation.stamp.nanoseconds();
  const int64_t gyroNs = m_stream.gyro.stamp.nanoseconds();
  const int64_t linearAccelNs = m_stream.linear_accel.stamp.nanoseconds();
  const int64_t oldestNs = std::min({orientationNs, gyroNs, linearAccelNs});
  const int64_t newestNs = std::max({orientationNs, gyroNs, linearAccelNs});

  const int64_t coreThresholdNs = DurationNsFromUs(CoreCoherenceToleranceUs());
  if (!IsTimestampSpanCoherent(oldestNs, newestNs, coreThresholdNs))
  {
    MaybeLogImuGravityDiagnostics();
    return;
  }

  const CoreFrameSignature signature = LatestCoreSignature();
  if (m_stream.last_published_core_signature.has_value() &&
      signature.orientation_sequence ==
          m_stream.last_published_core_signature->orientation_sequence &&
      signature.gyro_sequence == m_stream.last_published_core_signature->gyro_sequence &&
      signature.linear_accel_sequence ==
          m_stream.last_published_core_signature->linear_accel_sequence &&
      signature.orientation_stamp_ns ==
          m_stream.last_published_core_signature->orientation_stamp_ns &&
      signature.gyro_stamp_ns == m_stream.last_published_core_signature->gyro_stamp_ns &&
      signature.linear_accel_stamp_ns ==
          m_stream.last_published_core_signature->linear_accel_stamp_ns)
  {
    MaybeLogImuGravityDiagnostics();
    return;
  }

  PublishLatestFrame(LatestCoreStamp());
  m_stream.last_published_core_signature = signature;
  m_diag.rate_health.CountImuPublished();
}

void Bno086ImuNode::MaybePublishImuGravityOnRotationVector(const SensorEvent& event)
{
  if (event.report_id != ReportId::RotationVector)
    return;

  if (!m_stream.orientation.has_sample || !m_stream.latest_frame.has_orientation)
  {
    MaybeLogImuGravityDiagnostics();
    return;
  }

  if (!m_stream.gyro.has_sample || !m_stream.latest_frame.has_gyro)
  {
    MaybeLogImuGravityDiagnostics();
    return;
  }

  if (!m_stream.imu_gravity.has_sample || !m_stream.latest_frame.has_accel)
  {
    MaybeLogImuGravityDiagnostics();
    return;
  }

  const int64_t orientationStampNs = m_stream.orientation.stamp.nanoseconds();
  const int64_t maxAccelAgeNs = ImuGravityMaxOrientationAgeNs();
  const int64_t accelFutureToleranceNs = ReportFutureToleranceNs(ReportId::Accelerometer);
  const std::optional<Bno086ImuGravityAccelSample> accelSelection =
      SelectImuGravityAccelSample(orientationStampNs, accelFutureToleranceNs);

  if (!accelSelection.has_value())
  {
    MaybeLogImuGravityDiagnostics();
    return;
  }

  const Bno086ImuGravityAccelSample& accelSample = *accelSelection;
  const int64_t accelStampNs = accelSample.stamp_ns;
  const int64_t maxGyroAgeNs = ImuGravityMaxGyroAgeNs();
  const SampleFreshnessResult accelFreshness = EvaluateSampleFreshness(
      orientationStampNs, accelStampNs, maxAccelAgeNs, accelFutureToleranceNs);
  const SampleFreshnessResult gyroFreshness =
      EvaluateSampleFreshness(orientationStampNs, m_stream.gyro.stamp.nanoseconds(), maxGyroAgeNs,
                              ReportFutureToleranceNs(ReportId::GyroscopeCalibrated));

  if (accelFreshness.status == SampleFreshnessStatus::TooOld)
  {
    MaybeLogImuGravityDiagnostics();
    return;
  }

  if (accelFreshness.status == SampleFreshnessStatus::TooFuture)
  {
    MaybeLogImuGravityDiagnostics();
    return;
  }

  if (gyroFreshness.status == SampleFreshnessStatus::TooOld)
  {
    MaybeLogImuGravityDiagnostics();
    return;
  }

  if (gyroFreshness.status == SampleFreshnessStatus::TooFuture)
  {
    MaybeLogImuGravityDiagnostics();
    return;
  }

  if (m_stream.last_published_imu_gravity_anchor_stamp_ns.has_value() &&
      orientationStampNs <= *m_stream.last_published_imu_gravity_anchor_stamp_ns)
  {
    MaybeLogImuGravityDiagnostics();
    return;
  }

  // `imu_gravity` publishes a full sensor_msgs/Imu at fused orientation
  // cadence. Its linear_acceleration field intentionally
  // contains calibrated acceleration including gravity. This is the stream
  // used by monocular-inertial SLAM.
  //
  // BNO08X calibrated acceleration includes gravity, while the linear
  // acceleration report used by `imu` has gravity removed.
  const sensor_msgs::msg::Imu imuGravityMsg =
      BuildImuGravityMessage(m_stream.orientation.stamp, accelSample);
  std::string invalidReason;
  if (!IsImuGravitySampleValid(imuGravityMsg, invalidReason))
  {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), IMU_GRAVITY_SAMPLE_WARN_THROTTLE_MS,
                         "Skipping BNO086 imu_gravity sample: %s", invalidReason.c_str());
    MaybeLogImuGravityDiagnostics();
    return;
  }

  m_imuGravityPublisher->publish(imuGravityMsg);
  m_stream.last_published_imu_gravity_anchor_stamp_ns = orientationStampNs;
  m_diag.rate_health.CountImuGravityPublished();
  MaybeLogImuGravityDiagnostics();
}

void Bno086ImuNode::MaybePublishGravityOnGravityReport(const SensorEvent& event)
{
  if (event.report_id != ReportId::Gravity)
    return;

  if (!m_stream.latest_frame.has_gravity)
    return;

  const geometry_msgs::msg::AccelWithCovarianceStamped gravityMsg =
      BuildGravityMessage(m_stream.gravity.stamp);
  m_gravityPublisher->publish(gravityMsg);
}

void Bno086ImuNode::PublishLatestFrame(const rclcpp::Time& stamp)
{
  if (!(m_stream.latest_frame.has_orientation && m_stream.latest_frame.has_gyro &&
        m_stream.latest_frame.has_linear_accel))
  {
    if (!m_diag.warned_missing_imu_fields)
    {
      RCLCPP_WARN(get_logger(), "Waiting for complete BNO086 IMU fields "
                                "(orientation+gyro+linear_accel)");
      m_diag.warned_missing_imu_fields = true;
    }
    return;
  }

  m_diag.warned_missing_imu_fields = false;

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
  imuMsg.header.frame_id = m_config.frame_id;

  imuMsg.orientation.x = m_stream.latest_frame.orientation_xyzw[0];
  imuMsg.orientation.y = m_stream.latest_frame.orientation_xyzw[1];
  imuMsg.orientation.z = m_stream.latest_frame.orientation_xyzw[2];
  imuMsg.orientation.w = m_stream.latest_frame.orientation_xyzw[3];

  imuMsg.angular_velocity.x = m_stream.latest_frame.gyro_rads[0];
  imuMsg.angular_velocity.y = m_stream.latest_frame.gyro_rads[1];
  imuMsg.angular_velocity.z = m_stream.latest_frame.gyro_rads[2];

  imuMsg.linear_acceleration.x = m_stream.latest_frame.linear_accel_mps2[0];
  imuMsg.linear_acceleration.y = m_stream.latest_frame.linear_accel_mps2[1];
  imuMsg.linear_acceleration.z = m_stream.latest_frame.linear_accel_mps2[2];

  imuMsg.orientation_covariance.fill(0.0);
  imuMsg.angular_velocity_covariance.fill(0.0);
  imuMsg.linear_acceleration_covariance.fill(0.0);

  if (m_stream.latest_frame.has_orientation_covariance)
    SetCovariance(imuMsg.orientation_covariance, m_stream.latest_frame.orientation_cov_rad2);

  if (m_stream.latest_frame.has_gyro_covariance)
    SetCovariance(imuMsg.angular_velocity_covariance, m_stream.latest_frame.gyro_cov_rads2_2);

  if (m_stream.latest_frame.has_linear_accel_covariance)
  {
    SetCovariance(imuMsg.linear_acceleration_covariance,
                  m_stream.latest_frame.linear_accel_cov_mps2_2);
  }

  return imuMsg;
}

sensor_msgs::msg::Imu Bno086ImuNode::BuildImuGravityMessage(
    const rclcpp::Time& stamp, const Bno086ImuGravityAccelSample& accel_sample) const
{
  sensor_msgs::msg::Imu imuGravityMsg;
  imuGravityMsg.header.stamp = stamp;
  imuGravityMsg.header.frame_id = m_config.frame_id;

  imuGravityMsg.orientation.x = m_stream.latest_frame.orientation_xyzw[0];
  imuGravityMsg.orientation.y = m_stream.latest_frame.orientation_xyzw[1];
  imuGravityMsg.orientation.z = m_stream.latest_frame.orientation_xyzw[2];
  imuGravityMsg.orientation.w = m_stream.latest_frame.orientation_xyzw[3];

  imuGravityMsg.angular_velocity.x = m_stream.latest_frame.gyro_rads[0];
  imuGravityMsg.angular_velocity.y = m_stream.latest_frame.gyro_rads[1];
  imuGravityMsg.angular_velocity.z = m_stream.latest_frame.gyro_rads[2];

  imuGravityMsg.linear_acceleration.x = accel_sample.accel_mps2[0];
  imuGravityMsg.linear_acceleration.y = accel_sample.accel_mps2[1];
  imuGravityMsg.linear_acceleration.z = accel_sample.accel_mps2[2];

  imuGravityMsg.orientation_covariance.fill(0.0);
  imuGravityMsg.angular_velocity_covariance.fill(0.0);
  imuGravityMsg.linear_acceleration_covariance.fill(0.0);

  if (m_stream.latest_frame.has_orientation_covariance)
    SetCovariance(imuGravityMsg.orientation_covariance, m_stream.latest_frame.orientation_cov_rad2);

  if (m_stream.latest_frame.has_gyro_covariance)
    SetCovariance(imuGravityMsg.angular_velocity_covariance,
                  m_stream.latest_frame.gyro_cov_rads2_2);

  if (accel_sample.has_covariance)
  {
    SetCovariance(imuGravityMsg.linear_acceleration_covariance, accel_sample.covariance_mps2_2);
  }

  return imuGravityMsg;
}

geometry_msgs::msg::AccelWithCovarianceStamped Bno086ImuNode::BuildGravityMessage(
    const rclcpp::Time& stamp) const
{
  std::optional<OASIS::IMU::Mat3> gravityCovariance;
  if (m_stream.latest_frame.has_gravity_covariance)
    gravityCovariance = m_stream.latest_frame.gravity_cov_mps2_2;

  const PublishedGravityMeasurement gravityMeasurement =
      MakePublishedGravityMeasurement(m_stream.latest_frame.gravity_mps2, gravityCovariance);

  geometry_msgs::msg::AccelWithCovarianceStamped gravityMsg;
  gravityMsg.header.stamp = stamp;
  gravityMsg.header.frame_id = m_config.frame_id;
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
  const std::array<double, 4> predictedOrientation =
      PredictQuaternion(m_stream.latest_frame.orientation_xyzw, m_stream.latest_frame.gyro_rads,
                        m_config.prediction_horizon_sec);

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
  predictedImuMsg.orientation_covariance = PredictedCovarianceFromPresent(
      present_imu.orientation_covariance, m_config.prediction_horizon_sec, sigmaNoiseRad,
      sigmaRmsRad, sigmaBoundRad);

  return predictedImuMsg;
}

oasis_msgs::msg::ImuVr Bno086ImuNode::BuildPredictedVrMessage(
    const sensor_msgs::msg::Imu& present_imu, const sensor_msgs::msg::Imu& predicted_imu) const
{
  oasis_msgs::msg::ImuVr vrMsg;
  const std::uint8_t predictionAccuracy =
      std::min(m_stream.orientation.accuracy, m_stream.gyro.accuracy);
  vrMsg.header = present_imu.header;
  vrMsg.valid = m_stream.latest_frame.has_orientation && m_stream.latest_frame.has_gyro;
  vrMsg.source = m_config.prediction_source;
  vrMsg.prediction_horizon_sec = m_config.prediction_horizon_sec;
  vrMsg.orientation = predicted_imu.orientation;
  vrMsg.orientation_covariance = predicted_imu.orientation_covariance;
  vrMsg.accuracy_status = predictionAccuracy;
  vrMsg.covariance_is_prediction_model_based = m_config.prediction_horizon_sec > 0.0;

  double sigmaNoiseRad = 0.0;
  double sigmaRmsRad = 0.0;
  double sigmaBoundRad = 0.0;
  PredictedCovarianceFromPresent(present_imu.orientation_covariance,
                                 m_config.prediction_horizon_sec, sigmaNoiseRad, sigmaRmsRad,
                                 sigmaBoundRad);
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
  const std::vector<FeatureResponse> featureResponses = m_shtp->TakeFeatureResponses();
  for (const FeatureResponse& featureResponse : featureResponses)
  {
    const auto it = std::find_if(m_startup.latest_feature_responses.begin(),
                                 m_startup.latest_feature_responses.end(),
                                 [&featureResponse](const FeatureResponse& response)
                                 { return response.report_id == featureResponse.report_id; });
    if (it == m_startup.latest_feature_responses.end())
      m_startup.latest_feature_responses.emplace_back(featureResponse);
    else
      *it = featureResponse;
  }

  MaybeLogFeatureSummary();
}

void Bno086ImuNode::MaybeLogFeatureSummary()
{
  if (m_startup.logged_feature_summary)
    return;

  const std::vector<FeatureConfiguration>& configurations = m_shtp->GetFeatureConfigurations();
  if (configurations.empty())
    return;

  const bool receivedAll = m_startup.latest_feature_responses.size() >= configurations.size();
  bool timedOut = false;
  if (!receivedAll && m_startup.feature_configuration_started_at.time_since_epoch().count() != 0)
  {
    const auto elapsedMs =
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - m_startup.feature_configuration_started_at)
            .count();
    timedOut = elapsedMs >= m_config.feature_summary_timeout_ms;
  }

  if (!receivedAll && !timedOut)
    return;

  LogFeatureSummary();
  m_startup.logged_feature_summary = true;
}

void Bno086ImuNode::LogFeatureSummary() const
{
  const std::vector<FeatureConfiguration>& configurations = m_shtp->GetFeatureConfigurations();
  const std::string summary =
      BuildFeatureSummary(configurations, m_startup.latest_feature_responses);
  RCLCPP_INFO(get_logger(), "%s", summary.c_str());
}

void Bno086ImuNode::MaybeLogImuGravityDiagnostics()
{
  const auto now = std::chrono::steady_clock::now();
  if (!m_diag.rate_health.ShouldLog(now, m_config.diagnostics_log_period_ms))
    return;

  const Bno086RateSnapshot rates = m_diag.rate_health.BuildSnapshot(now);
  MaybeEmitImuGravityDiagnosticsLog(rates);
  m_diag.rate_health.MarkSnapshotLogged(now);
}

void Bno086ImuNode::RecordDrainThroughputDiagnostics(const Bno086DrainCounters& counters,
                                                     Bno086DrainAction exit_action,
                                                     bool /*hintn_asserted_after_exit*/,
                                                     std::uint32_t drain_duration_us,
                                                     std::uint32_t /*pending_queue_depth_at_exit*/)
{
  m_diag.drain_health.Record(counters, exit_action, drain_duration_us);
}

void Bno086ImuNode::MaybeEmitImuGravityDiagnosticsLog(const Bno086RateSnapshot& rates)
{
  const bool unhealthy = IsBno086DiagnosticsUnhealthy(rates);
  const Bno086TransportStats transportStats = m_transport->GetStats();

  RCLCPP_DEBUG(get_logger(),
               "BNO086 rates: accel=%.0f gyro=%.0f rot=%.0f lin=%.0f grav=%.0f "
               "imu_g=%.0f imu=%.0f",
               rates.decoded_hz[0], rates.decoded_hz[1], rates.decoded_hz[2], rates.decoded_hz[3],
               rates.decoded_hz[4], rates.imu_gravity_hz, rates.imu_hz);
  RCLCPP_DEBUG(get_logger(),
               "BNO086 drain: n=%llu pkt=%.1f/%u evt=%.1f/%u zero=%llu "
               "dur=%.0f/%.0f cap=%llu/%llu err=%llu/%llu az=%llu bad=%llu",
               static_cast<unsigned long long>(m_diag.drain_health.Drains()),
               m_diag.drain_health.PhysicalPacketsMean(), m_diag.drain_health.PhysicalPacketsMax(),
               m_diag.drain_health.SensorEventsMean(), m_diag.drain_health.SensorEventsMax(),
               static_cast<unsigned long long>(m_diag.drain_health.AllZeroBackoffCount()),
               m_diag.drain_health.DrainDurationMeanMs(),
               static_cast<double>(m_diag.drain_health.DrainDurationMaxUs()) / 1.0e3,
               static_cast<unsigned long long>(m_diag.drain_health.PhysicalPacketCapHitCount()),
               static_cast<unsigned long long>(m_diag.drain_health.PollIterationCapHitCount()),
               static_cast<unsigned long long>(m_diag.drain_health.NoProgressDrainCount()),
               static_cast<unsigned long long>(m_diag.drain_health.TransportErrorCount()),
               static_cast<unsigned long long>(m_diag.drain_health.AllZeroBudgetHitCount()),
               static_cast<unsigned long long>(transportStats.invalid_full_packet_count));

  if (m_diag.was_unhealthy && !unhealthy)
  {
    RCLCPP_INFO(get_logger(), "BNO086 imu_gravity recovered: imu_gravity_rate_hz=%.2f",
                rates.imu_gravity_hz);
  }

  m_diag.was_unhealthy = unhealthy;
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
      return RateHzToIntervalUs(m_reports.accelerometer_rate_hz);
    case ReportId::GyroscopeCalibrated:
      return RateHzToIntervalUs(m_reports.gyro_rate_hz);
    case ReportId::RotationVector:
      return RateHzToIntervalUs(m_reports.rotation_vector_rate_hz);
    case ReportId::LinearAcceleration:
      return RateHzToIntervalUs(m_reports.linear_acceleration_rate_hz);
    case ReportId::Gravity:
      return RateHzToIntervalUs(m_reports.gravity_rate_hz);
    default:
      break;
  }

  return std::nullopt;
}

std::optional<FeatureResponse> Bno086ImuNode::LatestFeatureResponse(ReportId report_id) const
{
  const auto it = std::find_if(
      m_startup.latest_feature_responses.begin(), m_startup.latest_feature_responses.end(),
      [report_id](const FeatureResponse& response) { return response.report_id == report_id; });

  if (it == m_startup.latest_feature_responses.end())
    return std::nullopt;

  return *it;
}

bool Bno086ImuNode::IsBno086DiagnosticsUnhealthy(const Bno086RateSnapshot& rates) const
{
  const Bno086ExpectedRates expectedRates{
      m_reports.accelerometer_rate_hz, m_reports.gyro_rate_hz, m_reports.rotation_vector_rate_hz,
      m_reports.linear_acceleration_rate_hz, m_reports.gravity_rate_hz};
  return m_diag.rate_health.HasRateFailure(rates, expectedRates, MIN_HEALTHY_RATE_FRACTION) ||
         m_diag.drain_health.HasSafetyFailure();
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
      static_cast<int64_t>(linearIntervalUs.value_or(DEFAULT_REPORT_INTERVAL_US)) * 1'000,
      static_cast<int64_t>(gyroIntervalUs.value_or(DEFAULT_REPORT_INTERVAL_US)) * 1'000,
      static_cast<int64_t>(orientationIntervalUs.value_or(DEFAULT_REPORT_INTERVAL_US)) * 1'000,
      static_cast<int64_t>(MIN_COHERENT_SAMPLE_SPAN_US) * 1'000);

  return static_cast<std::uint32_t>(toleranceNs / 1'000);
}

int64_t Bno086ImuNode::ImuGravityMaxOrientationAgeNs() const
{
  const std::optional<std::uint32_t> intervalUs =
      EffectiveReportIntervalUs(ReportId::RotationVector);
  return EffectiveMaxPastAgeNs(
      static_cast<int64_t>(m_config.imu_gravity_max_orientation_age_ms * 1.0e6),
      static_cast<int64_t>(intervalUs.value_or(DEFAULT_REPORT_INTERVAL_US)) * 1'000, 80'000'000);
}

int64_t Bno086ImuNode::ImuGravityMaxGyroAgeNs() const
{
  const std::optional<std::uint32_t> intervalUs =
      EffectiveReportIntervalUs(ReportId::GyroscopeCalibrated);
  return EffectiveMaxPastAgeNs(
      static_cast<int64_t>(m_config.imu_gravity_max_gyro_age_ms * 1.0e6),
      static_cast<int64_t>(intervalUs.value_or(DEFAULT_REPORT_INTERVAL_US)) * 1'000, 80'000'000);
}

int64_t Bno086ImuNode::ReportFutureToleranceNs(ReportId report_id) const
{
  const std::optional<std::uint32_t> intervalUs = EffectiveReportIntervalUs(report_id);
  if (intervalUs.has_value() && *intervalUs > 0)
    return static_cast<int64_t>(*intervalUs) * 1'000;

  return DurationNsFromUs(CoreCoherenceToleranceUs());
}

void Bno086ImuNode::RecordImuGravityAccelSample(const rclcpp::Time& sample_stamp)
{
  Bno086ImuGravityAccelSample sample;
  sample.has_sample = true;
  sample.stamp_ns = sample_stamp.nanoseconds();
  sample.accel_mps2 = m_stream.latest_frame.accel_mps2;
  sample.covariance_mps2_2 = m_stream.latest_frame.accel_cov_mps2_2;
  sample.has_covariance = m_stream.latest_frame.has_accel_covariance;
  sample.sequence = m_stream.imu_gravity.sequence;
  sample.accuracy = m_stream.imu_gravity.accuracy;

  m_stream.imu_gravity_accel_history.Push(sample);
}

std::optional<Bno086ImuGravityAccelSample> Bno086ImuNode::SelectImuGravityAccelSample(
    int64_t anchor_stamp_ns, int64_t future_tolerance_ns) const
{
  return m_stream.imu_gravity_accel_history.SelectAtOrBefore(anchor_stamp_ns, future_tolerance_ns);
}

Bno086ImuNode::CoreFrameSignature Bno086ImuNode::LatestCoreSignature() const
{
  CoreFrameSignature signature;
  signature.orientation_sequence = m_stream.orientation.sequence;
  signature.gyro_sequence = m_stream.gyro.sequence;
  signature.linear_accel_sequence = m_stream.linear_accel.sequence;
  signature.orientation_stamp_ns = m_stream.orientation.stamp.nanoseconds();
  signature.gyro_stamp_ns = m_stream.gyro.stamp.nanoseconds();
  signature.linear_accel_stamp_ns = m_stream.linear_accel.stamp.nanoseconds();
  return signature;
}

rclcpp::Time Bno086ImuNode::LatestCoreStamp() const
{
  const int64_t orientationNs = m_stream.orientation.stamp.nanoseconds();
  const int64_t gyroNs = m_stream.gyro.stamp.nanoseconds();
  const int64_t linearAccelNs = m_stream.linear_accel.stamp.nanoseconds();
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

      m_stream.latest_frame.orientation_xyzw[0] = QToDouble(
          event.values[static_cast<std::size_t>(RotationVectorValueIndex::QuaternionI)], 14);
      m_stream.latest_frame.orientation_xyzw[1] = QToDouble(
          event.values[static_cast<std::size_t>(RotationVectorValueIndex::QuaternionJ)], 14);
      m_stream.latest_frame.orientation_xyzw[2] = QToDouble(
          event.values[static_cast<std::size_t>(RotationVectorValueIndex::QuaternionK)], 14);
      m_stream.latest_frame.orientation_xyzw[3] = QToDouble(
          event.values[static_cast<std::size_t>(RotationVectorValueIndex::QuaternionReal)], 14);
      NormalizeQuaternion(m_stream.latest_frame.orientation_xyzw);
      m_stream.latest_frame.has_orientation = true;
      m_stream.orientation.has_sample = true;
      m_stream.orientation.stamp = sample_stamp;
      m_stream.orientation.sequence = event.sequence;
      m_stream.orientation.accuracy = event.accuracy;

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
      m_stream.latest_frame.orientation_cov_rad2 = covariancePolicy.covariance_rad2;
      m_stream.latest_frame.has_orientation_covariance = true;
      MaybeLogOrientationCovariancePolicy(covariancePolicy);
      break;
    }

    case ReportId::GyroscopeCalibrated:
      m_stream.latest_frame.gyro_rads[0] = QToDouble(event.values[0], 9);
      m_stream.latest_frame.gyro_rads[1] = QToDouble(event.values[1], 9);
      m_stream.latest_frame.gyro_rads[2] = QToDouble(event.values[2], 9);
      m_stream.latest_frame.has_gyro = true;
      m_stream.gyro.has_sample = true;
      m_stream.gyro.stamp = sample_stamp;
      m_stream.gyro.sequence = event.sequence;
      m_stream.gyro.accuracy = event.accuracy;

      m_stream.latest_frame.gyro_cov_rads2_2 =
          CovarianceFromAccuracyBucket(event.accuracy, 1.2, 0.5, 0.18, 0.06);
      m_stream.latest_frame.has_gyro_covariance = true;
      break;

    case ReportId::LinearAcceleration:
      m_stream.latest_frame.linear_accel_mps2[0] = QToDouble(event.values[0], 8);
      m_stream.latest_frame.linear_accel_mps2[1] = QToDouble(event.values[1], 8);
      m_stream.latest_frame.linear_accel_mps2[2] = QToDouble(event.values[2], 8);
      m_stream.latest_frame.has_linear_accel = true;
      m_stream.linear_accel.has_sample = true;
      m_stream.linear_accel.stamp = sample_stamp;
      m_stream.linear_accel.sequence = event.sequence;
      m_stream.linear_accel.accuracy = event.accuracy;

      m_stream.latest_frame.linear_accel_cov_mps2_2 =
          CovarianceFromAccuracyBucket(event.accuracy, 4.0, 2.0, 0.8, 0.25);
      m_stream.latest_frame.has_linear_accel_covariance = true;
      break;

    case ReportId::Accelerometer:
      m_stream.latest_frame.accel_mps2[0] = QToDouble(event.values[0], 8);
      m_stream.latest_frame.accel_mps2[1] = QToDouble(event.values[1], 8);
      m_stream.latest_frame.accel_mps2[2] = QToDouble(event.values[2], 8);
      m_stream.latest_frame.has_accel = true;
      m_stream.imu_gravity.has_sample = true;
      m_stream.imu_gravity.stamp = sample_stamp;
      m_stream.imu_gravity.sequence = event.sequence;
      m_stream.imu_gravity.accuracy = event.accuracy;

      m_stream.latest_frame.accel_cov_mps2_2 =
          CovarianceFromAccuracyBucket(event.accuracy, 4.0, 2.0, 0.8, 0.25);
      m_stream.latest_frame.has_accel_covariance = true;
      RecordImuGravityAccelSample(sample_stamp);
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
      m_stream.latest_frame.gravity_mps2 = CanonicalizeGravityVector(rawGravityMps2);
      m_stream.latest_frame.gravity_cov_mps2_2 =
          CovarianceFromAccuracyBucket(event.accuracy, 1.0, 0.5, 0.2, 0.08);
      m_stream.latest_frame.has_gravity = true;
      m_stream.latest_frame.has_gravity_covariance = true;
      m_stream.gravity.has_sample = true;
      m_stream.gravity.stamp = sample_stamp;
      m_stream.gravity.sequence = event.sequence;
      m_stream.gravity.accuracy = event.accuracy;
      break;
    }

    default:
      break;
  }
}

std::optional<int64_t> Bno086ImuNode::FinalizeEventStampNs(
    const SensorEvent& event,
    const rclcpp::Time& interrupt_ros_at,
    std::optional<int64_t> expected_interval_ns)
{
  const Bno086TimestampCadenceResult result =
      m_timestampCadence->Finalize(event, interrupt_ros_at.nanoseconds(), expected_interval_ns);
  if (result.monotonic_guard_adjusted && result.stamp_ns.has_value() &&
      result.last_stamp_ns.has_value())
  {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                         "BNO086 cadence timestamp guard adjusted report=%s "
                         "sequence=%u candidate_ns=%lld last_ns=%lld final_ns=%lld "
                         "duplicate_sequence=%s sequence_delta=%u",
                         ReportName(event.report_id), event.sequence,
                         static_cast<long long>(result.candidate_stamp_ns),
                         static_cast<long long>(*result.last_stamp_ns),
                         static_cast<long long>(*result.stamp_ns),
                         result.duplicate_sequence ? "true" : "false", result.sequence_delta);
  }

  return result.stamp_ns;
}

void Bno086ImuNode::MaybeLogOrientationCovariancePolicy(
    const OrientationCovariancePolicyResult& covariance_policy)
{
  const bool sourceChanged =
      !m_covariance_log.logged_orientation_covariance_source ||
      covariance_policy.source != m_covariance_log.last_orientation_covariance_source;
  const bool bucketChanged =
      !m_covariance_log.logged_orientation_covariance_source ||
      covariance_policy.accuracy_bucket != m_covariance_log.last_orientation_accuracy_bucket;

  if (sourceChanged || bucketChanged)
  {
    RCLCPP_INFO(
        get_logger(),
        "BNO086 orientation covariance policy: report=%s bucket=%u source=%s "
        "raw_estimate_q12=%d estimate_rad=%.4f rejection=%s sigma_rad=%.4f "
        "sigma_deg=%.2f",
        ORIENTATION_REPORT_SOURCE, static_cast<unsigned>(covariance_policy.accuracy_bucket),
        OrientationCovarianceSourceName(covariance_policy.source),
        static_cast<int>(covariance_policy.raw_accuracy_estimate_q12),
        covariance_policy.accuracy_estimate_rad,
        OrientationCovarianceEstimateRejectionReasonName(covariance_policy.rejection_reason),
        covariance_policy.sigma_rad, covariance_policy.sigma_rad * 180.0 / PI_RAD);

    m_covariance_log.logged_orientation_covariance_source = true;
    m_covariance_log.last_orientation_covariance_source = covariance_policy.source;
    m_covariance_log.last_orientation_accuracy_bucket = covariance_policy.accuracy_bucket;
  }
}
