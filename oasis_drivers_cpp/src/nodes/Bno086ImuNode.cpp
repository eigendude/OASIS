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
#include <cstddef>
#include <cstdio>
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

constexpr double DEFAULT_REPORT_RATE_HZ = 100.0;

constexpr const char* IMU_TOPIC = "imu";
constexpr const char* IMU_PREDICTED_TOPIC = "imu_predicted";
constexpr const char* IMU_VR_TOPIC = "imu_vr";
constexpr const char* GRAVITY_TOPIC = "gravity";
constexpr const char* IMU_GRAVITY_TOPIC = "imu_gravity";
constexpr const char* DEFAULT_FRAME_ID = "imu_link";

constexpr int INTERRUPT_WAIT_TIMEOUT_MS = 20;
constexpr int PACKET_READ_TIMEOUT_MS = 5;
constexpr int MAX_PACKETS_PER_INTERRUPT = 1024;
constexpr double MAX_DRAIN_DURATION_MS = 50.0;
constexpr int TIMEOUT_RETRIES_WHILE_INTERRUPT_ASSERTED = 3;
constexpr int TIMEOUT_RETRY_SLEEP_US = 100;
constexpr std::uint64_t STARTUP_BACKLOG_DRAIN_COUNT = 5;
constexpr std::uint64_t STUCK_INTERRUPT_RECOVERY_CANDIDATE_THRESHOLD = 5;
constexpr double POLL_TIMEOUT_SLOP_MS = 1.0;

constexpr std::uint32_t MIN_COHERENT_SAMPLE_SPAN_US = 3'000;
constexpr std::uint32_t MAX_COHERENT_SAMPLE_SPAN_US = 20'000;
constexpr double DEFAULT_PREDICTION_HORIZON_SEC = 0.0;

// Maximum nearby orientation age accepted for imu_gravity composition
constexpr double DEFAULT_IMU_GRAVITY_MAX_ORIENTATION_AGE_MS = 25.0;

// Maximum nearby gyro age accepted for imu_gravity composition
constexpr double DEFAULT_IMU_GRAVITY_MAX_GYRO_AGE_MS = 25.0;

// Practical calibrated acceleration target requested from the BNO08X
constexpr double IMU_GRAVITY_TARGET_RATE_HZ = 100.0;

// Lower bound used to warn when the accepted feature rate misses 100 Hz
constexpr double MIN_NEAR_TARGET_FEATURE_RATE_HZ = 95.0;

// Upper bound used to warn when the accepted feature rate misses 100 Hz
constexpr double MAX_NEAR_TARGET_FEATURE_RATE_HZ = 105.0;

// Plausibility bound for gravity-included calibrated acceleration samples
constexpr double MAX_IMU_GRAVITY_ACCEL_MAGNITUDE_MPS2 = 200.0;

// Plausibility bound for calibrated gyro samples
constexpr double MAX_IMU_GRAVITY_GYRO_MAGNITUDE_RADS = 100.0;
constexpr double PI_RAD = 3.14159265358979323846;
constexpr const char* ORIENTATION_REPORT_SOURCE = "rotation_vector";
constexpr int ORIENTATION_COVARIANCE_LOG_THROTTLE_MS = 5'000;
constexpr int IMU_GRAVITY_DIAGNOSTICS_LOG_MS = 5'000;
constexpr int ACCEL_TIMESTAMP_REPAIR_LOG_MS = 5'000;

bool IsFiniteArray(const std::array<double, 9>& values)
{
  return std::all_of(values.begin(), values.end(),
                     [](double value) { return std::isfinite(value); });
}

double VectorMagnitude(double x, double y, double z)
{
  return std::sqrt(x * x + y * y + z * z);
}

std::size_t ReportIndex(ReportId report_id)
{
  return static_cast<std::size_t>(static_cast<std::uint8_t>(report_id));
}

const char* ContinuationResetReasonName(ContinuationResetReason reason)
{
  switch (reason)
  {
    case ContinuationResetReason::None:
      return "none";
    case ContinuationResetReason::EmptyPayload:
      return "empty_payload";
    case ContinuationResetReason::CommandHeaderOnly:
      return "command_header_only";
    case ContinuationResetReason::ShtpHeaderPrefix:
      return "shtp_header_prefix";
    case ContinuationResetReason::MaxBytesExceeded:
      return "max_bytes_exceeded";
    case ContinuationResetReason::OrphanContinuation:
      return "orphan_continuation";
    case ContinuationResetReason::EmbeddedShtpHeader:
      return "embedded_shtp_header";
    default:
      break;
  }

  return "unknown";
}

const char* PollStatusName(Bno086Shtp::PollStatus status)
{
  switch (status)
  {
    case Bno086Shtp::PollStatus::Timeout:
      return "timeout";
    case Bno086Shtp::PollStatus::TransportError:
      return "transport_error";
    case Bno086Shtp::PollStatus::PacketHandled:
      return "packet_handled";
    case Bno086Shtp::PollStatus::SensorEvent:
      return "sensor_event";
    default:
      break;
  }

  return "unknown";
}

std::string ContinuationPayloadPrefixHex(const ShtpDiagnostics& diagnostics)
{
  std::string prefix;
  for (std::size_t i = 0; i < diagnostics.latest_continuation_reset_payload_prefix_size; ++i)
  {
    char byteText[4];
    std::snprintf(byteText, sizeof(byteText), "%02X",
                  diagnostics.latest_continuation_reset_payload_prefix[i]);

    if (!prefix.empty())
      prefix += " ";

    prefix += byteText;
  }

  return prefix;
}
} // namespace

Bno086ImuNode::Bno086ImuNode() : rclcpp::Node(NODE_NAME)
{
  declare_parameter("i2c_device", std::string(DEFAULT_I2C_DEVICE));
  declare_parameter("i2c_address", static_cast<int>(DEFAULT_I2C_ADDRESS));
  declare_parameter("int_gpio", DEFAULT_INT_GPIO);
  declare_parameter("report_rate_hz", DEFAULT_REPORT_RATE_HZ);
  declare_parameter("prediction_horizon_sec", DEFAULT_PREDICTION_HORIZON_SEC);
  declare_parameter("imu_gravity_max_orientation_age_ms",
                    DEFAULT_IMU_GRAVITY_MAX_ORIENTATION_AGE_MS);
  declare_parameter("imu_gravity_max_gyro_age_ms", DEFAULT_IMU_GRAVITY_MAX_GYRO_AGE_MS);
  declare_parameter("bno086_packet_read_timeout_ms", PACKET_READ_TIMEOUT_MS);
  declare_parameter("bno086_max_packets_per_interrupt", MAX_PACKETS_PER_INTERRUPT);
  declare_parameter("bno086_max_drain_duration_ms", MAX_DRAIN_DURATION_MS);
  declare_parameter("bno086_timeout_retries_while_interrupt_asserted",
                    TIMEOUT_RETRIES_WHILE_INTERRUPT_ASSERTED);
  declare_parameter("bno086_timeout_retry_sleep_us", TIMEOUT_RETRY_SLEEP_US);

  declare_parameter("frame_id", std::string(DEFAULT_FRAME_ID));

  const std::string i2cDevice = get_parameter("i2c_device").as_string();
  const std::uint8_t i2cAddress = static_cast<std::uint8_t>(get_parameter("i2c_address").as_int());
  const int intGpio = get_parameter("int_gpio").as_int();
  const double reportRateHz = std::max(get_parameter("report_rate_hz").as_double(), 1.0);
  m_predictionHorizonSec = std::max(get_parameter("prediction_horizon_sec").as_double(), 0.0);
  m_imuGravityMaxOrientationAgeMs =
      std::max(get_parameter("imu_gravity_max_orientation_age_ms").as_double(), 0.0);
  m_imuGravityMaxGyroAgeMs =
      std::max(get_parameter("imu_gravity_max_gyro_age_ms").as_double(), 0.0);
  m_packetReadTimeoutMs =
      std::max(0, static_cast<int>(get_parameter("bno086_packet_read_timeout_ms").as_int()));
  m_maxPacketsPerInterrupt =
      std::max(1, static_cast<int>(get_parameter("bno086_max_packets_per_interrupt").as_int()));
  m_maxDrainDurationMs = std::max(1.0, get_parameter("bno086_max_drain_duration_ms").as_double());
  m_timeoutRetriesWhileInterruptAsserted = std::max(
      0,
      static_cast<int>(get_parameter("bno086_timeout_retries_while_interrupt_asserted").as_int()));
  m_timeoutRetrySleepUs =
      std::max(0, static_cast<int>(get_parameter("bno086_timeout_retry_sleep_us").as_int()));
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

  if (!m_shtp->Configure(shtpConfig))
  {
    RCLCPP_ERROR(get_logger(), "Failed to send initial BNO086 Set Feature commands");
    throw std::runtime_error("Failed to configure BNO086 reports");
  }

  if (intGpio == DEFAULT_INT_GPIO)
  {
    RCLCPP_INFO(get_logger(), "BNO086 INT uses GPIO%d (Raspberry Pi header pin 16), active low",
                intGpio);
  }

  RCLCPP_INFO(get_logger(),
              "BNO086 opened on %s (0x%02X), int_gpio=%d active_low, report_rate_hz=%.1f "
              "(requested sensor report rate)",
              i2cDevice.c_str(), static_cast<unsigned>(i2cAddress), intGpio, reportRateHz);
  RCLCPP_INFO(get_logger(), "ROS publication is interrupt-driven from GPIO packet drains");
  RCLCPP_INFO(get_logger(), "Predicted orientation output uses %s with prediction_horizon_sec=%.4f",
              m_predictionSource.c_str(), m_predictionHorizonSec);
  RCLCPP_INFO(get_logger(),
              "BNO086 imu_gravity uses calibrated acceleration cadence with "
              "orientation_max_age_ms=%.1f gyro_max_age_ms=%.1f",
              m_imuGravityMaxOrientationAgeMs, m_imuGravityMaxGyroAgeMs);
  RCLCPP_INFO(get_logger(),
              "BNO086 packet drain config: packet_read_timeout_ms=%d "
              "max_packets_per_interrupt=%d safety_cap max_drain_duration_ms=%.1f "
              "timeout_retries_while_interrupt_asserted=%d timeout_retry_sleep_us=%d",
              m_packetReadTimeoutMs, m_maxPacketsPerInterrupt, m_maxDrainDurationMs,
              m_timeoutRetriesWhileInterruptAsserted, m_timeoutRetrySleepUs);
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
  }
}

void Bno086ImuNode::DrainPacketsForInterrupt(
    const std::chrono::steady_clock::time_point& interrupt_steady_at,
    const rclcpp::Time& interrupt_ros_at)
{
  const auto drainStartSteady = std::chrono::steady_clock::now();

  ++m_interruptDrainDiagnostics.interrupt_received_count;
  ++m_interruptDrainDiagnostics.drain_cycles_started;
  m_interruptDrainDiagnostics.latest_interrupt_to_last_packet_ms = 0.0;
  m_interruptDrainDiagnostics.max_drain_duration_ms_config = m_maxDrainDurationMs;

  const bool startupBacklogDrain =
      !m_loggedCommEstablished ||
      m_interruptDrainDiagnostics.drain_cycles_started <= STARTUP_BACKLOG_DRAIN_COUNT;

  std::uint64_t packetsThisDrain = 0;
  std::uint64_t sensorEventsThisDrain = 0;
  bool exitedTimeout = false;
  bool exitedTimeoutAfterProgress = false;
  bool exitedTransportError = false;
  bool exitedDurationBudget = false;
  bool exitedMaxPackets = false;
  bool recordedFirstPoll = false;
  int timeoutRetriesWhileAsserted = 0;

  while (m_running.load())
  {
    const auto packetNowSteady = std::chrono::steady_clock::now();
    const double drainElapsedMs =
        static_cast<double>(
            std::chrono::duration_cast<std::chrono::nanoseconds>(packetNowSteady - drainStartSteady)
                .count()) /
        1.0e6;
    const double remainingDrainBudgetMs = m_maxDrainDurationMs - drainElapsedMs;
    if (remainingDrainBudgetMs <= 0.0)
    {
      if (m_interruptGpio.IsAssertedLow())
        exitedDurationBudget = true;
      break;
    }

    if (packetsThisDrain >= static_cast<std::uint64_t>(m_maxPacketsPerInterrupt))
    {
      if (m_interruptGpio.IsAssertedLow())
        exitedMaxPackets = true;
      break;
    }

    const auto packetDeltaNs =
        std::chrono::duration_cast<std::chrono::nanoseconds>(packetNowSteady - interrupt_steady_at)
            .count();
    const rclcpp::Time packetRosAt = interrupt_ros_at + rclcpp::Duration(0, packetDeltaNs);

    if (!recordedFirstPoll)
    {
      const double interruptToFirstPollMs = static_cast<double>(packetDeltaNs) / 1.0e6;
      m_interruptDrainDiagnostics.latest_interrupt_to_first_poll_ms = interruptToFirstPollMs;
      m_interruptDrainDiagnostics.max_interrupt_to_first_poll_ms = std::max(
          m_interruptDrainDiagnostics.max_interrupt_to_first_poll_ms, interruptToFirstPollMs);

      if (interruptToFirstPollMs > 10.0)
      {
        ++m_interruptDrainDiagnostics.interrupt_response_over_10ms;
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), IMU_GRAVITY_DIAGNOSTICS_LOG_MS,
            "BNO086 interrupt response exceeded 10 ms; delayed H_INTN servicing may cause "
            "BNO08X retries or processing starvation: interrupt_to_first_poll_ms=%.3f",
            interruptToFirstPollMs);
      }

      recordedFirstPoll = true;
    }

    std::vector<TimestampedSensorEvent> events;
    const int pollTimeoutMs = std::max(
        0, std::min(m_packetReadTimeoutMs, static_cast<int>(std::ceil(remainingDrainBudgetMs))));
    const auto pollStartSteady = std::chrono::steady_clock::now();
    const Bno086Shtp::PollStatus pollStatus =
        m_shtp->Poll(events, pollTimeoutMs, packetRosAt.nanoseconds());
    const auto pollDoneSteady = std::chrono::steady_clock::now();
    const double pollDurationMs =
        static_cast<double>(
            std::chrono::duration_cast<std::chrono::nanoseconds>(pollDoneSteady - pollStartSteady)
                .count()) /
        1.0e6;
    RecordPollDuration(m_interruptDrainDiagnostics, pollDurationMs, pollTimeoutMs,
                       POLL_TIMEOUT_SLOP_MS);

    const bool hintnAssertedAfterPoll = m_interruptGpio.IsAssertedLow();
    const double longPollThresholdMs =
        std::max(10.0, static_cast<double>(m_packetReadTimeoutMs) + 5.0);
    if (pollDurationMs > longPollThresholdMs)
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), IMU_GRAVITY_DIAGNOSTICS_LOG_MS,
                           "BNO086 Poll exceeded timeout expectation: poll_duration_ms=%.3f "
                           "packet_read_timeout_ms=%d poll_timeout_ms=%d poll_status=%s "
                           "hintn_asserted_after_poll=%s packets_this_drain=%llu "
                           "sensor_events_this_drain=%llu",
                           pollDurationMs, m_packetReadTimeoutMs, pollTimeoutMs,
                           PollStatusName(pollStatus), hintnAssertedAfterPoll ? "true" : "false",
                           static_cast<unsigned long long>(packetsThisDrain),
                           static_cast<unsigned long long>(sensorEventsThisDrain));
    }

    if (pollStatus == Bno086Shtp::PollStatus::Timeout)
    {
      const bool madeProgress = packetsThisDrain > 0 || sensorEventsThisDrain > 0;
      const TimeoutRetryDecision retryDecision =
          HandleDrainTimeout(m_interruptDrainDiagnostics, hintnAssertedAfterPoll, madeProgress,
                             timeoutRetriesWhileAsserted, m_timeoutRetriesWhileInterruptAsserted);

      if (retryDecision.retry)
      {
        ++timeoutRetriesWhileAsserted;
        if (m_timeoutRetrySleepUs > 0)
          std::this_thread::sleep_for(std::chrono::microseconds(m_timeoutRetrySleepUs));

        continue;
      }

      exitedTimeout = retryDecision.exit_timeout;
      exitedTimeoutAfterProgress = madeProgress && retryDecision.exit_timeout;
      break;
    }

    if (pollStatus == Bno086Shtp::PollStatus::TransportError)
    {
      exitedTransportError = true;
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "BNO086 transport error while draining interrupt data");
      break;
    }

    ++packetsThisDrain;
    const auto packetHandledSteady = std::chrono::steady_clock::now();
    const auto handledDeltaNs = std::chrono::duration_cast<std::chrono::nanoseconds>(
                                    packetHandledSteady - interrupt_steady_at)
                                    .count();
    const double interruptToLastPacketMs = static_cast<double>(handledDeltaNs) / 1.0e6;
    m_interruptDrainDiagnostics.latest_interrupt_to_last_packet_ms = interruptToLastPacketMs;
    m_interruptDrainDiagnostics.max_interrupt_to_last_packet_ms = std::max(
        m_interruptDrainDiagnostics.max_interrupt_to_last_packet_ms, interruptToLastPacketMs);

    MaybeLogFeatureResponses();

    if (pollStatus == Bno086Shtp::PollStatus::SensorEvent)
    {
      sensorEventsThisDrain += events.size();
      for (const TimestampedSensorEvent& timestamped : events)
      {
        const rclcpp::Time rawEventStamp(timestamped.stamp_ns, RCL_ROS_TIME);
        const rclcpp::Time eventStamp = NormalizeEventStamp(timestamped.event, rawEventStamp);
        ApplyEvent(timestamped.event, eventStamp);
        MaybePublishOnLinearAcceleration(timestamped.event);
        MaybePublishImuGravityOnAccelerometer(timestamped.event);
      }
    }

    if (!m_interruptGpio.IsAssertedLow() && pollStatus == Bno086Shtp::PollStatus::PacketHandled)
      break;
  }

  m_interruptDrainDiagnostics.latest_timeout_retries_while_hintn_asserted =
      static_cast<std::uint64_t>(timeoutRetriesWhileAsserted);

  const auto drainDoneSteady = std::chrono::steady_clock::now();
  const double drainCycleMs =
      static_cast<double>(
          std::chrono::duration_cast<std::chrono::nanoseconds>(drainDoneSteady - drainStartSteady)
              .count()) /
      1.0e6;

  RecordDrainDuration(m_interruptDrainDiagnostics, drainCycleMs, packetsThisDrain,
                      sensorEventsThisDrain, m_maxDrainDurationMs);

  if (drainCycleMs > 1.0)
    ++m_interruptDrainDiagnostics.drain_cycles_over_1ms;
  if (drainCycleMs > 5.0)
    ++m_interruptDrainDiagnostics.drain_cycles_over_5ms;
  if (drainCycleMs > 10.0)
    ++m_interruptDrainDiagnostics.drain_cycles_over_10ms;
  if (drainCycleMs > 20.0)
    ++m_interruptDrainDiagnostics.drain_cycles_over_20ms;

  ++m_interruptDrainDiagnostics.drain_cycles_completed;
  m_interruptDrainDiagnostics.packets_per_drain_latest = packetsThisDrain;
  m_interruptDrainDiagnostics.packets_per_drain_total += packetsThisDrain;
  m_interruptDrainDiagnostics.packets_per_drain_max =
      std::max(m_interruptDrainDiagnostics.packets_per_drain_max, packetsThisDrain);
  m_interruptDrainDiagnostics.sensor_events_per_drain_latest = sensorEventsThisDrain;
  m_interruptDrainDiagnostics.sensor_events_per_drain_max =
      std::max(m_interruptDrainDiagnostics.sensor_events_per_drain_max, sensorEventsThisDrain);

  if (exitedTransportError)
    ++m_interruptDrainDiagnostics.drain_exited_transport_error;
  if (exitedDurationBudget)
    ++m_interruptDrainDiagnostics.drain_exited_duration_budget;
  if (exitedMaxPackets)
    ++m_interruptDrainDiagnostics.drain_exited_max_packets;
  if (sensorEventsThisDrain == 0)
    ++m_interruptDrainDiagnostics.drain_exited_no_events;
  if (startupBacklogDrain)
    ++m_interruptDrainDiagnostics.startup_backlog_drains;

  const bool hintnAssertedAtExit = m_interruptGpio.IsAssertedLow();
  if (!hintnAssertedAtExit)
    ++m_interruptDrainDiagnostics.drain_exited_int_deasserted;

  RecordDrainExitHintnState(m_interruptDrainDiagnostics, hintnAssertedAtExit,
                            STUCK_INTERRUPT_RECOVERY_CANDIDATE_THRESHOLD);

  if (hintnAssertedAtExit &&
      ((exitedTimeout && !exitedTimeoutAfterProgress) || exitedMaxPackets || exitedDurationBudget))
  {
    const char* exitReason = "timeout";
    if (exitedDurationBudget)
      exitReason = "duration_budget";
    else if (exitedMaxPackets)
      exitReason = "max_packets";

    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), IMU_GRAVITY_DIAGNOSTICS_LOG_MS,
                         "BNO086 drain exited while H_INTN remained asserted: reason=%s "
                         "packets_this_drain=%llu sensor_events_this_drain=%llu "
                         "drain_duration_ms=%.3f packets_per_ms=%.3f "
                         "interrupt_to_last_packet_ms=%.3f packet_read_timeout_ms=%d "
                         "max_packets_per_interrupt=%d max_drain_duration_ms=%.1f "
                         "timeout_retries_used=%d",
                         exitReason, static_cast<unsigned long long>(packetsThisDrain),
                         static_cast<unsigned long long>(sensorEventsThisDrain),
                         m_interruptDrainDiagnostics.latest_drain_duration_ms,
                         m_interruptDrainDiagnostics.packets_per_ms_latest,
                         m_interruptDrainDiagnostics.latest_interrupt_to_last_packet_ms,
                         m_packetReadTimeoutMs, m_maxPacketsPerInterrupt, m_maxDrainDurationMs,
                         timeoutRetriesWhileAsserted);
  }
}

void Bno086ImuNode::MaybePublishOnLinearAcceleration(const SensorEvent& event)
{
  // LinearAcceleration is the cadence anchor for IMU publication once the
  // orientation + gyro + linear-accel trio is coherent
  if (event.report_id != ReportId::LinearAcceleration)
    return;

  if (!HasPublishableCoreFrame())
    return;

  const CoreFrameSignature signature = LatestCoreSignature();
  if (m_lastPublishedCoreSignature.has_value() &&
      signature.orientation_sequence == m_lastPublishedCoreSignature->orientation_sequence &&
      signature.gyro_sequence == m_lastPublishedCoreSignature->gyro_sequence &&
      signature.linear_accel_sequence == m_lastPublishedCoreSignature->linear_accel_sequence &&
      signature.orientation_stamp_ns == m_lastPublishedCoreSignature->orientation_stamp_ns &&
      signature.gyro_stamp_ns == m_lastPublishedCoreSignature->gyro_stamp_ns &&
      signature.linear_accel_stamp_ns == m_lastPublishedCoreSignature->linear_accel_stamp_ns)
  {
    return;
  }

  PublishLatestFrame(LatestCoreStamp());
  m_lastPublishedCoreSignature = signature;
}

void Bno086ImuNode::MaybePublishImuGravityOnAccelerometer(const SensorEvent& event)
{
  if (event.report_id != ReportId::Accelerometer)
    return;

  ++m_imuGravityDiagnostics.calibrated_accel_reports_received;

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

  const ImuGravityAccelSignature accelSignature{
      m_imuGravityState.sequence,
      accelStampNs,
  };

  if (m_lastPublishedImuGravityAccelSignature.has_value() &&
      accelSignature.sequence == m_lastPublishedImuGravityAccelSignature->sequence &&
      accelSignature.stamp_ns == m_lastPublishedImuGravityAccelSignature->stamp_ns)
  {
    ++m_imuGravityDiagnostics.imu_gravity_skipped_duplicate_sequence;
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
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), IMU_GRAVITY_DIAGNOSTICS_LOG_MS,
                         "Skipping BNO086 imu_gravity sample: %s", invalidReason.c_str());
    MaybeLogImuGravityDiagnostics();
    return;
  }

  m_imuGravityPublisher->publish(imuGravityMsg);
  m_lastPublishedImuGravityAccelSignature = accelSignature;
  ++m_imuGravityDiagnostics.imu_gravity_published;
  MaybeLogImuGravityDiagnostics();
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

  if (m_latestFrame.has_gravity)
  {
    std::optional<OASIS::IMU::Mat3> gravityCovariance;
    if (m_latestFrame.has_gravity_covariance)
      gravityCovariance = m_latestFrame.gravity_cov_mps2_2;

    const PublishedGravityMeasurement gravityMeasurement =
        MakePublishedGravityMeasurement(m_latestFrame.gravity_mps2, gravityCovariance);

    geometry_msgs::msg::AccelWithCovarianceStamped gravityMsg;
    gravityMsg.header = imuMsg.header;
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

    m_gravityPublisher->publish(gravityMsg);
  }
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
    if (featureResponse.report_interval_us > 0)
      m_actualFeatureIntervalUs[ReportIndex(featureResponse.report_id)] =
          featureResponse.report_interval_us;

    LogFeatureResponse(featureResponse);
  }
}

void Bno086ImuNode::LogFeatureResponse(const FeatureResponse& response) const
{
  const std::optional<std::uint32_t> requestedIntervalUs =
      RequestedFeatureIntervalUs(response.report_id);
  const std::uint32_t requestedUs = requestedIntervalUs.value_or(0U);
  const double actualRateHz = response.report_interval_us > 0
                                  ? 1'000'000.0 / static_cast<double>(response.report_interval_us)
                                  : 0.0;

  const bool calibratedAccel = response.report_id == ReportId::Accelerometer;
  const bool unexpectedCalibratedAccelRate =
      calibratedAccel && (actualRateHz < MIN_NEAR_TARGET_FEATURE_RATE_HZ ||
                          actualRateHz > MAX_NEAR_TARGET_FEATURE_RATE_HZ);

  if (unexpectedCalibratedAccelRate)
  {
    RCLCPP_WARN(get_logger(),
                "BNO086 Get Feature Response: report=%s id=0x%02X requested_interval_us=%u "
                "actual_interval_us=%u actual_rate_hz=%.2f batch_interval_us=%u "
                "feature_flags=0x%02X sensor_specific_config=0x%08X",
                ReportName(response.report_id), static_cast<unsigned>(response.report_id),
                requestedUs, response.report_interval_us, actualRateHz, response.batch_interval_us,
                static_cast<unsigned>(response.feature_flags), response.sensor_specific_config);
    return;
  }

  RCLCPP_INFO(get_logger(),
              "BNO086 Get Feature Response: report=%s id=0x%02X requested_interval_us=%u "
              "actual_interval_us=%u actual_rate_hz=%.2f batch_interval_us=%u "
              "feature_flags=0x%02X sensor_specific_config=0x%08X",
              ReportName(response.report_id), static_cast<unsigned>(response.report_id),
              requestedUs, response.report_interval_us, actualRateHz, response.batch_interval_us,
              static_cast<unsigned>(response.feature_flags), response.sensor_specific_config);
}

void Bno086ImuNode::MaybeLogImuGravityDiagnostics()
{
  const auto now = std::chrono::steady_clock::now();
  if (m_imuGravityDiagnostics.last_log_at.time_since_epoch().count() != 0)
  {
    const auto elapsedMs = std::chrono::duration_cast<std::chrono::milliseconds>(
                               now - m_imuGravityDiagnostics.last_log_at)
                               .count();
    if (elapsedMs < IMU_GRAVITY_DIAGNOSTICS_LOG_MS)
      return;

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
  }

  TimestampReconstructionDiagnostics reconstructionDiagnostics;
  ShtpDiagnostics shtpDiagnostics;
  const Bno086TransportDiagnostics& transportDiagnostics = m_transport.GetDiagnostics();
  if (m_shtp != nullptr)
  {
    reconstructionDiagnostics = m_shtp->GetTimestampReconstructionDiagnostics();
    shtpDiagnostics = m_shtp->GetDiagnostics();
  }

  const std::size_t accelIndex = ReportIndex(ReportId::Accelerometer);
  const std::size_t gyroIndex = ReportIndex(ReportId::GyroscopeCalibrated);
  const std::size_t rotationIndex = ReportIndex(ReportId::RotationVector);
  const std::size_t linearAccelIndex = ReportIndex(ReportId::LinearAcceleration);
  const std::size_t gravityIndex = ReportIndex(ReportId::Gravity);
  const ReportSequenceDiagnostics& accelSequenceDiagnostics =
      shtpDiagnostics.report_sequence[accelIndex];
  const double packetsPerDrainMean =
      m_interruptDrainDiagnostics.drain_cycles_completed > 0
          ? static_cast<double>(m_interruptDrainDiagnostics.packets_per_drain_total) /
                static_cast<double>(m_interruptDrainDiagnostics.drain_cycles_completed)
          : 0.0;

  if (shtpDiagnostics.continuation_packets_reset > m_lastLoggedShtpContinuationResetCount)
  {
    const std::string payloadPrefix = ContinuationPayloadPrefixHex(shtpDiagnostics);
    RCLCPP_WARN(get_logger(),
                "BNO086 SHTP continuation reset: count=%llu channel=%u accumulated_bytes=%u "
                "incoming_bytes=%u reason=%s raw_length=0x%04X parsed_length=%u sequence=%u "
                "active_buffer=%s payload_prefix=\"%s\"",
                static_cast<unsigned long long>(shtpDiagnostics.continuation_packets_reset),
                static_cast<unsigned>(shtpDiagnostics.latest_continuation_reset_channel),
                shtpDiagnostics.latest_continuation_reset_accumulated_bytes,
                shtpDiagnostics.latest_continuation_reset_incoming_bytes,
                ContinuationResetReasonName(shtpDiagnostics.latest_continuation_reset_reason),
                static_cast<unsigned>(shtpDiagnostics.latest_continuation_reset_raw_length),
                static_cast<unsigned>(shtpDiagnostics.latest_continuation_reset_packet_length),
                static_cast<unsigned>(shtpDiagnostics.latest_continuation_reset_sequence),
                shtpDiagnostics.latest_continuation_reset_had_active_buffer ? "true" : "false",
                payloadPrefix.c_str());
    m_lastLoggedShtpContinuationResetCount = shtpDiagnostics.continuation_packets_reset;
  }

  RCLCPP_INFO(
      get_logger(),
      "BNO086 imu_gravity diagnostics: calibrated_accel_reports_received=%llu "
      "imu_gravity_published=%llu skipped_missing_orientation=%llu "
      "skipped_missing_gyro=%llu skipped_stale_orientation=%llu "
      "skipped_stale_gyro=%llu skipped_duplicate_sequence=%llu skipped_nonfinite=%llu "
      "timestamp_reconstruction_base_resets=%llu "
      "timestamp_reconstruction_base_resets_negative_delta=%llu "
      "timestamp_reconstruction_base_resets_large_delta=%llu "
      "timestamp_reconstruction_base_resets_host_mismatch=%llu "
      "timestamp_reconstruction_base_wraps_accepted=%llu "
      "timestamp_reconstruction_missing_base=%llu "
      "timestamp_reconstruction_delay_applied=%llu "
      "latest_base_delta_us=%lld latest_host_delta_us=%lld "
      "latest_base_host_error_us=%lld "
      "shtp_packets_read=%llu shtp_sensor_packets_read=%llu "
      "shtp_sensor_events_decoded=%llu shtp_accel_decoded=%llu "
      "shtp_gyro_decoded=%llu shtp_rotation_decoded=%llu "
      "shtp_linear_accel_decoded=%llu shtp_gravity_decoded=%llu "
      "shtp_accel_sequence_gaps=%llu shtp_accel_sequence_gap_max=%u "
      "shtp_accel_duplicate_sequences=%llu shtp_decode_errors=%llu "
      "shtp_continuation_resets=%llu shtp_max_events_per_packet=%u "
      "shtp_max_packet_payload_bytes=%u shtp_continuation_flag_packets=%llu "
      "shtp_continuation_without_active_buffer=%llu "
      "shtp_continuation_flag_on_decodable_payload=%llu "
      "shtp_continuation_flag_on_control_payload=%llu "
      "shtp_continuation_flag_on_sensor_payload=%llu "
      "shtp_active_fragment_buffers_started=%llu "
      "shtp_active_fragment_buffers_completed=%llu "
      "shtp_active_fragment_buffers_reset=%llu "
      "shtp_orphan_continuation_decoded=%llu shtp_orphan_continuation_discarded=%llu "
      "shtp_embedded_shtp_header_payloads=%llu "
      "shtp_packet_sequence_gaps_channel3=%llu "
      "shtp_packet_sequence_gap_max_channel3=%u "
      "shtp_packet_duplicate_sequences_channel3=%llu "
      "interrupt_received_count=%llu int_to_first_poll_ms=%.3f "
      "max_int_to_first_poll_ms=%.3f int_to_last_packet_ms=%.3f "
      "max_int_to_last_packet_ms=%.3f interrupt_response_over_10ms=%llu "
      "drain_cycles_started=%llu drain_cycles_completed=%llu "
      "packets_per_drain_latest=%llu packets_per_drain_max=%llu "
      "packets_per_drain_mean=%.2f sensor_events_per_drain_latest=%llu "
      "sensor_events_per_drain_max=%llu drain_exit_timeout=%llu "
      "drain_exit_timeout_after_progress=%llu "
      "drain_exit_timeout_no_progress=%llu "
      "max_drain_duration_ms_config=%.1f latest_drain_duration_ms=%.3f "
      "max_drain_duration_ms_observed=%.3f packets_per_ms_latest=%.3f "
      "sensor_events_per_ms_latest=%.3f startup_backlog_drains=%llu "
      "drain_exit_timeout_hintn_deasserted=%llu "
      "drain_exit_timeout_hintn_asserted=%llu latest_timeout_hintn_asserted=%s "
      "timeout_retries_while_hintn_asserted=%llu "
      "latest_timeout_retries_while_hintn_asserted=%llu "
      "poll_calls=%llu latest_poll_duration_ms=%.3f max_poll_duration_ms=%.3f "
      "poll_duration_over_timeout=%llu poll_duration_over_10ms=%llu "
      "poll_duration_over_50ms=%llu "
      "latest_transport_read_duration_ms=%.3f max_transport_read_duration_ms=%.3f "
      "transport_read_over_timeout_count=%llu transport_read_calls=%llu "
      "transport_header_read_ms=%.3f transport_packet_read_ms=%.3f "
      "transport_transaction_ms=%.3f transport_transaction_bytes=%u "
      "transport_packet_read_over_timeout=%llu transport_invalid_headers=%llu "
      "transport_pseudo_payloads=%llu transport_zero_length_headers=%llu "
      "transport_invalid_full_packets=%llu transport_full_read_low_budget=%llu "
      "transport_transaction_over_timeout=%llu transport_transaction_failures=%llu "
      "drain_exit_int_deasserted=%llu drain_exit_max_packets=%llu "
      "drain_exit_duration_budget=%llu "
      "drain_exit_transport_error=%llu drain_exit_no_events=%llu "
      "hintn_asserted_at_exit=%s hintn_still_asserted_at_exit_count=%llu "
      "consecutive_hintn_asserted_drain_exits=%llu "
      "max_consecutive_hintn_asserted_drain_exits=%llu "
      "stuck_interrupt_recovery_candidate_count=%llu "
      "drain_cycles_over_1ms=%llu drain_cycles_over_5ms=%llu "
      "drain_cycles_over_10ms=%llu drain_cycles_over_20ms=%llu "
      "timestamp_repaired_nonmonotonic_accel=%llu true_duplicate_accel_stamp=%llu "
      "accel_sequence_gap_count=%llu accel_sequence_gap_max=%llu "
      "latest_accel_raw_delta_ms=%.3f latest_accel_normalized_delta_ms=%.3f "
      "latest_accel_repair_offset_ns=%lld latest_accel_sequence_delta=%u "
      "latest_orientation_age_ms=%.2f latest_gyro_age_ms=%.2f "
      "latest_calibrated_accel_rate_hz=%.2f latest_imu_gravity_rate_hz=%.2f "
      "target_rate_hz=%.1f",
      static_cast<unsigned long long>(m_imuGravityDiagnostics.calibrated_accel_reports_received),
      static_cast<unsigned long long>(m_imuGravityDiagnostics.imu_gravity_published),
      static_cast<unsigned long long>(
          m_imuGravityDiagnostics.imu_gravity_skipped_missing_orientation),
      static_cast<unsigned long long>(m_imuGravityDiagnostics.imu_gravity_skipped_missing_gyro),
      static_cast<unsigned long long>(
          m_imuGravityDiagnostics.imu_gravity_skipped_stale_orientation),
      static_cast<unsigned long long>(m_imuGravityDiagnostics.imu_gravity_skipped_stale_gyro),
      static_cast<unsigned long long>(
          m_imuGravityDiagnostics.imu_gravity_skipped_duplicate_sequence),
      static_cast<unsigned long long>(m_imuGravityDiagnostics.imu_gravity_skipped_nonfinite),
      static_cast<unsigned long long>(reconstructionDiagnostics.base_resets),
      static_cast<unsigned long long>(reconstructionDiagnostics.base_resets_negative_delta),
      static_cast<unsigned long long>(reconstructionDiagnostics.base_resets_large_delta),
      static_cast<unsigned long long>(reconstructionDiagnostics.base_resets_host_mismatch),
      static_cast<unsigned long long>(reconstructionDiagnostics.base_wraps_accepted),
      static_cast<unsigned long long>(reconstructionDiagnostics.missing_base),
      static_cast<unsigned long long>(reconstructionDiagnostics.delay_applied),
      static_cast<long long>(reconstructionDiagnostics.latest_base_delta_us),
      static_cast<long long>(reconstructionDiagnostics.latest_host_delta_us),
      static_cast<long long>(reconstructionDiagnostics.latest_base_host_error_us),
      static_cast<unsigned long long>(shtpDiagnostics.packets_read),
      static_cast<unsigned long long>(shtpDiagnostics.sensor_packets_read),
      static_cast<unsigned long long>(shtpDiagnostics.sensor_events_decoded),
      static_cast<unsigned long long>(shtpDiagnostics.decoded_report_counts[accelIndex]),
      static_cast<unsigned long long>(shtpDiagnostics.decoded_report_counts[gyroIndex]),
      static_cast<unsigned long long>(shtpDiagnostics.decoded_report_counts[rotationIndex]),
      static_cast<unsigned long long>(shtpDiagnostics.decoded_report_counts[linearAccelIndex]),
      static_cast<unsigned long long>(shtpDiagnostics.decoded_report_counts[gravityIndex]),
      static_cast<unsigned long long>(accelSequenceDiagnostics.gap_count),
      static_cast<unsigned>(accelSequenceDiagnostics.gap_max),
      static_cast<unsigned long long>(accelSequenceDiagnostics.duplicate_sequence_count),
      static_cast<unsigned long long>(shtpDiagnostics.decode_errors),
      static_cast<unsigned long long>(shtpDiagnostics.continuation_packets_reset),
      shtpDiagnostics.max_events_per_packet, shtpDiagnostics.max_packet_payload_bytes,
      static_cast<unsigned long long>(shtpDiagnostics.packets_with_continuation_flag),
      static_cast<unsigned long long>(shtpDiagnostics.continuation_without_active_buffer),
      static_cast<unsigned long long>(shtpDiagnostics.continuation_flag_on_decodable_payload),
      static_cast<unsigned long long>(shtpDiagnostics.continuation_flag_on_control_payload),
      static_cast<unsigned long long>(shtpDiagnostics.continuation_flag_on_sensor_payload),
      static_cast<unsigned long long>(shtpDiagnostics.active_fragment_buffers_started),
      static_cast<unsigned long long>(shtpDiagnostics.active_fragment_buffers_completed),
      static_cast<unsigned long long>(shtpDiagnostics.active_fragment_buffers_reset),
      static_cast<unsigned long long>(shtpDiagnostics.orphan_continuation_decoded),
      static_cast<unsigned long long>(shtpDiagnostics.orphan_continuation_discarded),
      static_cast<unsigned long long>(shtpDiagnostics.embedded_shtp_header_payloads),
      static_cast<unsigned long long>(shtpDiagnostics.packet_sequence_gaps_by_channel[3]),
      static_cast<unsigned>(shtpDiagnostics.packet_sequence_gap_max_by_channel[3]),
      static_cast<unsigned long long>(shtpDiagnostics.packet_duplicate_sequences_by_channel[3]),
      static_cast<unsigned long long>(m_interruptDrainDiagnostics.interrupt_received_count),
      m_interruptDrainDiagnostics.latest_interrupt_to_first_poll_ms,
      m_interruptDrainDiagnostics.max_interrupt_to_first_poll_ms,
      m_interruptDrainDiagnostics.latest_interrupt_to_last_packet_ms,
      m_interruptDrainDiagnostics.max_interrupt_to_last_packet_ms,
      static_cast<unsigned long long>(m_interruptDrainDiagnostics.interrupt_response_over_10ms),
      static_cast<unsigned long long>(m_interruptDrainDiagnostics.drain_cycles_started),
      static_cast<unsigned long long>(m_interruptDrainDiagnostics.drain_cycles_completed),
      static_cast<unsigned long long>(m_interruptDrainDiagnostics.packets_per_drain_latest),
      static_cast<unsigned long long>(m_interruptDrainDiagnostics.packets_per_drain_max),
      packetsPerDrainMean,
      static_cast<unsigned long long>(m_interruptDrainDiagnostics.sensor_events_per_drain_latest),
      static_cast<unsigned long long>(m_interruptDrainDiagnostics.sensor_events_per_drain_max),
      static_cast<unsigned long long>(m_interruptDrainDiagnostics.drain_exited_timeout),
      static_cast<unsigned long long>(
          m_interruptDrainDiagnostics.drain_exited_timeout_after_progress),
      static_cast<unsigned long long>(m_interruptDrainDiagnostics.drain_exited_timeout_no_progress),
      m_interruptDrainDiagnostics.max_drain_duration_ms_config,
      m_interruptDrainDiagnostics.latest_drain_duration_ms,
      m_interruptDrainDiagnostics.max_drain_duration_ms_observed,
      m_interruptDrainDiagnostics.packets_per_ms_latest,
      m_interruptDrainDiagnostics.sensor_events_per_ms_latest,
      static_cast<unsigned long long>(m_interruptDrainDiagnostics.startup_backlog_drains),
      static_cast<unsigned long long>(
          m_interruptDrainDiagnostics.drain_exited_timeout_hintn_deasserted),
      static_cast<unsigned long long>(
          m_interruptDrainDiagnostics.drain_exited_timeout_hintn_asserted),
      m_interruptDrainDiagnostics.latest_timeout_hintn_asserted ? "true" : "false",
      static_cast<unsigned long long>(
          m_interruptDrainDiagnostics.timeout_retries_while_hintn_asserted),
      static_cast<unsigned long long>(
          m_interruptDrainDiagnostics.latest_timeout_retries_while_hintn_asserted),
      static_cast<unsigned long long>(m_interruptDrainDiagnostics.poll_calls),
      m_interruptDrainDiagnostics.latest_poll_duration_ms,
      m_interruptDrainDiagnostics.max_poll_duration_ms,
      static_cast<unsigned long long>(m_interruptDrainDiagnostics.poll_duration_over_timeout_count),
      static_cast<unsigned long long>(m_interruptDrainDiagnostics.poll_duration_over_10ms_count),
      static_cast<unsigned long long>(m_interruptDrainDiagnostics.poll_duration_over_50ms_count),
      transportDiagnostics.latest_transport_read_duration_ms,
      transportDiagnostics.max_transport_read_duration_ms,
      static_cast<unsigned long long>(transportDiagnostics.transport_read_over_timeout_count),
      static_cast<unsigned long long>(transportDiagnostics.transport_read_calls),
      transportDiagnostics.latest_header_read_duration_ms,
      transportDiagnostics.latest_packet_read_duration_ms,
      transportDiagnostics.latest_transaction_duration_ms,
      static_cast<unsigned>(transportDiagnostics.latest_transaction_bytes),
      static_cast<unsigned long long>(transportDiagnostics.packet_read_over_timeout_count),
      static_cast<unsigned long long>(transportDiagnostics.read_packet_invalid_headers),
      static_cast<unsigned long long>(transportDiagnostics.read_packet_pseudo_payloads),
      static_cast<unsigned long long>(transportDiagnostics.read_packet_zero_length_headers),
      static_cast<unsigned long long>(transportDiagnostics.read_packet_invalid_full_packets),
      static_cast<unsigned long long>(
          transportDiagnostics.full_packet_read_started_with_low_budget),
      static_cast<unsigned long long>(transportDiagnostics.transaction_over_timeout_count),
      static_cast<unsigned long long>(transportDiagnostics.transaction_failures),
      static_cast<unsigned long long>(m_interruptDrainDiagnostics.drain_exited_int_deasserted),
      static_cast<unsigned long long>(m_interruptDrainDiagnostics.drain_exited_max_packets),
      static_cast<unsigned long long>(m_interruptDrainDiagnostics.drain_exited_duration_budget),
      static_cast<unsigned long long>(m_interruptDrainDiagnostics.drain_exited_transport_error),
      static_cast<unsigned long long>(m_interruptDrainDiagnostics.drain_exited_no_events),
      m_interruptDrainDiagnostics.latest_hintn_asserted_at_exit ? "true" : "false",
      static_cast<unsigned long long>(
          m_interruptDrainDiagnostics.count_hintn_still_asserted_at_exit),
      static_cast<unsigned long long>(
          m_interruptDrainDiagnostics.consecutive_hintn_asserted_drain_exits),
      static_cast<unsigned long long>(
          m_interruptDrainDiagnostics.max_consecutive_hintn_asserted_drain_exits),
      static_cast<unsigned long long>(
          m_interruptDrainDiagnostics.stuck_interrupt_recovery_candidate_count),
      static_cast<unsigned long long>(m_interruptDrainDiagnostics.drain_cycles_over_1ms),
      static_cast<unsigned long long>(m_interruptDrainDiagnostics.drain_cycles_over_5ms),
      static_cast<unsigned long long>(m_interruptDrainDiagnostics.drain_cycles_over_10ms),
      static_cast<unsigned long long>(m_interruptDrainDiagnostics.drain_cycles_over_20ms),
      static_cast<unsigned long long>(
          m_imuGravityDiagnostics.timestamp_repaired_nonmonotonic_accel),
      static_cast<unsigned long long>(m_imuGravityDiagnostics.true_duplicate_accel_stamp),
      static_cast<unsigned long long>(m_imuGravityDiagnostics.accel_sequence_gap_count),
      static_cast<unsigned long long>(m_imuGravityDiagnostics.accel_sequence_gap_max),
      m_imuGravityDiagnostics.latest_accel_raw_delta_ms,
      m_imuGravityDiagnostics.latest_accel_normalized_delta_ms,
      static_cast<long long>(m_imuGravityDiagnostics.latest_accel_repair_offset_ns),
      static_cast<unsigned>(m_imuGravityDiagnostics.latest_accel_sequence_delta),
      m_imuGravityDiagnostics.latest_orientation_age_ms, m_imuGravityDiagnostics.latest_gyro_age_ms,
      m_imuGravityDiagnostics.latest_calibrated_accel_rate_hz,
      m_imuGravityDiagnostics.latest_imu_gravity_rate_hz, IMU_GRAVITY_TARGET_RATE_HZ);

  m_imuGravityDiagnostics.last_log_at = now;
  m_imuGravityDiagnostics.last_rate_accel_reports =
      m_imuGravityDiagnostics.calibrated_accel_reports_received;
  m_imuGravityDiagnostics.last_rate_imu_gravity_published =
      m_imuGravityDiagnostics.imu_gravity_published;
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

rclcpp::Time Bno086ImuNode::NormalizeEventStamp(const SensorEvent& event,
                                                const rclcpp::Time& raw_stamp)
{
  ReportTimestampState& state = m_reportTimestampStates[ReportIndex(event.report_id)];
  const TimestampNormalizationConfig config = TimestampConfigForReport(event.report_id);
  const TimestampNormalizationResult result =
      NormalizeReportTimestamp(state, event.sequence, raw_stamp.nanoseconds(), config);
  const rclcpp::Time normalizedStamp(result.stamp_ns, raw_stamp.get_clock_type());

  UpdateTimestampDiagnostics(event, result, raw_stamp, normalizedStamp);
  return normalizedStamp;
}

TimestampNormalizationConfig Bno086ImuNode::TimestampConfigForReport(ReportId report_id) const
{
  TimestampNormalizationConfig config;
  config.expected_interval_us = ExpectedFeatureIntervalUs(report_id);
  return config;
}

void Bno086ImuNode::UpdateTimestampDiagnostics(const SensorEvent& event,
                                               const TimestampNormalizationResult& result,
                                               const rclcpp::Time& raw_stamp,
                                               const rclcpp::Time& normalized_stamp)
{
  if (event.report_id != ReportId::Accelerometer)
    return;

  const ReportTimestampState& state = m_reportTimestampStates[ReportIndex(event.report_id)];
  m_imuGravityDiagnostics.timestamp_repaired_nonmonotonic_accel = state.repaired_nonmonotonic;
  m_imuGravityDiagnostics.true_duplicate_accel_stamp = state.true_duplicate;
  m_imuGravityDiagnostics.accel_sequence_gap_count = state.sequence_gap_count;
  m_imuGravityDiagnostics.accel_sequence_gap_max = state.sequence_gap_max;
  m_imuGravityDiagnostics.latest_accel_sequence_delta = result.sequence_delta;
  m_imuGravityDiagnostics.latest_accel_raw_delta_ms =
      static_cast<double>(result.raw_stamp_delta_ns) / 1.0e6;
  m_imuGravityDiagnostics.latest_accel_normalized_delta_ms =
      static_cast<double>(result.normalized_stamp_delta_ns) / 1.0e6;
  m_imuGravityDiagnostics.latest_accel_repair_offset_ns =
      normalized_stamp.nanoseconds() - raw_stamp.nanoseconds();

  if (result.status != TimestampNormalizationStatus::RepairedNonmonotonic)
    return;

  RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), ACCEL_TIMESTAMP_REPAIR_LOG_MS,
                       "Repaired BNO086 calibrated accel timestamp: seq_delta=%u raw_delta_ms=%.3f "
                       "repaired_delta_ms=%.6f raw_stamp_ns=%lld repaired_stamp_ns=%lld",
                       static_cast<unsigned>(result.sequence_delta),
                       static_cast<double>(result.raw_stamp_delta_ns) / 1.0e6,
                       static_cast<double>(result.normalized_stamp_delta_ns) / 1.0e6,
                       static_cast<long long>(raw_stamp.nanoseconds()),
                       static_cast<long long>(normalized_stamp.nanoseconds()));
}

std::optional<std::uint32_t> Bno086ImuNode::ActualFeatureIntervalUs(ReportId report_id) const
{
  return m_actualFeatureIntervalUs[ReportIndex(report_id)];
}

std::uint32_t Bno086ImuNode::ExpectedFeatureIntervalUs(ReportId report_id) const
{
  const std::optional<std::uint32_t> actualIntervalUs = ActualFeatureIntervalUs(report_id);
  if (actualIntervalUs.has_value())
    return *actualIntervalUs;

  const std::optional<std::uint32_t> requestedIntervalUs = RequestedFeatureIntervalUs(report_id);
  if (requestedIntervalUs.has_value())
    return *requestedIntervalUs;

  return m_reportIntervalUs;
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
      break;
    }

    default:
      break;
  }
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
      return "calibrated_acceleration";
    case ReportId::GyroscopeCalibrated:
      return "calibrated_gyro";
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
