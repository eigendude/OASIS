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

// Units: ns. SH-2 delay values above this are not plausible sample latency
constexpr int64_t MAX_PLAUSIBLE_SAMPLE_DELAY_NS = 50'000'000;

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

struct Bno086ImuNode::ImuGravityAccelSample
{
  bool has_sample{false};
  rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
  OASIS::IMU::Vec3 accel_mps2{0.0, 0.0, 0.0};
  OASIS::IMU::Mat3 covariance_mps2_2{};
  bool has_covariance{false};
  std::uint8_t sequence{0};
  std::uint8_t accuracy{0};
};

class Bno086ImuNode::ImuGravityAccelHistory
{
public:
  void Push(const ImuGravityAccelSample& sample)
  {
    m_samples[m_next] = sample;
    m_next = (m_next + 1) % m_samples.size();
    m_count = std::min(m_count + 1, m_samples.size());
  }

  std::optional<ImuGravityAccelSample> SelectAtOrBefore(int64_t anchor_stamp_ns,
                                                        int64_t future_tolerance_ns) const
  {
    std::optional<ImuGravityAccelSample> nearestPast;
    std::optional<ImuGravityAccelSample> nearestFuture;

    for (std::size_t i = 0; i < m_count; ++i)
    {
      const ImuGravityAccelSample& sample = m_samples[i];
      if (!sample.has_sample)
        continue;

      const int64_t sampleStampNs = sample.stamp.nanoseconds();
      if (sampleStampNs <= anchor_stamp_ns)
      {
        if (!nearestPast.has_value() || sampleStampNs > nearestPast->stamp.nanoseconds())
          nearestPast = sample;
      }
      else if (!nearestFuture.has_value() || sampleStampNs < nearestFuture->stamp.nanoseconds())
      {
        nearestFuture = sample;
      }
    }

    if (nearestPast.has_value())
      return nearestPast;

    if (nearestFuture.has_value() &&
        nearestFuture->stamp.nanoseconds() - anchor_stamp_ns <= future_tolerance_ns)
    {
      return nearestFuture;
    }

    return std::nullopt;
  }

  void Reset()
  {
    m_samples = {};
    m_next = 0;
    m_count = 0;
  }

private:
  std::array<ImuGravityAccelSample, 1024> m_samples{};
  std::size_t m_next{0};
  std::size_t m_count{0};
};

struct Bno086ImuNode::RateSnapshot
{
  std::array<double, 5> decoded_hz{};
  double imu_gravity_hz{0.0};
  double imu_hz{0.0};
};

struct Bno086ImuNode::RateDiagnostics
{
  std::array<std::uint64_t, 5> decoded_reports_received{};
  std::array<std::uint64_t, 5> last_rate_decoded_reports{};
  std::uint64_t imu_gravity_published{0};
  std::uint64_t imu_published{0};
  std::uint64_t last_rate_imu_gravity_published{0};
  std::uint64_t last_rate_imu_published{0};
  std::chrono::steady_clock::time_point last_log_at{};
};

class Bno086ImuNode::Bno086DrainHealth
{
public:
  void Record(const Bno086DrainCounters& counters,
              Bno086DrainAction exit_action,
              std::uint32_t drain_duration_us)
  {
    ++m_drains;
    m_physicalPacketsSum += counters.physical_packets_this_drain;
    m_physicalPacketsMax = std::max(m_physicalPacketsMax, counters.physical_packets_this_drain);
    m_sensorEventsSum += counters.sensor_events_this_drain;
    m_sensorEventsMax = std::max(m_sensorEventsMax, counters.sensor_events_this_drain);
    m_drainDurationSumUs += drain_duration_us;
    m_drainDurationMaxUs = std::max(m_drainDurationMaxUs, drain_duration_us);

    if (exit_action == Bno086DrainAction::PhysicalPacketCap)
      ++m_physicalPacketCapHitCount;
    if (exit_action == Bno086DrainAction::PollIterationCap)
      ++m_pollIterationCapHitCount;
    if (exit_action == Bno086DrainAction::AllZeroBudget)
      ++m_allZeroBudgetHitCount;
    if (exit_action == Bno086DrainAction::NoProgressBudget)
      ++m_noProgressDrainCount;
    if (exit_action == Bno086DrainAction::TransportError)
      ++m_transportErrorCount;
  }

  void CountAllZeroBackoff() { ++m_allZeroBackoffCount; }

  bool HasSafetyFailure() const
  {
    return m_physicalPacketCapHitCount > 0 || m_pollIterationCapHitCount > 0 ||
           m_noProgressDrainCount > 0 || m_transportErrorCount > 0 || m_allZeroBudgetHitCount > 0;
  }

  std::uint64_t Drains() const { return m_drains; }
  std::uint32_t PhysicalPacketsMax() const { return m_physicalPacketsMax; }
  std::uint32_t SensorEventsMax() const { return m_sensorEventsMax; }
  std::uint64_t AllZeroBackoffCount() const { return m_allZeroBackoffCount; }
  std::uint32_t DrainDurationMaxUs() const { return m_drainDurationMaxUs; }
  std::uint64_t PhysicalPacketCapHitCount() const { return m_physicalPacketCapHitCount; }
  std::uint64_t PollIterationCapHitCount() const { return m_pollIterationCapHitCount; }
  std::uint64_t NoProgressDrainCount() const { return m_noProgressDrainCount; }
  std::uint64_t TransportErrorCount() const { return m_transportErrorCount; }
  std::uint64_t AllZeroBudgetHitCount() const { return m_allZeroBudgetHitCount; }

  double PhysicalPacketsMean() const
  {
    return m_drains > 0 ? static_cast<double>(m_physicalPacketsSum) / static_cast<double>(m_drains)
                        : 0.0;
  }

  double SensorEventsMean() const
  {
    return m_drains > 0 ? static_cast<double>(m_sensorEventsSum) / static_cast<double>(m_drains)
                        : 0.0;
  }

  double DrainDurationMeanMs() const
  {
    return m_drains > 0
               ? static_cast<double>(m_drainDurationSumUs) / static_cast<double>(m_drains) / 1.0e3
               : 0.0;
  }

private:
  std::uint64_t m_drains{0};
  std::uint64_t m_physicalPacketsSum{0};
  std::uint32_t m_physicalPacketsMax{0};
  std::uint64_t m_sensorEventsSum{0};
  std::uint32_t m_sensorEventsMax{0};
  std::uint64_t m_allZeroBackoffCount{0};
  std::uint64_t m_drainDurationSumUs{0};
  std::uint32_t m_drainDurationMaxUs{0};
  std::uint64_t m_physicalPacketCapHitCount{0};
  std::uint64_t m_pollIterationCapHitCount{0};
  std::uint64_t m_noProgressDrainCount{0};
  std::uint64_t m_transportErrorCount{0};
  std::uint64_t m_allZeroBudgetHitCount{0};
};

Bno086ImuNode::Bno086ImuNode() : rclcpp::Node(NODE_NAME)
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
  m_allZeroBackoffUs =
      std::clamp(static_cast<int>(get_parameter("bno086_all_zero_backoff_us").as_int()),
                 MIN_ALL_ZERO_BACKOFF_US, MAX_ALL_ZERO_BACKOFF_US);
  m_diagnosticsLogPeriodMs =
      std::max(static_cast<int>(get_parameter("bno086_diagnostics_log_period_ms").as_int()),
               MIN_BNO086_DIAGNOSTICS_LOG_PERIOD_MS);
  m_predictionHorizonSec = std::max(get_parameter("prediction_horizon_sec").as_double(), 0.0);
  m_imuGravityMaxOrientationAgeMs =
      std::max(get_parameter("imu_gravity_max_orientation_age_ms").as_double(), 0.0);
  m_imuGravityMaxGyroAgeMs =
      std::max(get_parameter("imu_gravity_max_gyro_age_ms").as_double(), 0.0);

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
  (void)m_shtp->DrainFeatureResponses(m_featureResponseStartupDrainMs,
                                      m_featureResponseStartupMaxPackets, m_packetReadTimeoutMs);
  MaybeLogFeatureResponses();

  if (intGpio == DEFAULT_INT_GPIO)
  {
    RCLCPP_INFO(get_logger(), "BNO086 INT uses GPIO%d (Raspberry Pi header pin 16), active low",
                intGpio);
  }

  RCLCPP_INFO(get_logger(), "BNO086 opened on %s (0x%02X), int_gpio=%d active_low",
              i2cDevice.c_str(), static_cast<unsigned>(i2cAddress), intGpio);
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
  RCLCPP_INFO(get_logger(), "BNO086 publication cadence: imu=linear_acceleration "
                            "imu_gravity=rotation_vector gravity=gravity_report");
  if (m_predictionHorizonSec > 0.0)
  {
    RCLCPP_INFO(get_logger(),
                "Predicted orientation output uses %s with prediction_horizon_sec=%.4f",
                m_predictionSource.c_str(), m_predictionHorizonSec);
  }
  RCLCPP_INFO(get_logger(),
              "BNO086 imu_gravity uses rotation vector cadence with "
              "orientation_max_age_ms=%.1f gyro_max_age_ms=%.1f",
              m_imuGravityMaxOrientationAgeMs, m_imuGravityMaxGyroAgeMs);
}

Bno086ImuNode::~Bno086ImuNode() = default;

bool Bno086ImuNode::Initialize()
{
  for (Bno086ReportTimestampTracker& tracker : m_timestampTrackers)
    tracker.Reset();
  m_bnoCadenceEpochNs.reset();
  m_lastEmittedTimestampNs.fill(std::nullopt);
  m_lastPublishedCoreSignature.reset();
  m_lastPublishedImuGravityAnchorStampNs.reset();
  if (!m_imuGravityAccelHistory)
    m_imuGravityAccelHistory = std::make_unique<ImuGravityAccelHistory>();
  m_imuGravityAccelHistory->Reset();
  m_drainHealth = std::make_unique<Bno086DrainHealth>();
  m_rateDiagnostics = std::make_unique<RateDiagnostics>();

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
  drainLimits.max_all_zero_polls_per_interrupt = m_maxAllZeroPollsPerInterrupt;
  drainLimits.max_sensor_events_per_drain = m_maxSensorEventsPerDrain;
  drainLimits.max_pending_events_flush_per_drain = m_maxPendingEventsFlushPerDrain;
  const rclcpp::Time drainReceiveAnchor = interrupt_ros_at;
  const auto drainStartedAt = std::chrono::steady_clock::now();
  Bno086DrainAction drainExitAction = Bno086DrainAction::Complete;

  while (m_running.load())
  {
    const bool hintnAssertedBeforePoll = m_interruptGpio.IsAssertedLow();
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
        elapsedBeforePollMs >= m_maxDrainDurationMs)
    {
      drainExitAction = Bno086DrainAction::DrainDurationBudget;
      break;
    }

    const Bno086Shtp::PollResult pollResult = m_shtp->Poll(m_packetReadTimeoutMs);
    const bool hintnAssertedAfterPoll = m_interruptGpio.IsAssertedLow();
    const Bno086DrainDecision afterPollDecision =
        Bno086DrainAfterPoll(pollResult, drainLimits, drainCounters, hintnAssertedAfterPoll);
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
          m_allZeroBackoffUs, static_cast<double>(drainDurationUs) / 1e3);
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

    if (pollResult.status == Bno086Shtp::PollStatus::AllZeroHeader &&
        afterPollDecision.action == Bno086DrainAction::Continue &&
        afterPollDecision.hintn_asserted && m_allZeroBackoffUs > 0)
    {
      m_drainHealth->CountAllZeroBackoff();
      std::this_thread::sleep_for(std::chrono::microseconds(m_allZeroBackoffUs));
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
        CountDecodedReport(*pollResult.event);
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
    if (m_shtp->PendingEventCount() == 0 && elapsedAfterPollMs >= m_maxDrainDurationMs)
    {
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

  if (!(m_latestFrame.has_orientation && m_latestFrame.has_gyro &&
        m_latestFrame.has_linear_accel) ||
      !(m_orientationState.has_sample && m_gyroState.has_sample && m_linearAccelState.has_sample))
  {
    MaybeLogImuGravityDiagnostics();
    return;
  }

  const int64_t orientationNs = m_orientationState.stamp.nanoseconds();
  const int64_t gyroNs = m_gyroState.stamp.nanoseconds();
  const int64_t linearAccelNs = m_linearAccelState.stamp.nanoseconds();
  const int64_t oldestNs = std::min({orientationNs, gyroNs, linearAccelNs});
  const int64_t newestNs = std::max({orientationNs, gyroNs, linearAccelNs});

  const int64_t coreThresholdNs = DurationFromUs(CoreCoherenceToleranceUs()).nanoseconds();
  if (!IsTimestampSpanCoherent(oldestNs, newestNs, coreThresholdNs))
  {
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
    MaybeLogImuGravityDiagnostics();
    return;
  }

  PublishLatestFrame(LatestCoreStamp());
  m_lastPublishedCoreSignature = signature;
  ++m_rateDiagnostics->imu_published;
}

void Bno086ImuNode::MaybePublishImuGravityOnRotationVector(const SensorEvent& event)
{
  if (event.report_id != ReportId::RotationVector)
    return;

  if (!m_orientationState.has_sample || !m_latestFrame.has_orientation)
  {
    MaybeLogImuGravityDiagnostics();
    return;
  }

  if (!m_gyroState.has_sample || !m_latestFrame.has_gyro)
  {
    MaybeLogImuGravityDiagnostics();
    return;
  }

  if (!m_imuGravityState.has_sample || !m_latestFrame.has_accel)
  {
    MaybeLogImuGravityDiagnostics();
    return;
  }

  const int64_t orientationStampNs = m_orientationState.stamp.nanoseconds();
  const int64_t maxAccelAgeNs = ImuGravityMaxOrientationAgeNs();
  const int64_t accelFutureToleranceNs = ReportFutureToleranceNs(ReportId::Accelerometer);
  const std::optional<ImuGravityAccelSample> accelSelection =
      SelectImuGravityAccelSample(orientationStampNs, accelFutureToleranceNs);

  if (!accelSelection.has_value())
  {
    MaybeLogImuGravityDiagnostics();
    return;
  }

  const ImuGravityAccelSample& accelSample = *accelSelection;
  const int64_t accelStampNs = accelSample.stamp.nanoseconds();
  const int64_t maxGyroAgeNs = ImuGravityMaxGyroAgeNs();
  const SampleFreshnessResult accelFreshness = EvaluateSampleFreshness(
      orientationStampNs, accelStampNs, maxAccelAgeNs, accelFutureToleranceNs);
  const SampleFreshnessResult gyroFreshness =
      EvaluateSampleFreshness(orientationStampNs, m_gyroState.stamp.nanoseconds(), maxGyroAgeNs,
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

  if (m_lastPublishedImuGravityAnchorStampNs.has_value() &&
      orientationStampNs <= *m_lastPublishedImuGravityAnchorStampNs)
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
      BuildImuGravityMessage(m_orientationState.stamp, accelSample);
  std::string invalidReason;
  if (!IsImuGravitySampleValid(imuGravityMsg, invalidReason))
  {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), IMU_GRAVITY_SAMPLE_WARN_THROTTLE_MS,
                         "Skipping BNO086 imu_gravity sample: %s", invalidReason.c_str());
    MaybeLogImuGravityDiagnostics();
    return;
  }

  m_imuGravityPublisher->publish(imuGravityMsg);
  m_lastPublishedImuGravityAnchorStampNs = orientationStampNs;
  ++m_rateDiagnostics->imu_gravity_published;
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

sensor_msgs::msg::Imu Bno086ImuNode::BuildImuGravityMessage(
    const rclcpp::Time& stamp, const ImuGravityAccelSample& accel_sample) const
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

  imuGravityMsg.linear_acceleration.x = accel_sample.accel_mps2[0];
  imuGravityMsg.linear_acceleration.y = accel_sample.accel_mps2[1];
  imuGravityMsg.linear_acceleration.z = accel_sample.accel_mps2[2];

  imuGravityMsg.orientation_covariance.fill(0.0);
  imuGravityMsg.angular_velocity_covariance.fill(0.0);
  imuGravityMsg.linear_acceleration_covariance.fill(0.0);

  if (m_latestFrame.has_orientation_covariance)
    SetCovariance(imuGravityMsg.orientation_covariance, m_latestFrame.orientation_cov_rad2);

  if (m_latestFrame.has_gyro_covariance)
    SetCovariance(imuGravityMsg.angular_velocity_covariance, m_latestFrame.gyro_cov_rads2_2);

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
  }

  MaybeLogFeatureSummary();
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
  if (m_rateDiagnostics->last_log_at.time_since_epoch().count() != 0)
  {
    const auto elapsedMs =
        std::chrono::duration_cast<std::chrono::milliseconds>(now - m_rateDiagnostics->last_log_at)
            .count();
    if (elapsedMs < m_diagnosticsLogPeriodMs)
      return;
  }

  const RateSnapshot rates = UpdateImuGravityDiagnosticsRates(now);
  MaybeEmitImuGravityDiagnosticsLog(rates);
  m_rateDiagnostics->last_log_at = now;
}

void Bno086ImuNode::RecordDrainThroughputDiagnostics(const Bno086DrainCounters& counters,
                                                     Bno086DrainAction exit_action,
                                                     bool /*hintn_asserted_after_exit*/,
                                                     std::uint32_t drain_duration_us,
                                                     std::uint32_t /*pending_queue_depth_at_exit*/)
{
  if (m_drainHealth)
    m_drainHealth->Record(counters, exit_action, drain_duration_us);
}

auto Bno086ImuNode::UpdateImuGravityDiagnosticsRates(
    const std::chrono::steady_clock::time_point& now) -> RateSnapshot
{
  RateSnapshot snapshot;
  if (m_rateDiagnostics->last_log_at.time_since_epoch().count() != 0)
  {
    const auto elapsedMs =
        std::chrono::duration_cast<std::chrono::milliseconds>(now - m_rateDiagnostics->last_log_at)
            .count();
    if (elapsedMs > 0)
    {
      const std::uint64_t imuGravityPublishedDelta =
          m_rateDiagnostics->imu_gravity_published -
          m_rateDiagnostics->last_rate_imu_gravity_published;
      const std::uint64_t imuPublishedDelta =
          m_rateDiagnostics->imu_published - m_rateDiagnostics->last_rate_imu_published;
      snapshot.imu_gravity_hz =
          static_cast<double>(imuGravityPublishedDelta) * 1000.0 / static_cast<double>(elapsedMs);
      snapshot.imu_hz =
          static_cast<double>(imuPublishedDelta) * 1000.0 / static_cast<double>(elapsedMs);

      for (std::size_t i = 0; i < snapshot.decoded_hz.size(); ++i)
      {
        const std::uint64_t decodedDelta = m_rateDiagnostics->decoded_reports_received[i] -
                                           m_rateDiagnostics->last_rate_decoded_reports[i];
        snapshot.decoded_hz[i] =
            static_cast<double>(decodedDelta) * 1000.0 / static_cast<double>(elapsedMs);
      }
    }
  }

  m_rateDiagnostics->last_rate_imu_gravity_published = m_rateDiagnostics->imu_gravity_published;
  m_rateDiagnostics->last_rate_imu_published = m_rateDiagnostics->imu_published;
  m_rateDiagnostics->last_rate_decoded_reports = m_rateDiagnostics->decoded_reports_received;
  return snapshot;
}

void Bno086ImuNode::MaybeEmitImuGravityDiagnosticsLog(const RateSnapshot& rates)
{
  const bool unhealthy = IsBno086DiagnosticsUnhealthy(rates);
  const Bno086TransportStats transportStats = m_transport.GetStats();

  RCLCPP_DEBUG(get_logger(),
               "BNO086 rates: accel=%.0f gyro=%.0f rot=%.0f lin=%.0f grav=%.0f "
               "imu_g=%.0f imu=%.0f",
               rates.decoded_hz[0], rates.decoded_hz[1], rates.decoded_hz[2], rates.decoded_hz[3],
               rates.decoded_hz[4], rates.imu_gravity_hz, rates.imu_hz);
  RCLCPP_DEBUG(get_logger(),
               "BNO086 drain: n=%llu pkt=%.1f/%u evt=%.1f/%u zero=%llu "
               "dur=%.0f/%.0f cap=%llu/%llu err=%llu/%llu az=%llu bad=%llu",
               static_cast<unsigned long long>(m_drainHealth->Drains()),
               m_drainHealth->PhysicalPacketsMean(), m_drainHealth->PhysicalPacketsMax(),
               m_drainHealth->SensorEventsMean(), m_drainHealth->SensorEventsMax(),
               static_cast<unsigned long long>(m_drainHealth->AllZeroBackoffCount()),
               m_drainHealth->DrainDurationMeanMs(),
               static_cast<double>(m_drainHealth->DrainDurationMaxUs()) / 1.0e3,
               static_cast<unsigned long long>(m_drainHealth->PhysicalPacketCapHitCount()),
               static_cast<unsigned long long>(m_drainHealth->PollIterationCapHitCount()),
               static_cast<unsigned long long>(m_drainHealth->NoProgressDrainCount()),
               static_cast<unsigned long long>(m_drainHealth->TransportErrorCount()),
               static_cast<unsigned long long>(m_drainHealth->AllZeroBudgetHitCount()),
               static_cast<unsigned long long>(transportStats.invalid_full_packet_count));

  if (m_diagnosticsWasUnhealthy && !unhealthy)
  {
    RCLCPP_INFO(get_logger(), "BNO086 imu_gravity recovered: imu_gravity_rate_hz=%.2f",
                rates.imu_gravity_hz);
  }

  m_diagnosticsWasUnhealthy = unhealthy;
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

bool Bno086ImuNode::IsBno086DiagnosticsUnhealthy(const RateSnapshot& rates) const
{
  if (rates.decoded_hz[0] > 0.0 &&
      rates.decoded_hz[0] < m_accelerometerRateHz * MIN_HEALTHY_RATE_FRACTION)
    return true;

  if (rates.decoded_hz[1] > 0.0 && rates.decoded_hz[1] < m_gyroRateHz * MIN_HEALTHY_RATE_FRACTION)
    return true;

  if (rates.decoded_hz[2] > 0.0 &&
      rates.decoded_hz[2] < m_rotationVectorRateHz * MIN_HEALTHY_RATE_FRACTION)
    return true;

  if (rates.decoded_hz[3] > 0.0 &&
      rates.decoded_hz[3] < m_linearAccelerationRateHz * MIN_HEALTHY_RATE_FRACTION)
    return true;

  if (rates.decoded_hz[4] > 0.0 &&
      rates.decoded_hz[4] < m_gravityRateHz * MIN_HEALTHY_RATE_FRACTION)
    return true;

  if (rates.imu_gravity_hz > 0.0 &&
      rates.imu_gravity_hz < m_rotationVectorRateHz * MIN_HEALTHY_RATE_FRACTION)
    return true;

  return m_drainHealth != nullptr && m_drainHealth->HasSafetyFailure();
}

void Bno086ImuNode::CountDecodedReport(const SensorEvent& event)
{
  const std::optional<std::size_t> reportIndex = DiagnosticReportIndex(event.report_id);
  if (reportIndex.has_value())
    ++m_rateDiagnostics->decoded_reports_received[*reportIndex];
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
      static_cast<int64_t>(m_imuGravityMaxOrientationAgeMs * 1.0e6),
      static_cast<int64_t>(intervalUs.value_or(DEFAULT_REPORT_INTERVAL_US)) * 1'000, 80'000'000);
}

int64_t Bno086ImuNode::ImuGravityMaxGyroAgeNs() const
{
  const std::optional<std::uint32_t> intervalUs =
      EffectiveReportIntervalUs(ReportId::GyroscopeCalibrated);
  return EffectiveMaxPastAgeNs(
      static_cast<int64_t>(m_imuGravityMaxGyroAgeMs * 1.0e6),
      static_cast<int64_t>(intervalUs.value_or(DEFAULT_REPORT_INTERVAL_US)) * 1'000, 80'000'000);
}

int64_t Bno086ImuNode::ReportFutureToleranceNs(ReportId report_id) const
{
  const std::optional<std::uint32_t> intervalUs = EffectiveReportIntervalUs(report_id);
  if (intervalUs.has_value() && *intervalUs > 0)
    return static_cast<int64_t>(*intervalUs) * 1'000;

  return DurationFromUs(CoreCoherenceToleranceUs()).nanoseconds();
}

void Bno086ImuNode::RecordImuGravityAccelSample(const rclcpp::Time& sample_stamp)
{
  ImuGravityAccelSample sample;
  sample.has_sample = true;
  sample.stamp = sample_stamp;
  sample.accel_mps2 = m_latestFrame.accel_mps2;
  sample.covariance_mps2_2 = m_latestFrame.accel_cov_mps2_2;
  sample.has_covariance = m_latestFrame.has_accel_covariance;
  sample.sequence = m_imuGravityState.sequence;
  sample.accuracy = m_imuGravityState.accuracy;

  m_imuGravityAccelHistory->Push(sample);
}

std::optional<Bno086ImuNode::ImuGravityAccelSample> Bno086ImuNode::SelectImuGravityAccelSample(
    int64_t anchor_stamp_ns, int64_t future_tolerance_ns) const
{
  return m_imuGravityAccelHistory->SelectAtOrBefore(anchor_stamp_ns, future_tolerance_ns);
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
      MaybeLogOrientationCovariancePolicy(covariancePolicy);
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

std::optional<int64_t> Bno086ImuNode::FinalizeEventStampNs(
    const SensorEvent& event,
    const rclcpp::Time& interrupt_ros_at,
    std::optional<int64_t> expected_interval_ns)
{
  const auto timestampIndex = static_cast<std::size_t>(event.report_id);
  if (!m_bnoCadenceEpochNs.has_value())
    m_bnoCadenceEpochNs = HostAnchorNs(event, interrupt_ros_at.nanoseconds());

  const ReportTimestampTrackerResult trackedStamp =
      m_timestampTrackers[timestampIndex].Update(ReportTimestampTrackerInput{
          event.sequence, expected_interval_ns, interrupt_ros_at.nanoseconds(), event.has_delay,
          event.delay_us, m_bnoCadenceEpochNs});

  int64_t stampNs = trackedStamp.stamp_ns;
  bool duplicate = trackedStamp.duplicate_sequence;

  const std::optional<int64_t>& lastStampNs = m_lastEmittedTimestampNs[timestampIndex];
  if (duplicate && lastStampNs.has_value())
    return std::nullopt;

  if (lastStampNs.has_value() && stampNs <= *lastStampNs)
  {
    if (trackedStamp.duplicate_sequence)
    {
      duplicate = true;
      stampNs = *lastStampNs;
    }
    else if (expected_interval_ns.has_value() && *expected_interval_ns > 0)
    {
      stampNs = *lastStampNs + *expected_interval_ns;
    }
    else
    {
      stampNs = *lastStampNs + 1;
    }

    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "BNO086 cadence timestamp guard adjusted report=%s "
        "sequence=%u candidate_ns=%lld last_ns=%lld final_ns=%lld "
        "duplicate_sequence=%s sequence_delta=%u",
        ReportName(event.report_id), event.sequence, static_cast<long long>(trackedStamp.stamp_ns),
        static_cast<long long>(*lastStampNs), static_cast<long long>(stampNs),
        trackedStamp.duplicate_sequence ? "true" : "false", trackedStamp.sequence_delta);
  }

  m_lastEmittedTimestampNs[timestampIndex] = stampNs;
  return duplicate ? std::nullopt : std::make_optional(stampNs);
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

void Bno086ImuNode::MaybeLogOrientationCovariancePolicy(
    const OrientationCovariancePolicyResult& covariance_policy)
{
  const bool sourceChanged = !m_loggedOrientationCovarianceSource ||
                             covariance_policy.source != m_lastOrientationCovarianceSource;
  const bool bucketChanged = !m_loggedOrientationCovarianceSource ||
                             covariance_policy.accuracy_bucket != m_lastOrientationAccuracyBucket;

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

    m_loggedOrientationCovarianceSource = true;
    m_lastOrientationCovarianceSource = covariance_policy.source;
    m_lastOrientationAccuracyBucket = covariance_policy.accuracy_bucket;
  }
}
