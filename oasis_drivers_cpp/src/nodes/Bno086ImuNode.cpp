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
#include <stdexcept>

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
constexpr const char* GRAVITY_TOPIC = "gravity";
constexpr const char* ACCEL_TOPIC = "accel";
constexpr const char* DEFAULT_FRAME_ID = "imu_link";

constexpr int INTERRUPT_WAIT_TIMEOUT_MS = 20;
constexpr int PACKET_READ_TIMEOUT_MS = 2;
constexpr int MAX_PACKETS_PER_INTERRUPT = 128;

constexpr std::uint32_t MAX_BASE_TIMESTAMP_STEP_US = 1'000'000;
constexpr std::uint32_t MIN_COHERENT_SAMPLE_SPAN_US = 3'000;
constexpr std::uint32_t MAX_COHERENT_SAMPLE_SPAN_US = 20'000;

rclcpp::Duration DurationFromUs(std::uint32_t microseconds)
{
  return rclcpp::Duration(0, static_cast<int64_t>(microseconds) * 1000);
}
} // namespace

Bno086ImuNode::Bno086ImuNode() : rclcpp::Node(NODE_NAME)
{
  declare_parameter("i2c_device", std::string(DEFAULT_I2C_DEVICE));
  declare_parameter("i2c_address", static_cast<int>(DEFAULT_I2C_ADDRESS));
  declare_parameter("int_gpio", DEFAULT_INT_GPIO);
  declare_parameter("report_rate_hz", DEFAULT_REPORT_RATE_HZ);

  declare_parameter("frame_id", std::string(DEFAULT_FRAME_ID));

  const std::string i2cDevice = get_parameter("i2c_device").as_string();
  const std::uint8_t i2cAddress = static_cast<std::uint8_t>(get_parameter("i2c_address").as_int());
  const int intGpio = get_parameter("int_gpio").as_int();
  const double reportRateHz = std::max(get_parameter("report_rate_hz").as_double(), 1.0);
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
}

bool Bno086ImuNode::Initialize()
{
  m_imuPublisher = create_publisher<sensor_msgs::msg::Imu>(IMU_TOPIC, rclcpp::SensorDataQoS{});
  m_gravityPublisher =
      create_publisher<geometry_msgs::msg::Vector3Stamped>(GRAVITY_TOPIC, rclcpp::SensorDataQoS{});
  m_accelPublisher =
      create_publisher<geometry_msgs::msg::Vector3Stamped>(ACCEL_TOPIC, rclcpp::SensorDataQoS{});

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
  for (int i = 0; i < MAX_PACKETS_PER_INTERRUPT && m_running.load(); ++i)
  {
    std::optional<SensorEvent> event;
    const Bno086Shtp::PollStatus pollStatus = m_shtp->Poll(event, PACKET_READ_TIMEOUT_MS);

    if (pollStatus == Bno086Shtp::PollStatus::Timeout)
      break;

    if (pollStatus == Bno086Shtp::PollStatus::TransportError)
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "BNO086 transport error while draining interrupt data");
      break;
    }

    if (pollStatus == Bno086Shtp::PollStatus::SensorEvent && event.has_value())
    {
      const auto sampleNowSteady = std::chrono::steady_clock::now();
      const auto sampleDeltaNs = std::chrono::duration_cast<std::chrono::nanoseconds>(
                                     sampleNowSteady - interrupt_steady_at)
                                     .count();
      const rclcpp::Time sampleInterruptRosAt =
          interrupt_ros_at + rclcpp::Duration(0, sampleDeltaNs);
      const rclcpp::Time eventStamp = EstimateEventStamp(*event, sampleInterruptRosAt);
      ApplyEvent(*event, eventStamp);
      MaybePublishOnLinearAcceleration(*event);
    }

    if (!m_interruptGpio.IsAssertedLow() && pollStatus == Bno086Shtp::PollStatus::PacketHandled)
      break;
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

  m_imuPublisher->publish(imuMsg);

  if (m_latestFrame.has_gravity)
  {
    geometry_msgs::msg::Vector3Stamped gravityMsg;
    gravityMsg.header = imuMsg.header;
    gravityMsg.vector.x = m_latestFrame.gravity_mps2[0];
    gravityMsg.vector.y = m_latestFrame.gravity_mps2[1];
    gravityMsg.vector.z = m_latestFrame.gravity_mps2[2];
    m_gravityPublisher->publish(gravityMsg);
  }

  if (m_latestFrame.has_accel)
  {
    geometry_msgs::msg::Vector3Stamped accelMsg;
    accelMsg.header = imuMsg.header;
    accelMsg.vector.x = m_latestFrame.accel_mps2[0];
    accelMsg.vector.y = m_latestFrame.accel_mps2[1];
    accelMsg.vector.z = m_latestFrame.accel_mps2[2];
    m_accelPublisher->publish(accelMsg);
  }
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
      m_latestFrame.orientation_xyzw[0] = QToDouble(event.values[0], 14);
      m_latestFrame.orientation_xyzw[1] = QToDouble(event.values[1], 14);
      m_latestFrame.orientation_xyzw[2] = QToDouble(event.values[2], 14);
      m_latestFrame.orientation_xyzw[3] = QToDouble(event.values[3], 14);
      NormalizeQuaternion(m_latestFrame.orientation_xyzw);
      m_latestFrame.has_orientation = true;
      m_orientationState.has_sample = true;
      m_orientationState.stamp = sample_stamp;
      m_orientationState.sequence = event.sequence;

      m_latestFrame.orientation_cov_rad2 =
          CovarianceFromAccuracy(event.accuracy, 1.5, 0.8, 0.35, 0.12);
      m_latestFrame.has_orientation_covariance = true;
      break;

    case ReportId::GyroscopeCalibrated:
      m_latestFrame.gyro_rads[0] = QToDouble(event.values[0], 9);
      m_latestFrame.gyro_rads[1] = QToDouble(event.values[1], 9);
      m_latestFrame.gyro_rads[2] = QToDouble(event.values[2], 9);
      m_latestFrame.has_gyro = true;
      m_gyroState.has_sample = true;
      m_gyroState.stamp = sample_stamp;
      m_gyroState.sequence = event.sequence;

      m_latestFrame.gyro_cov_rads2_2 = CovarianceFromAccuracy(event.accuracy, 1.2, 0.5, 0.18, 0.06);
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

      m_latestFrame.linear_accel_cov_mps2_2 =
          CovarianceFromAccuracy(event.accuracy, 4.0, 2.0, 0.8, 0.25);
      m_latestFrame.has_linear_accel_covariance = true;
      break;

    case ReportId::Accelerometer:
      m_latestFrame.accel_mps2[0] = QToDouble(event.values[0], 8);
      m_latestFrame.accel_mps2[1] = QToDouble(event.values[1], 8);
      m_latestFrame.accel_mps2[2] = QToDouble(event.values[2], 8);
      m_latestFrame.has_accel = true;
      break;

    case ReportId::Gravity:
      m_latestFrame.gravity_mps2[0] = QToDouble(event.values[0], 8);
      m_latestFrame.gravity_mps2[1] = QToDouble(event.values[1], 8);
      m_latestFrame.gravity_mps2[2] = QToDouble(event.values[2], 8);
      m_latestFrame.has_gravity = true;
      break;

    default:
      break;
  }
}

rclcpp::Time Bno086ImuNode::EstimateEventStamp(const SensorEvent& event,
                                               const rclcpp::Time& interrupt_ros_at)
{
  rclcpp::Time estimatedStamp = interrupt_ros_at;

  if (event.has_base_timestamp)
  {
    if (m_lastBaseTimestampUs.has_value() && m_lastBaseRosStamp.has_value())
    {
      const std::uint32_t deltaUs = event.base_timestamp_us - *m_lastBaseTimestampUs;

      if (deltaUs <= MAX_BASE_TIMESTAMP_STEP_US)
        estimatedStamp = *m_lastBaseRosStamp + DurationFromUs(deltaUs);
    }

    m_lastBaseTimestampUs = event.base_timestamp_us;
    m_lastBaseRosStamp = estimatedStamp;
  }

  if (event.has_delay)
    estimatedStamp = estimatedStamp - DurationFromUs(event.delay_us);

  return estimatedStamp;
}

OASIS::IMU::Mat3 Bno086ImuNode::CovarianceFromAccuracy(std::uint8_t accuracy,
                                                       double sigma_unreliable,
                                                       double sigma_low,
                                                       double sigma_medium,
                                                       double sigma_high)
{
  // SH-2 exposes only an accuracy class, not a full covariance model.
  // Map that class to an axis-aligned 3x3 covariance and reserve larger
  // uncertainty for accuracy=0 so "unreliable" is distinct from "low".
  const double sigma = (accuracy >= 3)   ? sigma_high
                       : (accuracy == 2) ? sigma_medium
                       : (accuracy == 1) ? sigma_low
                                         : sigma_unreliable;

  OASIS::IMU::Mat3 covariance{};
  covariance[0][0] = sigma * sigma;
  covariance[0][1] = 0.0;
  covariance[0][2] = 0.0;

  covariance[1][0] = 0.0;
  covariance[1][1] = sigma * sigma;
  covariance[1][2] = 0.0;

  covariance[2][0] = 0.0;
  covariance[2][1] = 0.0;
  covariance[2][2] = sigma * sigma;
  return covariance;
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
