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
constexpr const char* ACCEL_TOPIC = "accel";
constexpr const char* DEFAULT_FRAME_ID = "imu_link";

constexpr int INTERRUPT_WAIT_TIMEOUT_MS = 20;
constexpr int PACKET_READ_TIMEOUT_MS = 2;
constexpr int MAX_PACKETS_PER_INTERRUPT = 128;

constexpr std::uint32_t MAX_BASE_TIMESTAMP_STEP_US = 1'000'000;
constexpr std::uint32_t MIN_COHERENT_SAMPLE_SPAN_US = 3'000;
constexpr std::uint32_t MAX_COHERENT_SAMPLE_SPAN_US = 20'000;
constexpr double DEFAULT_PREDICTION_HORIZON_SEC = 0.0;
constexpr double PI_RAD = 3.14159265358979323846;
constexpr const char* ORIENTATION_REPORT_SOURCE = "rotation_vector";
constexpr int ORIENTATION_COVARIANCE_LOG_THROTTLE_MS = 5'000;
} // namespace

Bno086ImuNode::Bno086ImuNode() : rclcpp::Node(NODE_NAME)
{
  declare_parameter("i2c_device", std::string(DEFAULT_I2C_DEVICE));
  declare_parameter("i2c_address", static_cast<int>(DEFAULT_I2C_ADDRESS));
  declare_parameter("int_gpio", DEFAULT_INT_GPIO);
  declare_parameter("report_rate_hz", DEFAULT_REPORT_RATE_HZ);
  declare_parameter("prediction_horizon_sec", DEFAULT_PREDICTION_HORIZON_SEC);

  declare_parameter("frame_id", std::string(DEFAULT_FRAME_ID));

  const std::string i2cDevice = get_parameter("i2c_device").as_string();
  const std::uint8_t i2cAddress = static_cast<std::uint8_t>(get_parameter("i2c_address").as_int());
  const int intGpio = get_parameter("int_gpio").as_int();
  const double reportRateHz = std::max(get_parameter("report_rate_hz").as_double(), 1.0);
  m_predictionHorizonSec = std::max(get_parameter("prediction_horizon_sec").as_double(), 0.0);
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
  m_accelPublisher = create_publisher<geometry_msgs::msg::AccelWithCovarianceStamped>(
      ACCEL_TOPIC, rclcpp::SensorDataQoS{});

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

  if (m_latestFrame.has_accel)
  {
    geometry_msgs::msg::AccelWithCovarianceStamped accelMsg;
    accelMsg.header = imuMsg.header;
    accelMsg.accel.accel.linear.x = m_latestFrame.accel_mps2[0];
    accelMsg.accel.accel.linear.y = m_latestFrame.accel_mps2[1];
    accelMsg.accel.accel.linear.z = m_latestFrame.accel_mps2[2];

    accelMsg.accel.accel.angular.x = 0.0;
    accelMsg.accel.accel.angular.y = 0.0;
    accelMsg.accel.accel.angular.z = 0.0;

    accelMsg.accel.covariance.fill(0.0);

    // `accel` publishes linear acceleration in `accel.accel.linear`.
    // Angular acceleration is intentionally not estimated and is marked
    // unknown by covariance[21] = -1.0.
    if (m_latestFrame.has_linear_accel_covariance)
    {
      SetLinearAccelCovariance(accelMsg.accel.covariance, m_latestFrame.linear_accel_cov_mps2_2);
    }
    else
    {
      accelMsg.accel.covariance[21] = -1.0;
    }

    m_accelPublisher->publish(accelMsg);
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
      const OrientationCovariancePolicyResult covariancePolicy =
          ResolveOrientationCovariancePolicy(event.accuracy, event.values[4]);

      m_latestFrame.orientation_xyzw[0] = QToDouble(event.values[0], 14);
      m_latestFrame.orientation_xyzw[1] = QToDouble(event.values[1], 14);
      m_latestFrame.orientation_xyzw[2] = QToDouble(event.values[2], 14);
      m_latestFrame.orientation_xyzw[3] = QToDouble(event.values[3], 14);
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
      m_orientationCovarianceDebug.sigma_rad = covariancePolicy.sigma_rad;
      m_orientationCovarianceDebug.source = covariancePolicy.source;
      m_orientationCovarianceDebug.has_accuracy_estimate = covariancePolicy.has_accuracy_estimate;
      m_orientationCovarianceDebug.accuracy_estimate_rad = covariancePolicy.accuracy_estimate_rad;
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

void Bno086ImuNode::SetLinearAccelCovariance(std::array<double, 36>& dst,
                                             const OASIS::IMU::Mat3& linear_cov)
{
  dst.fill(0.0);

  dst[0] = linear_cov[0][0];
  dst[1] = linear_cov[0][1];
  dst[2] = linear_cov[0][2];

  dst[6] = linear_cov[1][0];
  dst[7] = linear_cov[1][1];
  dst[8] = linear_cov[1][2];

  dst[12] = linear_cov[2][0];
  dst[13] = linear_cov[2][1];
  dst[14] = linear_cov[2][2];

  // geometry_msgs/AccelWithCovariance uses the rotational block for
  // angular acceleration. Mark it unknown because this topic carries
  // only linear acceleration.
  dst[21] = -1.0;
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
                "sigma_rad=%.4f sigma_deg=%.2f estimate_rad=%.4f",
                ORIENTATION_REPORT_SOURCE,
                static_cast<unsigned>(m_orientationCovarianceDebug.accuracy_bucket),
                OrientationCovarianceSourceName(m_orientationCovarianceDebug.source),
                m_orientationCovarianceDebug.sigma_rad,
                m_orientationCovarianceDebug.sigma_rad * 180.0 / PI_RAD,
                m_orientationCovarianceDebug.has_accuracy_estimate
                    ? m_orientationCovarianceDebug.accuracy_estimate_rad
                    : 0.0);

    m_loggedOrientationCovarianceSource = true;
    m_lastOrientationCovarianceSource = m_orientationCovarianceDebug.source;
    m_lastOrientationAccuracyBucket = m_orientationCovarianceDebug.accuracy_bucket;
  }

  RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), ORIENTATION_COVARIANCE_LOG_THROTTLE_MS,
                        "BNO086 orientation covariance status: report=%s bucket=%u source=%s "
                        "sigma_rad=%.4f sigma_deg=%.2f variance_rad2=%.6f estimate_rad=%.4f",
                        ORIENTATION_REPORT_SOURCE,
                        static_cast<unsigned>(m_orientationCovarianceDebug.accuracy_bucket),
                        OrientationCovarianceSourceName(m_orientationCovarianceDebug.source),
                        m_orientationCovarianceDebug.sigma_rad,
                        m_orientationCovarianceDebug.sigma_rad * 180.0 / PI_RAD,
                        m_latestFrame.orientation_cov_rad2[0][0],
                        m_orientationCovarianceDebug.has_accuracy_estimate
                            ? m_orientationCovarianceDebug.accuracy_estimate_rad
                            : 0.0);
}
