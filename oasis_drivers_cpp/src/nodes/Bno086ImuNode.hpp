/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#pragma once

#include "imu/bno086/core/Bno086DrainPolicy.hpp"
#include "imu/bno086/core/Bno086GravityUtils.hpp"
#include "imu/bno086/core/Bno086OrientationCovariancePolicy.hpp"
#include "imu/bno086/core/Bno086ReportTimestampTracker.hpp"
#include "imu/bno086/core/Bno086SampleCoherence.hpp"
#include "imu/bno086/gpio/Bno086Gpio.hpp"
#include "imu/bno086/sh2/Bno086Reports.hpp"
#include "imu/bno086/sh2/Bno086Shtp.hpp"
#include "imu/bno086/shtp/Bno086Transport.hpp"

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#include <oasis_msgs/msg/imu_vr.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <sensor_msgs/msg/imu.hpp>

namespace OASIS::ROS
{
/*!
 * \brief BNO086 IMU driver node
 *
 * Publishes:
 * - imu: fused orientation + calibrated gyro + gravity-removed linear accel
 * - imu_predicted: predicted orientation + present-time gyro + linear accel
 * - imu_vr: predicted orientation plus explicit prediction metadata
 * - gravity (optional): fused gravity vector expressed in imu_link and
 *   pointing down with magnitude near 9.81 m/s^2 at rest
 * - imu_gravity (optional): fused orientation + calibrated gyro +
 *   gravity-included calibrated acceleration
 *
 * Publication is interrupt-driven from active-low H_INTN packet drains
 *
 * Orientation uses the BNO086 Rotation Vector output, which is
 * magnetometer-backed internally for heading. Magnetometer is not published.
 */
class Bno086ImuNode : public rclcpp::Node
{
public:
  Bno086ImuNode();
  ~Bno086ImuNode() override;

  bool Initialize();
  void Deinitialize();

private:
  struct SampleState
  {
    bool has_sample{false};
    rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
    std::uint8_t sequence{0};
    std::uint8_t accuracy{0};
  };

  struct ImuGravityAccelSample;

  struct CoreFrameSignature
  {
    std::uint8_t orientation_sequence{0};
    std::uint8_t gyro_sequence{0};
    std::uint8_t linear_accel_sequence{0};
    int64_t orientation_stamp_ns{0};
    int64_t gyro_stamp_ns{0};
    int64_t linear_accel_stamp_ns{0};
  };

  class ImuGravityAccelHistory;
  class Bno086DrainHealth;
  struct RateDiagnostics;
  struct RateSnapshot;

  void InterruptLoop();
  void DrainPacketsForInterrupt(const std::chrono::steady_clock::time_point& interrupt_steady_at,
                                const rclcpp::Time& interrupt_ros_at);
  void MaybePublishOnLinearAcceleration(const OASIS::IMU::BNO086::SensorEvent& event);
  void MaybePublishImuGravityOnRotationVector(const OASIS::IMU::BNO086::SensorEvent& event);
  void MaybePublishGravityOnGravityReport(const OASIS::IMU::BNO086::SensorEvent& event);
  void PublishLatestFrame(const rclcpp::Time& stamp);
  std::uint32_t CoreCoherenceToleranceUs() const;
  int64_t ImuGravityMaxOrientationAgeNs() const;
  int64_t ImuGravityMaxGyroAgeNs() const;
  int64_t ReportFutureToleranceNs(OASIS::IMU::BNO086::ReportId report_id) const;
  void RecordImuGravityAccelSample(const rclcpp::Time& sample_stamp);
  std::optional<ImuGravityAccelSample> SelectImuGravityAccelSample(
      int64_t anchor_stamp_ns, int64_t future_tolerance_ns) const;
  CoreFrameSignature LatestCoreSignature() const;
  rclcpp::Time LatestCoreStamp() const;

  void ApplyEvent(const OASIS::IMU::BNO086::SensorEvent& event, const rclcpp::Time& sample_stamp);
  std::optional<int64_t> FinalizeEventStampNs(const OASIS::IMU::BNO086::SensorEvent& event,
                                              const rclcpp::Time& interrupt_ros_at,
                                              std::optional<int64_t> expected_interval_ns);
  sensor_msgs::msg::Imu BuildPresentImuMessage(const rclcpp::Time& stamp) const;
  sensor_msgs::msg::Imu BuildImuGravityMessage(const rclcpp::Time& stamp,
                                               const ImuGravityAccelSample& accel_sample) const;
  geometry_msgs::msg::AccelWithCovarianceStamped BuildGravityMessage(
      const rclcpp::Time& stamp) const;
  sensor_msgs::msg::Imu BuildPredictedImuMessage(const sensor_msgs::msg::Imu& present_imu) const;
  oasis_msgs::msg::ImuVr BuildPredictedVrMessage(const sensor_msgs::msg::Imu& present_imu,
                                                 const sensor_msgs::msg::Imu& predicted_imu) const;
  void MaybeLogFeatureResponses();
  void MaybeLogFeatureSummary();
  void LogFeatureSummary() const;
  std::string BuildFeatureSummaryLine(
      const OASIS::IMU::BNO086::FeatureConfiguration& configuration,
      const std::optional<OASIS::IMU::BNO086::FeatureResponse>& response) const;
  void MaybeLogImuGravityDiagnostics();
  void RecordDrainThroughputDiagnostics(const OASIS::IMU::BNO086::Bno086DrainCounters& counters,
                                        OASIS::IMU::BNO086::Bno086DrainAction exit_action,
                                        bool,
                                        std::uint32_t drain_duration_us,
                                        std::uint32_t);
  RateSnapshot UpdateImuGravityDiagnosticsRates(const std::chrono::steady_clock::time_point& now);
  void MaybeEmitImuGravityDiagnosticsLog(const RateSnapshot& rates);
  bool IsImuGravitySampleValid(const sensor_msgs::msg::Imu& message,
                               std::string& invalid_reason) const;
  std::optional<std::uint32_t> RequestedFeatureIntervalUs(
      OASIS::IMU::BNO086::ReportId report_id) const;
  std::optional<std::uint32_t> RequestedFeatureBatchIntervalUs(
      OASIS::IMU::BNO086::ReportId report_id) const;
  std::optional<std::uint32_t> EffectiveReportIntervalUs(
      OASIS::IMU::BNO086::ReportId report_id) const;
  std::optional<OASIS::IMU::BNO086::FeatureResponse> LatestFeatureResponse(
      OASIS::IMU::BNO086::ReportId report_id) const;
  bool IsBno086DiagnosticsUnhealthy(const RateSnapshot& rates) const;
  void CountDecodedReport(const OASIS::IMU::BNO086::SensorEvent& event);

  static std::array<double, 9> PredictedCovarianceFromPresent(
      const std::array<double, 9>& present_orientation_covariance,
      double prediction_horizon_sec,
      double& sigma_noise_rad,
      double& sigma_rms_rad,
      double& sigma_bound_rad);
  static OASIS::IMU::Mat3 CovarianceFromAccuracyBucket(std::uint8_t accuracy,
                                                       double sigma_unreliable,
                                                       double sigma_low,
                                                       double sigma_medium,
                                                       double sigma_high);

  static double QToDouble(std::int16_t value, unsigned q_point);
  static void NormalizeQuaternion(std::array<double, 4>& q);
  static std::array<double, 4> MultiplyQuaternion(const std::array<double, 4>& lhs,
                                                  const std::array<double, 4>& rhs);
  static std::array<double, 4> PredictOrientation(const std::array<double, 4>& orientation_xyzw,
                                                  const OASIS::IMU::Vec3& gyro_rads,
                                                  double prediction_horizon_sec);
  static void SetCovariance(std::array<double, 9>& dst, const OASIS::IMU::Mat3& src);
  void MaybeLogOrientationCovariancePolicy(
      const OASIS::IMU::BNO086::OrientationCovariancePolicyResult& covariance_policy);

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr m_imuPublisher;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr m_imuPredictedPublisher;
  rclcpp::Publisher<oasis_msgs::msg::ImuVr>::SharedPtr m_imuVrPublisher;
  rclcpp::Publisher<geometry_msgs::msg::AccelWithCovarianceStamped>::SharedPtr m_gravityPublisher;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr m_imuGravityPublisher;

  OASIS::IMU::BNO086::Bno086Transport m_transport;
  OASIS::IMU::BNO086::Bno086Gpio m_interruptGpio;
  std::unique_ptr<OASIS::IMU::BNO086::Bno086Shtp> m_shtp;
  OASIS::IMU::BNO086::ImuSampleFrame m_latestFrame;
  SampleState m_orientationState{};
  SampleState m_gyroState{};
  SampleState m_linearAccelState{};
  SampleState m_imuGravityState{};
  SampleState m_gravityState{};
  std::unique_ptr<ImuGravityAccelHistory> m_imuGravityAccelHistory;

  std::string m_frameId;
  double m_predictionHorizonSec{0.0};
  std::string m_predictionSource{"rotation_vector_plus_host_prediction"};

  std::atomic<bool> m_running{false};
  std::thread m_interruptThread;

  std::array<OASIS::IMU::BNO086::Bno086ReportTimestampTracker, 256> m_timestampTrackers{};
  std::optional<int64_t> m_bnoCadenceEpochNs;
  std::array<std::optional<int64_t>, 256> m_lastEmittedTimestampNs{};
  std::optional<CoreFrameSignature> m_lastPublishedCoreSignature;
  std::optional<int64_t> m_lastPublishedImuGravityAnchorStampNs;
  int m_packetReadTimeoutMs{5};
  int m_featureResponseStartupDrainMs{250};
  std::uint32_t m_featureResponseStartupMaxPackets{128};
  std::uint32_t m_maxPacketsPerInterrupt{1024};
  std::uint32_t m_maxPollIterationsPerInterrupt{4096};
  std::uint32_t m_maxNoProgressPollsPerInterrupt{64};
  std::uint32_t m_maxAllZeroPollsPerInterrupt{64};
  int m_allZeroBackoffUs{500};
  int m_maxDrainDurationMs{100};
  std::uint32_t m_maxSensorEventsPerDrain{4096};
  std::uint32_t m_maxPendingEventsFlushPerDrain{1024};
  double m_rotationVectorRateHz{100.0};
  double m_gyroRateHz{100.0};
  double m_accelerometerRateHz{100.0};
  double m_linearAccelerationRateHz{50.0};
  double m_gravityRateHz{25.0};
  std::uint32_t m_rotationVectorBatchIntervalUs{0};
  std::uint32_t m_gyroBatchIntervalUs{0};
  std::uint32_t m_accelerometerBatchIntervalUs{0};
  std::uint32_t m_linearAccelerationBatchIntervalUs{0};
  std::uint32_t m_gravityBatchIntervalUs{0};
  bool m_enableLinearAccelerationReport{true};
  bool m_enableGravityReport{true};
  int m_featureSummaryTimeoutMs{5000};
  int m_diagnosticsLogPeriodMs{5000};
  double m_imuGravityMaxOrientationAgeMs{80.0};
  double m_imuGravityMaxGyroAgeMs{80.0};
  std::uint32_t m_repeatedNoProgressTimeouts{0};
  std::unique_ptr<RateDiagnostics> m_rateDiagnostics;
  bool m_diagnosticsWasUnhealthy{false};

  bool m_loggedCommEstablished{false};
  bool m_loggedSetFeature{false};
  bool m_loggedFeatureSummary{false};
  bool m_warnedMissingImuFields{false};
  bool m_loggedOrientationCovarianceSource{false};
  OASIS::IMU::BNO086::OrientationCovarianceSource m_lastOrientationCovarianceSource{
      OASIS::IMU::BNO086::OrientationCovarianceSource::AccuracyBucketFallback};
  std::uint8_t m_lastOrientationAccuracyBucket{0};
  std::unique_ptr<Bno086DrainHealth> m_drainHealth;
  std::chrono::steady_clock::time_point m_featureConfigurationStartedAt{};
  std::vector<OASIS::IMU::BNO086::FeatureResponse> m_latestFeatureResponses;
};
} // namespace OASIS::ROS
