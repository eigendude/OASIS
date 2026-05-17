/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#pragma once

#include "imu/bno086/Bno086DrainPolicy.hpp"
#include "imu/bno086/Bno086Gpio.hpp"
#include "imu/bno086/Bno086GravityUtils.hpp"
#include "imu/bno086/Bno086OrientationCovariancePolicy.hpp"
#include "imu/bno086/Bno086ReportTimestampTracker.hpp"
#include "imu/bno086/Bno086SampleCoherence.hpp"
#include "imu/bno086/Bno086TimestampMapper.hpp"
#include "imu/bno086/Bno086TimestampNormalizer.hpp"
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
 * report_rate_hz configures BNO086 internal report timing only
 *
 * Orientation uses the BNO086 Rotation Vector output, which is
 * magnetometer-backed internally for heading. Magnetometer is not published.
 */
class Bno086ImuNode : public rclcpp::Node
{
public:
  Bno086ImuNode();

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

  struct CoreFrameSignature
  {
    std::uint8_t orientation_sequence{0};
    std::uint8_t gyro_sequence{0};
    std::uint8_t linear_accel_sequence{0};
    int64_t orientation_stamp_ns{0};
    int64_t gyro_stamp_ns{0};
    int64_t linear_accel_stamp_ns{0};
  };

  struct ImuGravityDiagnostics
  {
    std::uint64_t calibrated_accel_reports_received{0};
    std::uint64_t imu_gravity_published{0};
    std::uint64_t imu_gravity_skipped_missing_orientation{0};
    std::uint64_t imu_gravity_skipped_missing_gyro{0};
    std::uint64_t imu_gravity_skipped_stale_orientation{0};
    std::uint64_t imu_gravity_skipped_stale_gyro{0};
    std::uint64_t imu_gravity_skipped_future_orientation{0};
    std::uint64_t imu_gravity_skipped_future_gyro{0};
    std::uint64_t imu_gravity_skipped_duplicate_stamp{0};
    std::uint64_t imu_gravity_skipped_nonfinite{0};
    std::uint64_t timestamp_repaired_nonmonotonic_accel{0};
    std::uint64_t timestamp_repaired_duplicate_to_interval{0};
    std::uint64_t timestamp_repaired_nonmonotonic_to_interval{0};
    std::uint64_t timestamp_repaired_sequence_gap_to_interval{0};
    std::uint64_t timestamp_interval_repair_clamped_to_host{0};
    std::uint64_t timestamp_interval_repair_bounded_to_legacy{0};
    std::uint64_t timestamp_reconstruction_reset_count{0};
    std::uint64_t accel_sequence_gap_count{0};
    std::uint64_t linear_accel_events_seen{0};
    std::uint64_t imu_published{0};
    std::uint64_t imu_skipped_missing_core_frame{0};
    std::uint64_t imu_skipped_incoherent_core_frame{0};
    std::uint64_t imu_skipped_incoherent_core_frame_span{0};
    std::uint64_t imu_skipped_duplicate_core_signature{0};
    std::uint32_t latest_timestamp_repair_interval_us{0};
    double latest_orientation_age_ms{0.0};
    double latest_gyro_age_ms{0.0};
    double latest_core_span_ms{0.0};
    int64_t latest_accel_stamp_ns{0};
    int64_t latest_orientation_stamp_ns{0};
    int64_t latest_gyro_stamp_ns{0};
    int64_t latest_linear_accel_stamp_ns{0};
    double latest_calibrated_accel_rate_hz{0.0};
    double latest_imu_gravity_rate_hz{0.0};
    std::array<double, 5> latest_decoded_rate_hz{};
    std::array<double, 5> latest_feature_rate_hz{};
    std::array<std::uint64_t, 5> decoded_reports_received{};
    std::array<std::uint64_t, 5> last_rate_decoded_reports{};
    std::uint64_t latest_skipped_stale_orientation_delta{0};
    std::uint64_t latest_skipped_stale_gyro_delta{0};
    std::uint64_t latest_accel_sequence_gap_delta{0};
    std::uint64_t latest_timestamp_reconstruction_reset_delta{0};
    std::uint64_t last_rate_accel_reports{0};
    std::uint64_t last_rate_imu_gravity_published{0};
    std::uint64_t last_rate_skipped_stale_orientation{0};
    std::uint64_t last_rate_skipped_stale_gyro{0};
    std::uint64_t last_rate_accel_sequence_gap_count{0};
    std::uint64_t last_rate_timestamp_reconstruction_reset_count{0};
    std::chrono::steady_clock::time_point last_log_at{};
  };

  struct TimestampTraceStats
  {
    std::uint64_t events{0};
    std::uint64_t has_base_count{0};
    std::uint64_t has_delay_count{0};
    std::uint64_t sequence_gap_count{0};
    std::uint64_t cadence_tracker_gap_count{0};
    std::uint64_t cadence_tracker_reanchor_count{0};
    std::uint64_t monotonic_guard_count{0};
    std::uint64_t duplicate_sequence_count{0};
    std::uint64_t duplicate_raw_stamp_count{0};
    std::uint64_t tiny_raw_delta_count{0};
    std::uint64_t duplicate_normalized_stamp_count{0};
    std::uint64_t legacy_plus_one_repair_count{0};
    std::uint64_t interval_repair_count{0};
    std::uint64_t host_clamp_count{0};
    std::uint64_t bounded_to_legacy_count{0};
    std::uint64_t raw_delta_count{0};
    std::uint64_t normalized_delta_count{0};
    int64_t raw_delta_sum_ns{0};
    int64_t normalized_delta_sum_ns{0};
    std::optional<int64_t> raw_delta_min_ns;
    std::optional<int64_t> normalized_delta_min_ns;
    std::optional<int64_t> last_raw_stamp_ns;
    std::optional<int64_t> last_normalized_stamp_ns;
  };

  struct EstimatedEventStamp
  {
    rclcpp::Time stamp;
    OASIS::IMU::BNO086::ReportTimestampTrackerResult tracker;
  };

  struct OrientationCovarianceDebugState
  {
    std::uint8_t accuracy_bucket{0};
    std::int16_t raw_accuracy_estimate_q12{0};
    double sigma_rad{0.0};
    OASIS::IMU::BNO086::OrientationCovarianceSource source{
        OASIS::IMU::BNO086::OrientationCovarianceSource::AccuracyBucketFallback};
    bool has_accuracy_estimate{false};
    double accuracy_estimate_rad{0.0};
    OASIS::IMU::BNO086::OrientationCovarianceEstimateRejectionReason rejection_reason{
        OASIS::IMU::BNO086::OrientationCovarianceEstimateRejectionReason::None};
  };

  void InterruptLoop();
  void DrainPacketsForInterrupt(const std::chrono::steady_clock::time_point& interrupt_steady_at,
                                const rclcpp::Time& interrupt_ros_at);
  void MaybePublishOnLinearAcceleration(const OASIS::IMU::BNO086::SensorEvent& event);
  void MaybePublishImuGravityOnAccelerometer(const OASIS::IMU::BNO086::SensorEvent& event);
  void MaybePublishGravityOnGravityReport(const OASIS::IMU::BNO086::SensorEvent& event);
  void PublishLatestFrame(const rclcpp::Time& stamp);
  std::uint32_t CoreCoherenceToleranceUs() const;
  int64_t ReportFutureToleranceNs(OASIS::IMU::BNO086::ReportId report_id) const;
  bool HasPublishableCoreFrame() const;
  CoreFrameSignature LatestCoreSignature() const;
  rclcpp::Time LatestCoreStamp() const;

  void ApplyEvent(const OASIS::IMU::BNO086::SensorEvent& event, const rclcpp::Time& sample_stamp);
  EstimatedEventStamp EstimateEventStamp(const OASIS::IMU::BNO086::SensorEvent& event,
                                         const rclcpp::Time& interrupt_ros_at,
                                         std::optional<int64_t> expected_interval_ns);
  OASIS::IMU::BNO086::TimestampNormalizationResult FinalizeEventStamp(
      const OASIS::IMU::BNO086::SensorEvent& event,
      const EstimatedEventStamp& estimated_stamp,
      int64_t packet_host_stamp_ns,
      std::optional<int64_t> expected_interval_ns);
  sensor_msgs::msg::Imu BuildPresentImuMessage(const rclcpp::Time& stamp) const;
  sensor_msgs::msg::Imu BuildImuGravityMessage(const rclcpp::Time& stamp) const;
  geometry_msgs::msg::AccelWithCovarianceStamped BuildGravityMessage(
      const rclcpp::Time& stamp) const;
  sensor_msgs::msg::Imu BuildPredictedImuMessage(const sensor_msgs::msg::Imu& present_imu) const;
  oasis_msgs::msg::ImuVr BuildPredictedVrMessage(const sensor_msgs::msg::Imu& present_imu,
                                                 const sensor_msgs::msg::Imu& predicted_imu) const;
  void MaybeLogFeatureResponses();
  void LogFeatureResponse(const OASIS::IMU::BNO086::FeatureResponse& response);
  void MaybeLogFeatureSummary();
  void LogFeatureSummary() const;
  std::string BuildFeatureSummaryLine(
      const OASIS::IMU::BNO086::FeatureConfiguration& configuration,
      const std::optional<OASIS::IMU::BNO086::FeatureResponse>& response) const;
  void MaybeLogImuGravityDiagnostics();
  void UpdateImuGravityDiagnosticsRates(const std::chrono::steady_clock::time_point& now);
  void MaybeEmitImuGravityDiagnosticsLog();
  std::string BuildImuGravityDiagnosticsLogMessage() const;
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
  bool IsBno086RateUnhealthy() const;
  bool IsBno086DiagnosticsUnhealthy() const;
  void CountDecodedReport(const OASIS::IMU::BNO086::SensorEvent& event);
  void RecordTimestampTrace(const OASIS::IMU::BNO086::SensorEvent& event,
                            int64_t raw_stamp_ns,
                            int64_t packet_host_stamp_ns,
                            const OASIS::IMU::BNO086::TimestampNormalizationResult& normalized,
                            const OASIS::IMU::BNO086::ReportTimestampTrackerResult& tracker,
                            std::optional<std::uint32_t> expected_interval_us);
  void MaybeLogTimestampTraceLine(
      const OASIS::IMU::BNO086::SensorEvent& event,
      int64_t raw_stamp_ns,
      int64_t packet_host_stamp_ns,
      const OASIS::IMU::BNO086::TimestampNormalizationResult& normalized,
      std::optional<std::uint32_t> expected_interval_us,
      std::optional<int64_t> previous_raw_delta_ns,
      std::optional<int64_t> previous_normalized_delta_ns);
  void MaybeEmitTimestampSummaries() const;

  static std::array<double, 9> PredictedCovarianceFromPresent(
      const std::array<double, 9>& present_orientation_covariance,
      double prediction_horizon_sec,
      double& sigma_noise_rad,
      double& sigma_rms_rad,
      double& sigma_bound_rad);
  static rclcpp::Duration DurationFromUs(std::uint32_t microseconds);
  static OASIS::IMU::Mat3 CovarianceFromAccuracyBucket(std::uint8_t accuracy,
                                                       double sigma_unreliable,
                                                       double sigma_low,
                                                       double sigma_medium,
                                                       double sigma_high);
  static const char* ReportName(OASIS::IMU::BNO086::ReportId report_id);

  static double QToDouble(std::int16_t value, unsigned q_point);
  static void NormalizeQuaternion(std::array<double, 4>& q);
  static std::array<double, 4> MultiplyQuaternion(const std::array<double, 4>& lhs,
                                                  const std::array<double, 4>& rhs);
  static std::array<double, 4> PredictOrientation(const std::array<double, 4>& orientation_xyzw,
                                                  const OASIS::IMU::Vec3& gyro_rads,
                                                  double prediction_horizon_sec);
  static void SetCovariance(std::array<double, 9>& dst, const OASIS::IMU::Mat3& src);
  static std::optional<std::size_t> DiagnosticReportIndex(OASIS::IMU::BNO086::ReportId report_id);
  void MaybeLogOrientationCovariancePolicy();

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
  OrientationCovarianceDebugState m_orientationCovarianceDebug{};

  std::string m_frameId;
  double m_predictionHorizonSec{0.0};
  std::string m_predictionSource{"rotation_vector_plus_host_prediction"};

  std::atomic<bool> m_running{false};
  std::thread m_interruptThread;

  std::array<OASIS::IMU::BNO086::Bno086ReportTimestampTracker, 256> m_timestampTrackers{};
  std::optional<int64_t> m_bnoCadenceEpochNs;
  OASIS::IMU::BNO086::Bno086TimestampMapper m_timestampMapper;
  std::array<OASIS::IMU::BNO086::Bno086TimestampNormalizer, 256> m_timestampNormalizers{};
  std::array<std::optional<int64_t>, 256> m_lastEmittedTimestampNs{};
  std::optional<CoreFrameSignature> m_lastPublishedCoreSignature;
  std::optional<int64_t> m_lastPublishedImuGravityAccelStampNs;
  std::uint32_t m_reportIntervalUs{10'000};
  int m_packetReadTimeoutMs{5};
  int m_featureResponseStartupDrainMs{250};
  std::uint32_t m_featureResponseStartupMaxPackets{128};
  std::uint32_t m_maxPacketsPerInterrupt{1024};
  std::uint32_t m_maxPollIterationsPerInterrupt{4096};
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
  double m_imuGravityMaxOrientationAgeMs{25.0};
  double m_imuGravityMaxGyroAgeMs{25.0};
  std::uint32_t m_repeatedNoProgressTimeouts{0};
  ImuGravityDiagnostics m_imuGravityDiagnostics{};
  bool m_diagnosticsWasUnhealthy{false};

  bool m_loggedCommEstablished{false};
  bool m_loggedSetFeature{false};
  bool m_loggedFeatureSummary{false};
  bool m_loggedTimestampIntervalRepair{false};
  bool m_warnedMissingImuFields{false};
  bool m_loggedOrientationCovarianceSource{false};
  OASIS::IMU::BNO086::OrientationCovarianceSource m_lastOrientationCovarianceSource{
      OASIS::IMU::BNO086::OrientationCovarianceSource::AccuracyBucketFallback};
  std::uint8_t m_lastOrientationAccuracyBucket{0};
  std::uint32_t m_timestampTraceCount{0};
  std::uint32_t m_timestampTraceLogged{0};
  std::array<TimestampTraceStats, 5> m_timestampTraceStats{};
  std::chrono::steady_clock::time_point m_featureConfigurationStartedAt{};
  std::vector<OASIS::IMU::BNO086::FeatureResponse> m_latestFeatureResponses;
};
} // namespace OASIS::ROS
