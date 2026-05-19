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
#include "imu/bno086/core/Bno086ImuGravityAccelHistory.hpp"
#include "imu/bno086/core/Bno086OrientationCovariancePolicy.hpp"
#include "imu/bno086/core/Bno086SampleCoherence.hpp"
#include "imu/bno086/diagnostics/Bno086DrainHealth.hpp"
#include "imu/bno086/diagnostics/Bno086RateHealth.hpp"
#include "imu/bno086/ros/Bno086NodeParams.hpp"
#include "imu/bno086/sh2/Bno086Reports.hpp"

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

// Forward-declared subsystems
namespace OASIS::IMU::BNO086
{
class Bno086Transport;
class Bno086Gpio;
class Bno086Shtp;
class Bno086TimestampCadence;
} // namespace OASIS::IMU::BNO086

namespace OASIS::ROS
{
/*!
 * \brief BNO086 IMU driver node
 *
 * Publishes:
 * - imu: fused orientation + calibrated gyro + gravity-removed linear accel
 * - imu_predicted: predicted orientation + present-time gyro + linear accel
 * - imu_vr: predicted orientation plus explicit prediction metadata
 * - gravity: fused gravity vector expressed in imu_link and pointing down
 *   with magnitude near 9.81 m/s^2 at rest
 * - imu_gravity: fused orientation + calibrated gyro + gravity-included
 *   calibrated acceleration
 *
 * Publication is interrupt-driven from active-low H_INTN packet drains
 *
 * Required driver outputs follow their source report cadences:
 * - imu: Linear Acceleration
 * - imu_gravity: Rotation Vector
 * - gravity: Gravity
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

  struct CoreFrameSignature
  {
    std::uint8_t orientation_sequence{0};
    std::uint8_t gyro_sequence{0};
    std::uint8_t linear_accel_sequence{0};
    int64_t orientation_stamp_ns{0};
    int64_t gyro_stamp_ns{0};
    int64_t linear_accel_stamp_ns{0};
  };

  struct Bno086StreamState
  {
    OASIS::IMU::BNO086::ImuSampleFrame latest_frame;
    SampleState orientation{};
    SampleState gyro{};
    SampleState linear_accel{};
    SampleState imu_gravity{};
    SampleState gravity{};
    OASIS::IMU::BNO086::Bno086ImuGravityAccelHistory imu_gravity_accel_history;
    std::optional<CoreFrameSignature> last_published_core_signature;
    std::optional<int64_t> last_published_imu_gravity_anchor_stamp_ns;
  };

  struct Bno086DiagnosticsState
  {
    OASIS::IMU::BNO086::Bno086RateHealth rate_health;
    OASIS::IMU::BNO086::Bno086DrainHealth drain_health;
    bool was_unhealthy{false};
    std::uint32_t repeated_no_progress_timeouts{0};
    bool warned_missing_imu_fields{false};
  };

  struct Bno086StartupLogState
  {
    bool logged_comm_established{false};
    bool logged_set_feature{false};
    bool logged_feature_summary{false};
    std::chrono::steady_clock::time_point feature_configuration_started_at{};
    std::vector<OASIS::IMU::BNO086::FeatureResponse> latest_feature_responses;
  };

  struct Bno086CovarianceLogState
  {
    bool logged_orientation_covariance_source{false};
    OASIS::IMU::BNO086::OrientationCovarianceSource last_orientation_covariance_source{
        OASIS::IMU::BNO086::OrientationCovarianceSource::AccuracyBucketFallback};
    std::uint8_t last_orientation_accuracy_bucket{0};
  };

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
  std::optional<OASIS::IMU::BNO086::Bno086ImuGravityAccelSample> SelectImuGravityAccelSample(
      int64_t anchor_stamp_ns, int64_t future_tolerance_ns) const;
  CoreFrameSignature LatestCoreSignature() const;
  rclcpp::Time LatestCoreStamp() const;

  void ApplyEvent(const OASIS::IMU::BNO086::SensorEvent& event, const rclcpp::Time& sample_stamp);
  std::optional<int64_t> FinalizeEventStampNs(const OASIS::IMU::BNO086::SensorEvent& event,
                                              const rclcpp::Time& interrupt_ros_at,
                                              std::optional<int64_t> expected_interval_ns);
  void MaybeLogFeatureResponses();
  void MaybeLogFeatureSummary();
  void LogFeatureSummary() const;
  void MaybeLogImuGravityDiagnostics();
  void RecordDrainThroughputDiagnostics(const OASIS::IMU::BNO086::Bno086DrainCounters& counters,
                                        OASIS::IMU::BNO086::Bno086DrainAction exit_action,
                                        bool,
                                        std::uint32_t drain_duration_us,
                                        std::uint32_t);
  void MaybeEmitImuGravityDiagnosticsLog(const OASIS::IMU::BNO086::Bno086RateSnapshot& rates);
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
  bool IsBno086DiagnosticsUnhealthy(const OASIS::IMU::BNO086::Bno086RateSnapshot& rates) const;
  bool IsImuGravityRateHealthy(const OASIS::IMU::BNO086::Bno086RateSnapshot& rates) const;

  void MaybeLogOrientationCovariancePolicy(
      const OASIS::IMU::BNO086::OrientationCovariancePolicyResult& covariance_policy);

  // ROS publishers
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr m_imuPublisher;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr m_imuPredictedPublisher;
  rclcpp::Publisher<oasis_msgs::msg::ImuVr>::SharedPtr m_imuVrPublisher;
  rclcpp::Publisher<geometry_msgs::msg::AccelWithCovarianceStamped>::SharedPtr m_gravityPublisher;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr m_imuGravityPublisher;

  // BNO086 subsystems
  std::unique_ptr<OASIS::IMU::BNO086::Bno086Transport> m_transport;
  std::unique_ptr<OASIS::IMU::BNO086::Bno086Shtp> m_shtp;
  std::unique_ptr<OASIS::IMU::BNO086::Bno086Gpio> m_interruptGpio;
  std::unique_ptr<OASIS::IMU::BNO086::Bno086TimestampCadence> m_timestampCadence;

  // Threading parameters
  std::atomic<bool> m_running{false};
  std::thread m_interruptThread;

  // BNO086 state
  Bno086DriverConfig m_config;
  Bno086ReportConfig m_reports;
  Bno086DrainConfig m_drain_config;
  Bno086StreamState m_stream;
  Bno086DiagnosticsState m_diag;
  Bno086StartupLogState m_startup;
  Bno086CovarianceLogState m_covariance_log;
};
} // namespace OASIS::ROS
