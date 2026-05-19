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
#include "imu/bno086/core/Bno086Sh2Timestamp.hpp"
#include "imu/bno086/diagnostics/Bno086DrainHealth.hpp"
#include "imu/bno086/diagnostics/Bno086RateHealth.hpp"
#include "imu/bno086/gpio/Bno086Gpio.hpp"
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
class Bno086Shtp;
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
  struct SampleTiming
  {
    bool has_timebase_reference{false};
    bool used_missing_timebase_fallback{false};
    std::int32_t timebase_delta_ticks{0};
    std::uint16_t delay_ticks{0};
    int64_t packet_host_anchor_ns{0};
  };

  struct SampleState
  {
    bool has_sample{false};
    rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
    std::uint8_t sequence{0};
    std::uint8_t accuracy{0};
    SampleTiming timing{};
  };

  struct EventStamp
  {
    rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
    SampleTiming timing{};
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
    std::optional<CoreFrameSignature> last_published_core_signature;
    OASIS::IMU::BNO086::Bno086OutputStampGate imu_stamp_gate;
    OASIS::IMU::BNO086::Bno086OutputStampGate imu_gravity_stamp_gate;
    OASIS::IMU::BNO086::Bno086OutputStampGate gravity_stamp_gate;
  };

  enum class ImuGravityGateReason
  {
    MissingOrientation,
    MissingGyro,
    MissingAccel,
    InvalidSample,
    NonMonotonicStamp,
    PublisherNotReady,
  };

  struct ImuGravityGateCounters
  {
    std::uint64_t missing_orientation{0};
    std::uint64_t missing_gyro{0};
    std::uint64_t missing_accel{0};
    std::uint64_t invalid_sample{0};
    std::uint64_t non_monotonic_stamp{0};
    std::uint64_t publisher_not_ready{0};
    std::uint64_t published{0};
  };

  struct Bno086TimingDiagnosticCounters
  {
    std::uint64_t duplicate_stamp{0};
    std::uint64_t backward_stamp{0};
    std::uint64_t missing_timebase_fallback{0};
    std::uint64_t sequence_gap{0};
    std::uint64_t sequence_gap_delta2{0};
    std::uint64_t sequence_gap_delta3plus{0};
    std::uint64_t sequence_gap_delta10plus{0};
  };

  struct Bno086DiagnosticsState
  {
    OASIS::IMU::BNO086::Bno086RateHealth rate_health;
    OASIS::IMU::BNO086::Bno086DrainHealth drain_health;
    ImuGravityGateCounters imu_gravity_gate_counters;
    ImuGravityGateCounters last_logged_imu_gravity_gate_counters;
    bool was_unhealthy{false};
    bool was_imu_gravity_unhealthy{false};
    std::uint32_t repeated_no_progress_timeouts{0};
    bool warned_missing_imu_fields{false};
    OASIS::IMU::BNO086::Bno086ReportSequenceDiagnostics sequence_diagnostics;
    std::uint64_t duplicate_sequence_count{0};
    std::uint64_t sequence_gap_count{0};
    Bno086TimingDiagnosticCounters timing_counters;
    Bno086TimingDiagnosticCounters last_logged_timing_counters;
  };

  struct Bno086GpioTimestampLogState
  {
    std::uint32_t logged_samples{0};
    bool warned_bad_basis{false};
    bool use_gpio_event_timestamps{true};
    std::optional<double> last_difference_ms;
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
  OASIS::IMU::BNO086::Bno086ImuGravityAccelSample LatestImuGravityAccelSample() const;
  CoreFrameSignature LatestCoreSignature() const;
  rclcpp::Time LatestCoreStamp() const;
  EventStamp ComputeEventStamp(const OASIS::IMU::BNO086::SensorEvent& event,
                               const rclcpp::Time& packet_host_anchor);
  void RecordSequenceDiagnostics(const OASIS::IMU::BNO086::SensorEvent& event);
  void MaybeLogGpioTimestampBasis(
      const OASIS::IMU::BNO086::Bno086Gpio::AssertedLowTimestamp& asserted_timestamp,
      const std::chrono::steady_clock::time_point& now_steady);
  void LogStampDrop(const char* topic,
                    const OASIS::IMU::BNO086::SensorEvent& event,
                    const OASIS::IMU::BNO086::Bno086OutputStampGateResult& gate_result,
                    const SampleTiming& timing);

  void ApplyEvent(const OASIS::IMU::BNO086::SensorEvent& event, const EventStamp& event_stamp);
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
  void RecordImuGravityGateSkip(ImuGravityGateReason reason);
  void RecordImuGravityGatePublished();
  ImuGravityGateCounters ImuGravityGateDelta() const;
  Bno086TimingDiagnosticCounters TimingDiagnosticDelta() const;
  const char* DominantImuGravityGateReason(const ImuGravityGateCounters& counters) const;
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
  Bno086GpioTimestampLogState m_gpio_timestamp_log;
};
} // namespace OASIS::ROS
