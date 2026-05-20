/*
 *  Copyright (C) 2022-2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#pragma once

#include "MonocularSlamBase.h"

#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <builtin_interfaces/msg/time.hpp>
#include <sensor_msgs/msg/imu.hpp>

namespace ORB_SLAM3
{
namespace IMU
{
class Point;
} // namespace IMU
} // namespace ORB_SLAM3

namespace OASIS
{
namespace SLAM
{

class MonocularInertialSlam : public MonocularSlamBase
{
public:
  using InitRejectedCallback = std::function<void(const std::string&, int64_t)>;

  struct ImuBufferStatus
  {
    // True after at least one valid IMU sample has been accepted
    bool has_received_imu = false;

    // True when at least one accepted IMU sample is currently buffered
    bool has_buffered_imu = false;

    // Total IMU callbacks received by this SLAM wrapper
    std::size_t received_count = 0;

    // Total valid IMU callbacks accepted into the buffer
    std::size_t accepted_count = 0;

    // Total accepted IMU samples dropped by armed tracking overflow pruning
    std::size_t dropped_count = 0;

    // Current number of accepted IMU samples in the buffer
    std::size_t buffer_size = 0;

    // Oldest buffered IMU message stamp in nanoseconds, when buffered
    std::optional<int64_t> oldest_imu_stamp_ns;

    // Newest buffered IMU message stamp in nanoseconds, when buffered
    std::optional<int64_t> newest_imu_stamp_ns;

    // Previous finite-pose image stamp committed by tracking, in nanoseconds
    std::optional<int64_t> previous_tracked_image_stamp_ns;

    // True after ORB-SLAM3 reports inertial initialization for this map
    bool has_stable_slam_map = false;
  };

  MonocularInertialSlam(rclcpp::Node& node,
                        const std::string& pointCloudTopic,
                        const std::string& poseTopic);
  ~MonocularInertialSlam() override;

  // Lifecycle functions
  bool Initialize(const std::string& vocabularyFile, const std::string& settingsFile);
  void Deinitialize();

  // Node interface
  bool ReceiveImu(const sensor_msgs::msg::Imu& imuMsg);
  bool HasReceivedImu() const;
  ImuBufferStatus GetImuBufferStatus() const;
  std::optional<int64_t> FindContinuousImuWindowStart(int64_t requiredWindowNs,
                                                      int64_t maxGapNs) const;
  void ArmStartup(int64_t imuWindowStartNs);
  void DisarmStartup();
  void SetPreStableInitRejectedCallback(InitRejectedCallback callback);
  bool HasImuCoverageForImageStamp(int64_t imageStampNs) const;
  bool HasContinuousImuCoverageForImageStamp(int64_t imageStampNs) const;
  void NotifySensorStreamDiscontinuity(const std::string& reason,
                                       int64_t previousStampNs,
                                       int64_t currentStampNs);
  void NotifyPreStableMonocularInertialInitRetry(const std::string& reason,
                                                 int64_t previousStampNs,
                                                 int64_t currentStampNs);

protected:
  // Implementation of MonocularSlamBase
  std::optional<Eigen::Isometry3f> TrackFrame(const cv::Mat& rgbImage,
                                              int64_t timestampNs) override;
  void LogTrackingSummary(int trackingState,
                          std::size_t trackedPoints,
                          std::size_t mapPoints) override;

private:
  enum class MonoInertialInitializationStatus
  {
    UNKNOWN,
    VISUAL_CANDIDATE,
    INERTIAL_INITIALIZED,
    BAD_IMU_OR_RESET_PENDING,
    REJECTED,
  };

  struct TrackedImageImuBatch
  {
    // IMU messages in the interval selected for the tracked image
    std::vector<sensor_msgs::msg::Imu> imuMessages;

    // True after at least one tracked image timestamp has been established
    bool hasPreviousTrackedImage = false;

    // Timestamp of the previous tracked image in nanoseconds, when available
    std::optional<int64_t> previousTrackedImageStampNs;

    // IMU buffer status captured while selecting samples
    ImuBufferStatus imuStatus;
  };

  struct GyroDiagnosticSample
  {
    // Message timestamp in nanoseconds
    int64_t stamp_ns = 0;

    // Gyroscope x-axis angular rate in the message frame, rad/s
    double x_rads = 0.0;

    // Gyroscope y-axis angular rate in the message frame, rad/s
    double y_rads = 0.0;

    // Gyroscope z-axis angular rate in the message frame, rad/s
    double z_rads = 0.0;

    // Euclidean norm of the gyroscope vector, rad/s
    double norm_rads = 0.0;
  };

  struct ImuDiagnosticSample
  {
    // Message timestamp in nanoseconds
    int64_t stamp_ns = 0;

    // Gyroscope angular rates in the message frame, rad/s
    double gyro_x_rads = 0.0;
    double gyro_y_rads = 0.0;
    double gyro_z_rads = 0.0;

    // Linear acceleration in the message frame, m/s^2
    double accel_x_mps2 = 0.0;
    double accel_y_mps2 = 0.0;
    double accel_z_mps2 = 0.0;
  };

  struct ImuWindowDiagnostic
  {
    // Number of IMU samples in the window
    std::size_t sample_count = 0;

    // Window duration covered by the samples, seconds
    double duration_sec = 0.0;

    // Gyroscope RMS x/y/z in the message frame, rad/s
    double gyro_rms_x_rads = 0.0;
    double gyro_rms_y_rads = 0.0;
    double gyro_rms_z_rads = 0.0;

    // Acceleration RMS x/y/z in the message frame, m/s^2
    double accel_rms_x_mps2 = 0.0;
    double accel_rms_y_mps2 = 0.0;
    double accel_rms_z_mps2 = 0.0;

    // Acceleration mean x/y/z in the message frame, m/s^2
    double accel_mean_x_mps2 = 0.0;
    double accel_mean_y_mps2 = 0.0;
    double accel_mean_z_mps2 = 0.0;

    // Integrated gyroscope x/y/z over the window, radians
    double gyro_integral_x_rad = 0.0;
    double gyro_integral_y_rad = 0.0;
    double gyro_integral_z_rad = 0.0;

    // Maximum gyroscope vector norm in the window, rad/s
    double max_gyro_norm_rads = 0.0;

    // Maximum absolute acceleration norm deviation from gravity, m/s^2
    double max_accel_gravity_deviation_mps2 = 0.0;

    // Absolute acceleration mean norm deviation from gravity, m/s^2
    double mean_accel_gravity_deviation_mps2 = 0.0;
  };

  struct PoseAttitudeDiagnosticSample
  {
    // Pose timestamp in nanoseconds
    int64_t stamp_ns = 0;

    // Twc camera position x/y/z in the world frame, meters
    double x_m = 0.0;
    double y_m = 0.0;
    double z_m = 0.0;

    // Twc roll angle, radians
    double roll_rad = 0.0;

    // Twc pitch angle, radians
    double pitch_rad = 0.0;

    // Twc yaw angle, radians
    double yaw_rad = 0.0;
  };

  // Utility functions
  static int64_t StampToNanoseconds(const builtin_interfaces::msg::Time& stamp);
  static ORB_SLAM3::IMU::Point ToOrbImuPoint(const sensor_msgs::msg::Imu& imuMsg);
  void ResetMotionDiagnosticsLocked();
  void LogCameraImuTransform(const std::string& settingsFile);
  void LogOrbImuSanityLocked();
  void LogImuYawDiagnostic(const sensor_msgs::msg::Imu& imuMsg, double gyroNorm);
  void UpdateImuMotionDiagnostics(const sensor_msgs::msg::Imu& imuMsg);
  std::optional<ImuWindowDiagnostic> ComputeImuWindowDiagnosticLocked(int64_t newestStampNs,
                                                                      int64_t windowNs) const;
  std::optional<PoseAttitudeDiagnosticSample> ComputePoseAttitudeDeltaLocked(
      int64_t newestStampNs, int64_t windowNs) const;
  void LogPoseYawDiagnostic(ORB_SLAM3::System& slam,
                            int64_t timestampNs,
                            const Eigen::Isometry3f& pose,
                            int trackingState,
                            std::size_t trackedPoints,
                            std::size_t mapPoints);
  void LogStationaryDiagnostics(ORB_SLAM3::System& slam,
                                int64_t timestampNs,
                                const Eigen::Isometry3f& pose,
                                int trackingState,
                                std::size_t trackedPoints,
                                std::size_t mapPoints);
  void LogTrackingFailureDiagnostics(const char* failureReasonName,
                                     int64_t imageStampNs,
                                     int trackingState,
                                     std::size_t trackedPoints,
                                     std::size_t mapPoints);
  ImuBufferStatus GetImuBufferStatusLocked() const;
  std::optional<int64_t> FindContinuousImuWindowStartLocked(int64_t requiredWindowNs,
                                                            int64_t maxGapNs) const;
  void PruneUnarmedStartupImuWindowLocked(int64_t newestImuStampNs);
  std::size_t PruneArmedTrackingImuOverflowLocked();
  bool HasImuCoverageForImageStampLocked(int64_t imageStampNs) const;
  bool HasContinuousImuCoverageForImageStampLocked(int64_t imageStampNs) const;
  TrackedImageImuBatch TakeImuSamplesForTrackedImage(int64_t imageStampNs);
  void CommitTrackedImageStamp(int64_t imageStampNs);
  bool LogInitializationStatus(ORB_SLAM3::System& slam,
                               int64_t imageStampNs,
                               int trackingState,
                               std::size_t trackedPoints,
                               std::size_t mapPoints);
  void WarnAboutImuIntervalGaps(const std::vector<sensor_msgs::msg::Imu>& imuMessages,
                                int64_t previousImageStampNs,
                                double previousImageTimestamp);

  // IMU buffering
  mutable std::mutex m_imuMutex;
  std::deque<sensor_msgs::msg::Imu> m_imuBuffer;
  std::optional<int64_t> m_previousTrackedImageStampNs;
  std::optional<int64_t> m_lastAcceptedImuStampNs;
  bool m_hasReceivedImu = false;
  bool m_loggedFirstImu = false;
  std::size_t m_receivedImuMessages = 0;
  std::size_t m_acceptedImuMessages = 0;
  std::size_t m_droppedImuSamples = 0;
  std::optional<MonoInertialInitializationStatus> m_lastInitializationStatus;
  std::optional<ORB_SLAM3::TrackingFailureReason> m_lastInitializationFailureReason;
  std::optional<int> m_lastLoggedTrackingState;
  InitRejectedCallback m_preStableInitRejectedCallback;
  bool m_hasStableSlamMap = false;
  bool m_startupArmed = false;
  bool m_loggedEmptyImuMeasurementsError = false;
  bool m_curveActive = false;
  std::optional<bool> m_lastLoggedCurveActive;
  std::optional<double> m_lastPoseYawRad;
  std::optional<GyroDiagnosticSample> m_latestGyroDiagnostic;
  std::optional<ImuWindowDiagnostic> m_lastImuWindow05Sec;
  std::optional<ImuWindowDiagnostic> m_lastImuWindow1Sec;
  std::optional<PoseAttitudeDiagnosticSample> m_lastPoseDelta05Sec;
  std::optional<PoseAttitudeDiagnosticSample> m_lastPoseDelta1Sec;
  bool m_stationaryDiagnosticActive = false;
  std::optional<PoseAttitudeDiagnosticSample> m_stationaryReferencePose;
  std::optional<ORB_SLAM3::InertialStateDiagnostic> m_stationaryReferenceState;
  std::optional<Eigen::Matrix3d> m_tbcRotation;
  std::optional<Eigen::Matrix3d> m_tcbRotation;
  std::deque<ImuDiagnosticSample> m_imuDiagnosticWindow;
  std::deque<PoseAttitudeDiagnosticSample> m_poseAttitudeDiagnosticWindow;
  bool m_loggedOrbImuSanity = false;
};

} // namespace SLAM
} // namespace OASIS
