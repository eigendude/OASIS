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
#include <mutex>
#include <optional>
#include <string>
#include <vector>

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

    // Total accepted IMU samples dropped from buffer capacity pruning
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
  void ReceiveImu(const sensor_msgs::msg::Imu& imuMsg);
  bool HasReceivedImu() const;
  ImuBufferStatus GetImuBufferStatus() const;
  bool HasImuCoverageForImageStamp(int64_t imageStampNs) const;
  bool HasContinuousImuCoverageForImageStamp(int64_t imageStampNs) const;
  void NotifySensorStreamDiscontinuity(const std::string& reason,
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

  // Utility functions
  static int64_t StampToNanoseconds(const builtin_interfaces::msg::Time& stamp);
  static ORB_SLAM3::IMU::Point ToOrbImuPoint(const sensor_msgs::msg::Imu& imuMsg);
  ImuBufferStatus GetImuBufferStatusLocked() const;
  bool HasImuCoverageForImageStampLocked(int64_t imageStampNs) const;
  bool HasContinuousImuCoverageForImageStampLocked(int64_t imageStampNs) const;
  TrackedImageImuBatch TakeImuSamplesForTrackedImage(int64_t imageStampNs);
  void CommitTrackedImageStamp(int64_t imageStampNs);
  void LogInitializationStatus(ORB_SLAM3::System& slam,
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
  std::optional<int> m_lastLoggedTrackingState;
  bool m_hasStableSlamMap = false;
};

} // namespace SLAM
} // namespace OASIS
