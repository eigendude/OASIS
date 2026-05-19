/*
 *  Copyright (C) 2025-2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */
#pragma once

#include "slam/MonocularInertialSlam.h"

#include <condition_variable>
#include <cstddef>
#include <deque>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

#include <image_transport/subscriber.hpp>
#include <rclcpp/callback_group.hpp>
#include <rclcpp/subscription.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>

namespace rclcpp
{
class Logger;
class Node;
} // namespace rclcpp

namespace OASIS
{
namespace SLAM
{
class MonocularInertialSlam;
} // namespace SLAM

namespace ROS
{
class MonocularInertialSlamNode
{
public:
  MonocularInertialSlamNode(rclcpp::Node& node);
  ~MonocularInertialSlamNode();

  // Lifecycle functions
  bool Initialize();
  void Deinitialize();

private:
  // ROS interface
  void OnImage(const sensor_msgs::msg::Image& imageMsg);
  void OnImu(const sensor_msgs::msg::Imu& imuMsg);
  void HandleUnarmedStartupImage(const sensor_msgs::msg::Image& imageMsg);
  void RecordUnarmedStartupImu(const SLAM::MonocularInertialSlam::ImuBufferStatus& imuStatus);
  void EnterStartupUnarmed();
  void StartFreshEpochAfterImuStall(const SLAM::MonocularInertialSlam::ImuBufferStatus& imuStatus,
                                    int64_t imageStampNs,
                                    int64_t lagNs,
                                    std::size_t pendingQueueSize);
  void RecordPostStallRecoveryImu(const SLAM::MonocularInertialSlam::ImuBufferStatus& imuStatus);
  bool IsPostStallImageTooOldLocked(int64_t imageStampNs) const;
  bool IsPostStallCooloffActiveLocked() const;
  void EnterInitRetryBackoff(const std::string& reason, int64_t rejectedImageStampNs);
  bool IsInitRetryBackoffActiveLocked() const;
  bool IsInitRetryImageTooOldLocked(int64_t imageStampNs) const;
  bool TryArmStartup();
  bool IsStartupArmed() const;
  bool HandleImageDiscontinuity(int64_t imageStampNs);
  bool HasImuCoverageForImage(const sensor_msgs::msg::Image& imageMsg) const;
  void EnqueuePendingImageLocked(const sensor_msgs::msg::Image& imageMsg);
  void HoldImageUntilImuCoverage(const sensor_msgs::msg::Image& imageMsg);
  void PrunePendingImagesLocked();
  void NotifyImageWorker();
  void StartImageWorker();
  void StopImageWorker();
  void ImageWorkerLoop();
  void TryReleasePendingImageFromWorker();
  void EnterStableInputPause(const SLAM::MonocularInertialSlam::ImuBufferStatus& imuStatus,
                             int64_t imageStampNs,
                             int64_t lagNs,
                             std::size_t pendingQueueSize);
  void ClearStableInputPause();
  bool IsStableInputPaused() const;
  void RecordReleasedImage(int64_t imageStampNs);
  void RecordImuCallbackDuration(int64_t durationNs);
  void LogPeriodicTrackingDiagnostics(int64_t newestReleasedImageStampNs);
  void LogImageAheadOfImuDiagnostics(const SLAM::MonocularInertialSlam::ImuBufferStatus& imuStatus,
                                     int64_t imageStampNs,
                                     int64_t lagNs,
                                     std::size_t pendingQueueSize) const;
  void LogStartupWaitingStatus(std::optional<int64_t> newestImageStampNs,
                               const SLAM::MonocularInertialSlam::ImuBufferStatus& imuStatus,
                               bool imuReady,
                               bool armingWindowReady,
                               int64_t lagNs,
                               const char* reason) const;
  void ResetStartupArmingCandidateLocked();

  // Construction parameters
  rclcpp::Node& m_node;

  // ROS parameters
  std::unique_ptr<rclcpp::Logger> m_logger;
  rclcpp::CallbackGroup::SharedPtr m_imageCallbackGroup;
  rclcpp::CallbackGroup::SharedPtr m_imuCallbackGroup;
  std::unique_ptr<image_transport::Subscriber> m_imgSubscriber;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr m_imuSubscriber;

  int64_t m_postStallCooloffNs = 0;
  int64_t m_initRetryBackoffNs = 0;

  // SLAM parameters
  std::unique_ptr<SLAM::MonocularInertialSlam> m_monocularInertialSlam;

  // Pending monocular-inertial images awaiting IMU stamp coverage
  mutable std::mutex m_pendingImageMutex;
  std::condition_variable m_imageWorkerCv;
  std::deque<sensor_msgs::msg::Image> m_pendingImages;
  std::optional<int64_t> m_lastImageStampNs;
  std::optional<int64_t> m_newestStartupImageStampNs;
  std::optional<int64_t> m_newestStartupImuStampNs;
  std::optional<int64_t> m_lastStartupImageStampNs;
  std::optional<int64_t> m_startupArmingCandidateStartNs;
  std::optional<int64_t> m_startupArmingBoundaryNs;
  std::optional<int64_t> m_postStallRecoveryBoundaryNs;
  std::optional<int64_t> m_postStallRecoveryBoundaryWallNs;
  std::optional<int64_t> m_lastPostStallImuStampNs;
  std::optional<int64_t> m_initRetryBoundaryNs;
  std::optional<int64_t> m_initRetryBoundaryWallNs;
  std::string m_initRetryReason;
  std::size_t m_postStallImuSampleCount = 0;
  bool m_postStallRecoveryPending = false;
  bool m_initRetryPending = false;
  std::thread m_imageWorkerThread;
  bool m_imageWorkerStop = false;
  bool m_imageWorkerWake = false;
  bool m_stableInputPaused = false;
  bool m_startupArmed = false;

  // Monocular-inertial tracking diagnostics
  std::size_t m_releasedImageCount = 0;
  std::size_t m_imuCallbackCount = 0;
  int64_t m_imuCallbackTotalNs = 0;
  int64_t m_imuCallbackMaxNs = 0;
  std::optional<int64_t> m_lastTrackingDiagnosticWallNs;
  std::optional<int64_t> m_lastTrackingDiagnosticImageStampNs;
  std::size_t m_lastTrackingDiagnosticReleasedImageCount = 0;
  std::size_t m_lastTrackingDiagnosticAcceptedImuCount = 0;
  bool m_initialTrackingDiagnosticPending = false;
};
} // namespace ROS
} // namespace OASIS
