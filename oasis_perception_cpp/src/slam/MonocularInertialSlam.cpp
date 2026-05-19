/*
 *  Copyright (C) 2022-2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "MonocularInertialSlam.h"

#include "ros/RosUtils.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <iterator>
#include <mutex>
#include <optional>
#include <sstream>
#include <string_view>
#include <utility>

#include <geometry_msgs/msg/vector3.hpp>
#include <rclcpp/logging.hpp>
#include <sophus/se3.hpp>

using namespace OASIS;
using namespace SLAM;

namespace
{
constexpr std::size_t ORB_IMU_BUFFER_MAX_SAMPLES = 1024;

// Nanoseconds; unarmed startup history kept only for arming health checks
constexpr std::int64_t STARTUP_IMU_BUFFER_WINDOW_NS = 3'000'000'000;

// Samples; hard cap for pathological startup stamps that do not advance
constexpr std::size_t STARTUP_IMU_BUFFER_MAX_SAMPLES = 4096;

constexpr std::int64_t IMU_INTERVAL_GAP_WARN_NS = 30'000'000;
constexpr std::int64_t MAX_IMU_STAMP_GAP_NS = 500'000'000;
constexpr int IMU_DIAGNOSTIC_THROTTLE_MS = 5'000;

bool IsFiniteMatrix4(const Eigen::Matrix4f& matrix)
{
  return matrix.allFinite();
}

bool IsPreStableInitRetryReason(std::string_view reason)
{
  return reason == "recently_initialized_imu_tracking_lost" || reason == "ref_kf" ||
         reason == "not_enough_motion" || reason == "not_enough_motion_translation" ||
         reason == "timestamp_jump_before_init";
}

bool IsFiniteVector3(const geometry_msgs::msg::Vector3& vector)
{
  return std::isfinite(vector.x) && std::isfinite(vector.y) && std::isfinite(vector.z);
}

std::string SummarizePoseMatrix(const Eigen::Matrix4f& matrix)
{
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(3) << "[r00=" << matrix(0, 0)
         << ", r11=" << matrix(1, 1) << ", r22=" << matrix(2, 2) << ", tx=" << matrix(0, 3)
         << ", ty=" << matrix(1, 3) << ", tz=" << matrix(2, 3) << "]";
  return stream.str();
}

std::string FormatOptionalTimestamp(std::optional<int64_t> timestampNs)
{
  if (!timestampNs)
    return "none";

  std::ostringstream stream;
  stream << std::fixed << std::setprecision(3)
         << static_cast<double>(*timestampNs) / 1'000'000'000.0;
  return stream.str();
}
} // namespace

MonocularInertialSlam::MonocularInertialSlam(rclcpp::Node& node,
                                             const std::string& pointCloudTopic,
                                             const std::string& poseTopic)
  : MonocularSlamBase(node, pointCloudTopic, poseTopic)
{
}

MonocularInertialSlam::~MonocularInertialSlam() = default;

bool MonocularInertialSlam::Initialize(const std::string& vocabularyFile,
                                       const std::string& settingsFile)
{
  if (!InitializeSystem(vocabularyFile, settingsFile, ORB_SLAM3::System::IMU_MONOCULAR))
    return false;

  {
    std::lock_guard<std::mutex> lock(m_imuMutex);
    m_imuBuffer.clear();
    m_previousTrackedImageStampNs.reset();
    m_lastAcceptedImuStampNs.reset();
    m_hasReceivedImu = false;
    m_loggedFirstImu = false;
    m_receivedImuMessages = 0;
    m_acceptedImuMessages = 0;
    m_droppedImuSamples = 0;
    m_lastInitializationStatus.reset();
    m_lastInitializationFailureReason.reset();
    m_lastLoggedTrackingState.reset();
    m_hasStableSlamMap = false;
    m_startupArmed = false;
    m_loggedEmptyImuMeasurementsError = false;
  }

  return true;
}

void MonocularInertialSlam::Deinitialize()
{
  DeinitializeSystem();

  {
    std::lock_guard<std::mutex> lock(m_imuMutex);
    m_imuBuffer.clear();
    m_previousTrackedImageStampNs.reset();
    m_lastAcceptedImuStampNs.reset();
    m_hasReceivedImu = false;
    m_loggedFirstImu = false;
    m_receivedImuMessages = 0;
    m_acceptedImuMessages = 0;
    m_droppedImuSamples = 0;
    m_lastInitializationStatus.reset();
    m_lastInitializationFailureReason.reset();
    m_lastLoggedTrackingState.reset();
    m_hasStableSlamMap = false;
    m_startupArmed = false;
    m_loggedEmptyImuMeasurementsError = false;
  }
}

bool MonocularInertialSlam::ReceiveImu(const sensor_msgs::msg::Imu& imuMsg)
{
  if (!HasSlam())
    return false;

  const int64_t imuStampNs = StampToNanoseconds(imuMsg.header.stamp);
  const geometry_msgs::msg::Vector3& angularVelocity = imuMsg.angular_velocity;
  const geometry_msgs::msg::Vector3& linearAcceleration = imuMsg.linear_acceleration;

  std::unique_lock<std::mutex> lock(m_imuMutex);
  bool resetSlamAfterUnlock = false;
  bool hadStartupArmedDiscontinuity = false;

  ++m_receivedImuMessages;

  const bool hasFiniteAcceleration = IsFiniteVector3(linearAcceleration);
  const bool hasFiniteAngularVelocity = IsFiniteVector3(angularVelocity);
  if (imuStampNs == 0 || !hasFiniteAcceleration || !hasFiniteAngularVelocity)
  {
    RCLCPP_WARN_THROTTLE(Logger(), Clock(), IMU_DIAGNOSTIC_THROTTLE_MS,
                         "Invalid IMU: stamp=%lld frame=%s accel_ok=%d gyro_ok=%d",
                         static_cast<long long>(imuStampNs), imuMsg.header.frame_id.c_str(),
                         hasFiniteAcceleration ? 1 : 0, hasFiniteAngularVelocity ? 1 : 0);
    return false;
  }

  if (m_lastAcceptedImuStampNs && imuStampNs > *m_lastAcceptedImuStampNs &&
      imuStampNs - *m_lastAcceptedImuStampNs > MAX_IMU_STAMP_GAP_NS)
  {
    const int64_t previousImuStampNs = *m_lastAcceptedImuStampNs;
    const int64_t gapNs = imuStampNs - previousImuStampNs;
    if (m_startupArmed)
    {
      RCLCPP_WARN_THROTTLE(
          Logger(), Clock(), IMU_DIAGNOSTIC_THROTTLE_MS, "IMU gap: prev=%.3f curr=%.3f gap=%.0fms",
          static_cast<double>(previousImuStampNs) / 1'000'000'000.0,
          static_cast<double>(imuStampNs) / 1'000'000'000.0, static_cast<double>(gapNs) / 1.0e6);
    }
    else
    {
      RCLCPP_DEBUG_THROTTLE(Logger(), Clock(), IMU_DIAGNOSTIC_THROTTLE_MS,
                            "Startup IMU warmup gap: prev=%.3f curr=%.3f gap=%.0fms",
                            static_cast<double>(previousImuStampNs) / 1'000'000'000.0,
                            static_cast<double>(imuStampNs) / 1'000'000'000.0,
                            static_cast<double>(gapNs) / 1.0e6);
    }

    m_imuBuffer.clear();
    m_previousTrackedImageStampNs.reset();
    m_lastAcceptedImuStampNs.reset();
    m_lastInitializationStatus.reset();
    m_lastLoggedTrackingState.reset();
    m_hasStableSlamMap = false;
    m_loggedEmptyImuMeasurementsError = false;

    if (m_startupArmed)
    {
      m_startupArmed = false;
      resetSlamAfterUnlock = true;
      hadStartupArmedDiscontinuity = true;
    }
  }

  const auto insertIt =
      std::upper_bound(m_imuBuffer.begin(), m_imuBuffer.end(), imuStampNs,
                       [](int64_t stampNs, const sensor_msgs::msg::Imu& bufferedMsg)
                       { return stampNs < StampToNanoseconds(bufferedMsg.header.stamp); });
  m_imuBuffer.insert(insertIt, imuMsg);
  m_hasReceivedImu = true;
  if (!m_lastAcceptedImuStampNs || imuStampNs > *m_lastAcceptedImuStampNs)
    m_lastAcceptedImuStampNs = imuStampNs;
  ++m_acceptedImuMessages;

  if (m_startupArmed)
  {
    const std::size_t prunedSamples = PruneArmedTrackingImuOverflowLocked();
    if (prunedSamples > 0)
    {
      m_droppedImuSamples += prunedSamples;
      RCLCPP_WARN_THROTTLE(Logger(), Clock(), IMU_DIAGNOSTIC_THROTTLE_MS,
                           "IMU buffer overflow: size=%zu max=%zu pruned=%zu total_drop=%zu",
                           m_imuBuffer.size(), ORB_IMU_BUFFER_MAX_SAMPLES, prunedSamples,
                           m_droppedImuSamples);
    }
  }
  else
  {
    PruneUnarmedStartupImuWindowLocked(*m_lastAcceptedImuStampNs);
  }

  if (!m_loggedFirstImu)
  {
    const double imuTimestamp = static_cast<double>(imuStampNs) / 1'000'000'000.0;
    const double gyroNorm =
        std::sqrt(angularVelocity.x * angularVelocity.x + angularVelocity.y * angularVelocity.y +
                  angularVelocity.z * angularVelocity.z);
    RCLCPP_INFO(Logger(), "First IMU: t=%.3f frame=%s accel_z=%.3f gyro_norm=%.3f", imuTimestamp,
                imuMsg.header.frame_id.c_str(), linearAcceleration.z, gyroNorm);
    m_loggedFirstImu = true;
  }

  RCLCPP_DEBUG_THROTTLE(Logger(), Clock(), IMU_DIAGNOSTIC_THROTTLE_MS,
                        "IMU callbacks: rx=%zu ok=%zu buf=%zu", m_receivedImuMessages,
                        m_acceptedImuMessages, m_imuBuffer.size());

  if (resetSlamAfterUnlock)
  {
    lock.unlock();
    ResetImageProcessingState();
    ResetActiveMap();
  }

  return hadStartupArmedDiscontinuity;
}

bool MonocularInertialSlam::HasReceivedImu() const
{
  std::lock_guard<std::mutex> lock(m_imuMutex);

  return m_hasReceivedImu;
}

MonocularInertialSlam::ImuBufferStatus MonocularInertialSlam::GetImuBufferStatus() const
{
  std::lock_guard<std::mutex> lock(m_imuMutex);

  return GetImuBufferStatusLocked();
}

std::optional<int64_t> MonocularInertialSlam::FindContinuousImuWindowStart(int64_t requiredWindowNs,
                                                                           int64_t maxGapNs) const
{
  std::lock_guard<std::mutex> lock(m_imuMutex);

  return FindContinuousImuWindowStartLocked(requiredWindowNs, maxGapNs);
}

void MonocularInertialSlam::ArmStartup(int64_t imuWindowStartNs)
{
  {
    std::lock_guard<std::mutex> lock(m_imuMutex);
    const auto pruneEnd =
        std::lower_bound(m_imuBuffer.begin(), m_imuBuffer.end(), imuWindowStartNs,
                         [](const sensor_msgs::msg::Imu& bufferedMsg, int64_t stampNs)
                         { return StampToNanoseconds(bufferedMsg.header.stamp) < stampNs; });
    m_imuBuffer.erase(m_imuBuffer.begin(), pruneEnd);
    m_previousTrackedImageStampNs.reset();
    m_lastInitializationStatus.reset();
    m_lastLoggedTrackingState.reset();
    m_hasStableSlamMap = false;
    m_startupArmed = true;
    m_loggedEmptyImuMeasurementsError = false;
  }

  ResetImageProcessingState();
}

void MonocularInertialSlam::DisarmStartup()
{
  {
    std::lock_guard<std::mutex> lock(m_imuMutex);
    m_previousTrackedImageStampNs.reset();
    m_lastInitializationStatus.reset();
    m_lastLoggedTrackingState.reset();
    m_hasStableSlamMap = false;
    m_startupArmed = false;
    m_loggedEmptyImuMeasurementsError = false;
  }

  ResetImageProcessingState();
}

void MonocularInertialSlam::SetPreStableInitRejectedCallback(InitRejectedCallback callback)
{
  std::lock_guard<std::mutex> lock(m_imuMutex);

  m_preStableInitRejectedCallback = std::move(callback);
}

bool MonocularInertialSlam::HasImuCoverageForImageStamp(int64_t imageStampNs) const
{
  std::lock_guard<std::mutex> lock(m_imuMutex);

  return HasImuCoverageForImageStampLocked(imageStampNs);
}

bool MonocularInertialSlam::HasContinuousImuCoverageForImageStamp(int64_t imageStampNs) const
{
  std::lock_guard<std::mutex> lock(m_imuMutex);

  return HasContinuousImuCoverageForImageStampLocked(imageStampNs);
}

void MonocularInertialSlam::NotifySensorStreamDiscontinuity(const std::string& reason,
                                                            int64_t previousStampNs,
                                                            int64_t currentStampNs)
{
  {
    std::lock_guard<std::mutex> lock(m_imuMutex);
    m_imuBuffer.clear();
    m_previousTrackedImageStampNs.reset();
    m_lastAcceptedImuStampNs.reset();
    m_lastInitializationStatus.reset();
    m_lastLoggedTrackingState.reset();
    m_hasStableSlamMap = false;
    m_startupArmed = false;
    m_loggedEmptyImuMeasurementsError = false;
  }

  const int64_t gapNs = currentStampNs - previousStampNs;
  RCLCPP_WARN_THROTTLE(Logger(), Clock(), IMU_DIAGNOSTIC_THROTTLE_MS,
                       "SLAM reset: %s prev=%.3f curr=%.3f gap=%.0fms", reason.c_str(),
                       static_cast<double>(previousStampNs) / 1'000'000'000.0,
                       static_cast<double>(currentStampNs) / 1'000'000'000.0,
                       static_cast<double>(gapNs) / 1.0e6);

  ResetImageProcessingState();
  ResetActiveMap();
}

void MonocularInertialSlam::NotifyPreStableMonocularInertialInitRetry(const std::string& reason,
                                                                      int64_t previousStampNs,
                                                                      int64_t currentStampNs)
{
  {
    std::lock_guard<std::mutex> lock(m_imuMutex);
    m_imuBuffer.clear();
    m_previousTrackedImageStampNs.reset();
    m_lastAcceptedImuStampNs.reset();
    m_lastInitializationStatus.reset();
    m_lastInitializationFailureReason.reset();
    m_lastLoggedTrackingState.reset();
    m_hasStableSlamMap = false;
    m_startupArmed = false;
    m_loggedEmptyImuMeasurementsError = false;
  }

  const int64_t gapNs = currentStampNs - previousStampNs;
  RCLCPP_WARN_THROTTLE(Logger(), Clock(), IMU_DIAGNOSTIC_THROTTLE_MS,
                       "SLAM pre-init retry reset: %s prev=%.3f curr=%.3f gap=%.0fms",
                       reason.c_str(), static_cast<double>(previousStampNs) / 1'000'000'000.0,
                       static_cast<double>(currentStampNs) / 1'000'000'000.0,
                       static_cast<double>(gapNs) / 1.0e6);

  ResetImageProcessingState();
  if (ORB_SLAM3::System* slam = GetSlam())
    slam->ResetPreStableMonocularInertialInitialization(reason);
}

std::optional<Eigen::Isometry3f> MonocularInertialSlam::TrackFrame(const cv::Mat& rgbImage,
                                                                   int64_t timestampNs)
{
  const double timestamp = static_cast<double>(timestampNs) / 1'000'000'000.0;

  ORB_SLAM3::System* slam = GetSlam();
  if (slam == nullptr)
    return std::nullopt;

  bool startupArmed = false;
  {
    std::lock_guard<std::mutex> lock(m_imuMutex);
    startupArmed = m_startupArmed;
  }

  if (!startupArmed)
  {
    RCLCPP_ERROR_THROTTLE(Logger(), Clock(), IMU_DIAGNOSTIC_THROTTLE_MS,
                          "Rejecting SLAM frame before startup arming: img=%.3f", timestamp);
    return std::nullopt;
  }

  TrackedImageImuBatch imuBatch = TakeImuSamplesForTrackedImage(timestampNs);
  const std::vector<sensor_msgs::msg::Imu>& imuMessages = imuBatch.imuMessages;

  if (imuMessages.empty())
  {
    bool shouldLog = false;
    {
      std::lock_guard<std::mutex> lock(m_imuMutex);
      shouldLog = !m_loggedEmptyImuMeasurementsError;
      m_loggedEmptyImuMeasurementsError = true;
    }

    if (shouldLog)
    {
      RCLCPP_ERROR(Logger(),
                   "Rejecting SLAM frame with empty IMU measurements: prev=%s img=%.3f "
                   "seen=%d buf=%zu",
                   FormatOptionalTimestamp(imuBatch.previousTrackedImageStampNs).c_str(), timestamp,
                   imuBatch.imuStatus.has_received_imu ? 1 : 0, imuBatch.imuStatus.buffer_size);
    }
    return std::nullopt;
  }

  {
    std::lock_guard<std::mutex> lock(m_imuMutex);
    m_loggedEmptyImuMeasurementsError = false;
  }

  std::vector<ORB_SLAM3::IMU::Point> imuMeasurements;
  imuMeasurements.reserve(imuMessages.size());
  for (const sensor_msgs::msg::Imu& imuMsg : imuMessages)
    imuMeasurements.emplace_back(ToOrbImuPoint(imuMsg));

  try
  {
    const Sophus::SE3f sophusPose = slam->TrackMonocular(rgbImage, timestamp, imuMeasurements);
    const Eigen::Matrix4f poseMatrix = sophusPose.matrix();
    const int trackingState = slam->GetTrackingState();
    const std::vector<ORB_SLAM3::MapPoint*> trackedMapPoints = slam->GetTrackedMapPoints();

    std::vector<ORB_SLAM3::MapPoint*> mapPoints;
    ORB_SLAM3::Atlas* atlas = slam->GetAtlas();
    if (atlas != nullptr)
      mapPoints = atlas->GetAllMapPoints();

    const bool preStableInitRejected = LogInitializationStatus(
        *slam, timestampNs, trackingState, trackedMapPoints.size(), mapPoints.size());
    if (preStableInitRejected)
      return std::nullopt;

    if (!IsFiniteMatrix4(poseMatrix))
    {
      RCLCPP_ERROR(Logger(),
                   "Rejecting non-finite monocular-inertial SLAM pose in TrackMonocular at "
                   "%.6f (state=%d, tracked_points=%zu, imu_samples=%zu, pose=%s)",
                   timestamp, trackingState, trackedMapPoints.size(), imuMeasurements.size(),
                   SummarizePoseMatrix(poseMatrix).c_str());
      ResetActiveMap();
      return std::nullopt;
    }

    Eigen::Isometry3f pose = Eigen::Isometry3f::Identity();
    pose.matrix() = poseMatrix;

    CommitTrackedImageStamp(timestampNs);

    return pose;
  }
  catch (const std::exception& ex)
  {
    RCLCPP_ERROR(Logger(),
                 "Recoverable monocular-inertial SLAM failure in TrackMonocular at %.6f "
                 "with %zu IMU samples: %s",
                 timestamp, imuMeasurements.size(), ex.what());
    ResetActiveMap();
    return std::nullopt;
  }
  catch (...)
  {
    RCLCPP_ERROR(Logger(),
                 "Recoverable monocular-inertial SLAM failure in TrackMonocular at %.6f "
                 "with %zu IMU samples: unknown exception",
                 timestamp, imuMeasurements.size());
    ResetActiveMap();
    return std::nullopt;
  }
}

int64_t MonocularInertialSlam::StampToNanoseconds(const builtin_interfaces::msg::Time& stamp)
{
  return static_cast<int64_t>(stamp.sec) * 1'000'000'000LL + static_cast<int64_t>(stamp.nanosec);
}

ORB_SLAM3::IMU::Point MonocularInertialSlam::ToOrbImuPoint(const sensor_msgs::msg::Imu& imuMsg)
{
  const std_msgs::msg::Header& header = imuMsg.header;
  const double timestamp = ROS::RosUtils::HeaderStampToSeconds(header);

  const geometry_msgs::msg::Vector3& angularVelocity = imuMsg.angular_velocity;
  const geometry_msgs::msg::Vector3& linearAcceleration = imuMsg.linear_acceleration;

  const cv::Point3f acc(linearAcceleration.x, linearAcceleration.y, linearAcceleration.z);
  const cv::Point3f gyr(angularVelocity.x, angularVelocity.y, angularVelocity.z);

  return ORB_SLAM3::IMU::Point(acc, gyr, timestamp);
}

MonocularInertialSlam::ImuBufferStatus MonocularInertialSlam::GetImuBufferStatusLocked() const
{
  ImuBufferStatus status;
  status.has_received_imu = m_hasReceivedImu;
  status.has_buffered_imu = !m_imuBuffer.empty();
  status.received_count = m_receivedImuMessages;
  status.accepted_count = m_acceptedImuMessages;
  status.dropped_count = m_droppedImuSamples;
  status.buffer_size = m_imuBuffer.size();
  status.previous_tracked_image_stamp_ns = m_previousTrackedImageStampNs;
  status.has_stable_slam_map = m_hasStableSlamMap;

  if (!m_imuBuffer.empty())
  {
    status.oldest_imu_stamp_ns = StampToNanoseconds(m_imuBuffer.front().header.stamp);
    status.newest_imu_stamp_ns = StampToNanoseconds(m_imuBuffer.back().header.stamp);
  }

  return status;
}

std::optional<int64_t> MonocularInertialSlam::FindContinuousImuWindowStartLocked(
    int64_t requiredWindowNs, int64_t maxGapNs) const
{
  if (requiredWindowNs <= 0 || maxGapNs <= 0 || m_imuBuffer.empty())
    return std::nullopt;

  const int64_t newestImuStampNs = StampToNanoseconds(m_imuBuffer.back().header.stamp);
  const int64_t windowStartNs = newestImuStampNs - requiredWindowNs;

  auto firstAfterWindowStart =
      std::lower_bound(m_imuBuffer.begin(), m_imuBuffer.end(), windowStartNs,
                       [](const sensor_msgs::msg::Imu& bufferedMsg, int64_t stampNs)
                       { return StampToNanoseconds(bufferedMsg.header.stamp) < stampNs; });

  int64_t previousImuStampNs = 0;
  auto intervalIt = firstAfterWindowStart;
  if (firstAfterWindowStart == m_imuBuffer.begin())
  {
    if (firstAfterWindowStart == m_imuBuffer.end())
      return std::nullopt;

    previousImuStampNs = StampToNanoseconds(firstAfterWindowStart->header.stamp);
    if (previousImuStampNs != windowStartNs)
      return std::nullopt;

    ++intervalIt;
  }
  else
  {
    previousImuStampNs = StampToNanoseconds(std::prev(firstAfterWindowStart)->header.stamp);
  }

  for (auto it = intervalIt; it != m_imuBuffer.end(); ++it)
  {
    const int64_t imuStampNs = StampToNanoseconds(it->header.stamp);
    if (imuStampNs - previousImuStampNs > maxGapNs)
      return std::nullopt;

    previousImuStampNs = imuStampNs;
  }

  if (newestImuStampNs - previousImuStampNs > maxGapNs)
    return std::nullopt;

  return windowStartNs;
}

void MonocularInertialSlam::PruneUnarmedStartupImuWindowLocked(int64_t newestImuStampNs)
{
  const int64_t oldestStartupStampNs = newestImuStampNs - STARTUP_IMU_BUFFER_WINDOW_NS;
  const auto pruneEnd =
      std::lower_bound(m_imuBuffer.begin(), m_imuBuffer.end(), oldestStartupStampNs,
                       [](const sensor_msgs::msg::Imu& bufferedMsg, int64_t stampNs)
                       { return StampToNanoseconds(bufferedMsg.header.stamp) < stampNs; });
  m_imuBuffer.erase(m_imuBuffer.begin(), pruneEnd);

  while (m_imuBuffer.size() > STARTUP_IMU_BUFFER_MAX_SAMPLES)
    m_imuBuffer.pop_front();
}

std::size_t MonocularInertialSlam::PruneArmedTrackingImuOverflowLocked()
{
  std::size_t prunedSamples = 0;
  while (m_imuBuffer.size() > ORB_IMU_BUFFER_MAX_SAMPLES)
  {
    m_imuBuffer.pop_front();
    ++prunedSamples;
  }

  return prunedSamples;
}

bool MonocularInertialSlam::HasImuCoverageForImageStampLocked(int64_t imageStampNs) const
{
  if (m_imuBuffer.empty())
    return false;

  const int64_t oldestImuStampNs = StampToNanoseconds(m_imuBuffer.front().header.stamp);
  const int64_t newestImuStampNs = StampToNanoseconds(m_imuBuffer.back().header.stamp);
  if (oldestImuStampNs > imageStampNs || newestImuStampNs < imageStampNs)
    return false;

  if (!m_previousTrackedImageStampNs)
    return true;

  const auto intervalBegin =
      std::upper_bound(m_imuBuffer.begin(), m_imuBuffer.end(), *m_previousTrackedImageStampNs,
                       [](int64_t stampNs, const sensor_msgs::msg::Imu& bufferedMsg)
                       { return stampNs < StampToNanoseconds(bufferedMsg.header.stamp); });
  const auto intervalEnd =
      std::upper_bound(m_imuBuffer.begin(), m_imuBuffer.end(), imageStampNs,
                       [](int64_t stampNs, const sensor_msgs::msg::Imu& bufferedMsg)
                       { return stampNs < StampToNanoseconds(bufferedMsg.header.stamp); });

  return intervalBegin != intervalEnd;
}

bool MonocularInertialSlam::HasContinuousImuCoverageForImageStampLocked(int64_t imageStampNs) const
{
  if (!HasImuCoverageForImageStampLocked(imageStampNs))
    return false;

  if (!m_previousTrackedImageStampNs)
    return true;

  const auto intervalBegin =
      std::upper_bound(m_imuBuffer.begin(), m_imuBuffer.end(), *m_previousTrackedImageStampNs,
                       [](int64_t stampNs, const sensor_msgs::msg::Imu& bufferedMsg)
                       { return stampNs < StampToNanoseconds(bufferedMsg.header.stamp); });
  const auto intervalEnd =
      std::upper_bound(m_imuBuffer.begin(), m_imuBuffer.end(), imageStampNs,
                       [](int64_t stampNs, const sensor_msgs::msg::Imu& bufferedMsg)
                       { return stampNs < StampToNanoseconds(bufferedMsg.header.stamp); });

  if (intervalBegin == intervalEnd)
    return false;

  int64_t previousStampNs = *m_previousTrackedImageStampNs;
  for (auto it = intervalBegin; it != intervalEnd; ++it)
  {
    const int64_t imuStampNs = StampToNanoseconds(it->header.stamp);
    if (imuStampNs - previousStampNs > IMU_INTERVAL_GAP_WARN_NS)
      return false;

    previousStampNs = imuStampNs;
  }

  return imageStampNs - previousStampNs <= IMU_INTERVAL_GAP_WARN_NS;
}

MonocularInertialSlam::TrackedImageImuBatch MonocularInertialSlam::TakeImuSamplesForTrackedImage(
    int64_t imageStampNs)
{
  const double imageTimestamp = static_cast<double>(imageStampNs) / 1'000'000'000.0;

  TrackedImageImuBatch imuBatch;

  {
    std::lock_guard<std::mutex> lock(m_imuMutex);
    imuBatch.imuStatus = GetImuBufferStatusLocked();

    if (m_imuBuffer.empty())
    {
      RCLCPP_WARN_THROTTLE(
          Logger(), Clock(), IMU_DIAGNOSTIC_THROTTLE_MS,
          "Empty IMU buffer: img=%.3f seen=%d prev=%s", imageTimestamp,
          imuBatch.imuStatus.has_received_imu ? 1 : 0,
          FormatOptionalTimestamp(imuBatch.imuStatus.previous_tracked_image_stamp_ns).c_str());
    }

    if (!m_previousTrackedImageStampNs)
    {
      const auto intervalEnd =
          std::upper_bound(m_imuBuffer.begin(), m_imuBuffer.end(), imageStampNs,
                           [](int64_t stampNs, const sensor_msgs::msg::Imu& bufferedMsg)
                           { return stampNs < StampToNanoseconds(bufferedMsg.header.stamp); });

      imuBatch.imuMessages.reserve(
          static_cast<std::size_t>(std::distance(m_imuBuffer.begin(), intervalEnd)));
      for (auto it = m_imuBuffer.begin(); it != intervalEnd; ++it)
        imuBatch.imuMessages.emplace_back(*it);

      return imuBatch;
    }

    imuBatch.hasPreviousTrackedImage = true;
    imuBatch.previousTrackedImageStampNs = m_previousTrackedImageStampNs;

    const auto intervalBegin = std::upper_bound(
        m_imuBuffer.begin(), m_imuBuffer.end(), *imuBatch.previousTrackedImageStampNs,
        [](int64_t stampNs, const sensor_msgs::msg::Imu& bufferedMsg)
        { return stampNs < StampToNanoseconds(bufferedMsg.header.stamp); });
    const auto intervalEnd =
        std::upper_bound(m_imuBuffer.begin(), m_imuBuffer.end(), imageStampNs,
                         [](int64_t stampNs, const sensor_msgs::msg::Imu& bufferedMsg)
                         { return stampNs < StampToNanoseconds(bufferedMsg.header.stamp); });

    imuBatch.imuMessages.reserve(
        static_cast<std::size_t>(std::distance(intervalBegin, intervalEnd)));
    for (auto it = intervalBegin; it != intervalEnd; ++it)
      imuBatch.imuMessages.emplace_back(*it);
  }

  if (imuBatch.imuMessages.empty())
  {
    RCLCPP_WARN_THROTTLE(
        Logger(), Clock(), IMU_DIAGNOSTIC_THROTTLE_MS, "No IMU samples: prev=%s img=%.3f",
        FormatOptionalTimestamp(imuBatch.imuStatus.previous_tracked_image_stamp_ns).c_str(),
        imageTimestamp);
    return imuBatch;
  }

  const double previousImageTimestamp =
      static_cast<double>(*imuBatch.previousTrackedImageStampNs) / 1'000'000'000.0;
  WarnAboutImuIntervalGaps(imuBatch.imuMessages, *imuBatch.previousTrackedImageStampNs,
                           previousImageTimestamp);

  return imuBatch;
}

void MonocularInertialSlam::CommitTrackedImageStamp(int64_t imageStampNs)
{
  std::lock_guard<std::mutex> lock(m_imuMutex);

  if (m_previousTrackedImageStampNs)
  {
    const auto pruneEnd =
        std::upper_bound(m_imuBuffer.begin(), m_imuBuffer.end(), *m_previousTrackedImageStampNs,
                         [](int64_t stampNs, const sensor_msgs::msg::Imu& bufferedMsg)
                         { return stampNs < StampToNanoseconds(bufferedMsg.header.stamp); });
    m_imuBuffer.erase(m_imuBuffer.begin(), pruneEnd);
  }

  m_previousTrackedImageStampNs = imageStampNs;
}

void MonocularInertialSlam::LogTrackingSummary(int trackingState,
                                               std::size_t trackedPoints,
                                               std::size_t mapPoints)
{
  std::optional<int> previousTrackingState;
  {
    std::lock_guard<std::mutex> lock(m_imuMutex);
    previousTrackingState = m_lastLoggedTrackingState;
    if (previousTrackingState && *previousTrackingState == trackingState)
      return;

    m_lastLoggedTrackingState = trackingState;
  }

  if (previousTrackingState)
  {
    RCLCPP_INFO(Logger(), "Tracking state: %d -> %d pts=%zu map=%zu", *previousTrackingState,
                trackingState, trackedPoints, mapPoints);
    return;
  }

  RCLCPP_INFO(Logger(), "Tracking state: none -> %d pts=%zu map=%zu", trackingState, trackedPoints,
              mapPoints);
}

bool MonocularInertialSlam::LogInitializationStatus(ORB_SLAM3::System& slam,
                                                    int64_t imageStampNs,
                                                    int trackingState,
                                                    std::size_t trackedPoints,
                                                    std::size_t mapPoints)
{
  constexpr int ORB_TRACKING_OK = 2;

  const double imageTimestamp = static_cast<double>(imageStampNs) / 1'000'000'000.0;
  const bool imuInitialized = slam.IsImuInitialized();
  const bool badImu = slam.HasBadImu();
  ORB_SLAM3::TrackingFailureReason failureReason = slam.GetLastTrackingFailureReason();
  const char* failureReasonName = slam.GetLastTrackingFailureReasonName();
  if (failureReason == ORB_SLAM3::TrackingFailureReason::None && badImu)
  {
    failureReason = ORB_SLAM3::TrackingFailureReason::BadImu;
    failureReasonName = "bad_imu";
  }

  const bool hasFailureReason = failureReason != ORB_SLAM3::TrackingFailureReason::None;

  MonoInertialInitializationStatus status = MonoInertialInitializationStatus::UNKNOWN;
  if (hasFailureReason)
    status = MonoInertialInitializationStatus::REJECTED;
  else if (badImu)
    status = MonoInertialInitializationStatus::BAD_IMU_OR_RESET_PENDING;
  else if (imuInitialized)
    status = MonoInertialInitializationStatus::INERTIAL_INITIALIZED;
  else if (trackingState == ORB_TRACKING_OK)
    status = MonoInertialInitializationStatus::VISUAL_CANDIDATE;

  bool isTransition = false;
  const bool preStableInitRejected = !imuInitialized &&
                                     status == MonoInertialInitializationStatus::REJECTED &&
                                     IsPreStableInitRetryReason(failureReasonName);
  InitRejectedCallback preStableInitRejectedCallback;
  {
    std::lock_guard<std::mutex> lock(m_imuMutex);
    isTransition = !m_lastInitializationStatus || *m_lastInitializationStatus != status ||
                   m_lastInitializationFailureReason != failureReason;
    m_lastInitializationStatus = status;
    m_lastInitializationFailureReason = failureReason;
    m_hasStableSlamMap = status == MonoInertialInitializationStatus::INERTIAL_INITIALIZED;
    if (preStableInitRejected)
      preStableInitRejectedCallback = m_preStableInitRejectedCallback;
  }

  if (preStableInitRejectedCallback)
  {
    preStableInitRejectedCallback(failureReasonName, imageStampNs);
  }

  if (isTransition)
  {
    switch (status)
    {
      case MonoInertialInitializationStatus::VISUAL_CANDIDATE:
        RCLCPP_INFO(Logger(), "Init candidate: t=%.3f state=%d pts=%zu map=%zu", imageTimestamp,
                    trackingState, trackedPoints, mapPoints);
        break;
      case MonoInertialInitializationStatus::INERTIAL_INITIALIZED:
        RCLCPP_INFO(Logger(), "Init accepted: t=%.3f state=%d pts=%zu map=%zu", imageTimestamp,
                    trackingState, trackedPoints, mapPoints);
        break;
      case MonoInertialInitializationStatus::BAD_IMU_OR_RESET_PENDING:
        RCLCPP_WARN(Logger(), "Init rejected: reason=%s t=%.3f state=%d pts=%zu map=%zu",
                    failureReasonName, imageTimestamp, trackingState, trackedPoints, mapPoints);
        break;
      case MonoInertialInitializationStatus::REJECTED:
        RCLCPP_WARN(Logger(), "Init rejected: reason=%s t=%.3f state=%d pts=%zu map=%zu",
                    failureReasonName, imageTimestamp, trackingState, trackedPoints, mapPoints);
        break;
      case MonoInertialInitializationStatus::UNKNOWN:
        RCLCPP_DEBUG(Logger(), "Init unknown: t=%.3f state=%d pts=%zu map=%zu imu=%d bad=%d",
                     imageTimestamp, trackingState, trackedPoints, mapPoints,
                     imuInitialized ? 1 : 0, badImu ? 1 : 0);
        break;
    }
    return preStableInitRejected;
  }

  const char* statusName = "unknown";
  switch (status)
  {
    case MonoInertialInitializationStatus::VISUAL_CANDIDATE:
      statusName = "visual_candidate";
      break;
    case MonoInertialInitializationStatus::INERTIAL_INITIALIZED:
      statusName = "inertial_initialized";
      break;
    case MonoInertialInitializationStatus::BAD_IMU_OR_RESET_PENDING:
      statusName = "bad_imu_or_reset_pending";
      break;
    case MonoInertialInitializationStatus::REJECTED:
      statusName = "rejected";
      break;
    case MonoInertialInitializationStatus::UNKNOWN:
      break;
  }

  RCLCPP_DEBUG_THROTTLE(Logger(), Clock(), IMU_DIAGNOSTIC_THROTTLE_MS,
                        "Init status: %s reason=%s t=%.3f state=%d pts=%zu map=%zu imu=%d bad=%d",
                        statusName, failureReasonName, imageTimestamp, trackingState, trackedPoints,
                        mapPoints, imuInitialized ? 1 : 0, badImu ? 1 : 0);
  return preStableInitRejected;
}

void MonocularInertialSlam::WarnAboutImuIntervalGaps(
    const std::vector<sensor_msgs::msg::Imu>& imuMessages,
    int64_t previousImageStampNs,
    double previousImageTimestamp)
{
  if (imuMessages.empty())
    return;

  int64_t previousImuStampNs = StampToNanoseconds(imuMessages.front().header.stamp);
  const int64_t initialGapNs = previousImuStampNs - previousImageStampNs;
  if (initialGapNs > IMU_INTERVAL_GAP_WARN_NS)
  {
    const double firstImuTimestamp = static_cast<double>(previousImuStampNs) / 1'000'000'000.0;
    RCLCPP_WARN_THROTTLE(Logger(), Clock(), IMU_DIAGNOSTIC_THROTTLE_MS,
                         "IMU gap: first=%.3f prev_img=%.3f gap=%.0fms", firstImuTimestamp,
                         previousImageTimestamp, static_cast<double>(initialGapNs) / 1.0e6);
  }

  for (std::size_t index = 1; index < imuMessages.size(); ++index)
  {
    const int64_t imuStampNs = StampToNanoseconds(imuMessages[index].header.stamp);
    const int64_t gapNs = imuStampNs - previousImuStampNs;
    if (gapNs > IMU_INTERVAL_GAP_WARN_NS)
    {
      const double previousImuTimestamp = static_cast<double>(previousImuStampNs) / 1'000'000'000.0;
      const double imuTimestamp = static_cast<double>(imuStampNs) / 1'000'000'000.0;
      RCLCPP_WARN_THROTTLE(Logger(), Clock(), IMU_DIAGNOSTIC_THROTTLE_MS,
                           "IMU gap: %.3f -> %.3f gap=%.0fms", previousImuTimestamp, imuTimestamp,
                           static_cast<double>(gapNs) / 1.0e6);
      break;
    }

    previousImuStampNs = imuStampNs;
  }
}
