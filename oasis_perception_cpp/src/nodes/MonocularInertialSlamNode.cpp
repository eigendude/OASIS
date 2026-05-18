/*
 *  Copyright (C) 2025-2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "MonocularInertialSlamNode.h"

#include "slam/MonocularInertialSlam.h"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <functional>
#include <iomanip>
#include <iterator>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <string_view>
#include <utility>

#include <builtin_interfaces/msg/time.hpp>
#include <image_transport/image_transport.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/qos.hpp>
#include <rmw/qos_profiles.h>

using namespace OASIS;
using namespace ROS;

namespace
{
// Subscribed topics
constexpr std::string_view IMAGE_TOPIC = "image";
constexpr std::string_view IMU_TOPIC = "imu";

// Published topics
constexpr std::string_view MAP_IMAGE_TOPIC = "slam_map_image";
constexpr std::string_view POINT_CLOUD_TOPIC = "slam_point_cloud";
constexpr std::string_view POSE_TOPIC = "slam_pose";

// Parameters
constexpr std::string_view SYSTEM_ID_PARAMETER = "system_id";
constexpr std::string_view DEFAULT_SYSTEM_ID = "";
constexpr std::string_view IMAGE_TRANSPORT_PARAMETER = "image_transport";
constexpr std::string_view DEFAULT_IMAGE_TRANSPORT = "raw";
constexpr std::string_view VOCABULARY_FILE_PARAMETER = "vocabulary_file";
constexpr std::string_view DEFAULT_VOCABULARY_FILE = "";
constexpr std::string_view SETTINGS_FILE_PARAMETER = "settings_file";
constexpr std::string_view DEFAULT_SETTINGS_FILE = "";
constexpr std::size_t ORB_IMU_SUB_QOS_DEPTH = 512;
constexpr int64_t MAX_IMAGE_STAMP_GAP_NS = 500'000'000;
constexpr int64_t IMAGE_AHEAD_OF_IMU_WARN_NS = 500'000'000;
constexpr int64_t IMAGE_AHEAD_OF_IMU_RESET_NS = 2'000'000'000;
constexpr int64_t MAX_PENDING_IMAGE_WINDOW_NS = 1'000'000'000;
constexpr std::size_t MAX_PENDING_IMAGE_COUNT = 60;
constexpr int IMU_READINESS_THROTTLE_MS = 5'000;

// Nanoseconds; one second of recent IMU history gates startup arming
constexpr int64_t STARTUP_IMU_WINDOW_NS = 1'000'000'000;

// Nanoseconds; requested maximum IMU sample gap inside the startup window
constexpr int64_t STARTUP_MAX_IMU_GAP_NS = 100'000'000;

// Nanoseconds; maximum image-ahead-of-IMU stamp lag for startup arming
constexpr int64_t STARTUP_MAX_IMAGE_AHEAD_FOR_ARM_NS = 250'000'000;

// Nanoseconds; maximum IMU-ahead-of-image stamp lead for startup arming
constexpr int64_t STARTUP_MAX_IMU_AHEAD_FOR_ARM_NS = 1'000'000'000;

// Nanoseconds; duration that startup arming inputs must remain sane
constexpr int64_t STARTUP_ARMING_HYSTERESIS_NS = 1'000'000'000;

// Wall-time period between monocular-inertial health diagnostics
constexpr int64_t TRACKING_DIAGNOSTIC_PERIOD_NS = 15'000'000'000;

[[maybe_unused]] rclcpp::QoS BestEffortSensorQos(std::size_t depth)
{
  rclcpp::QoS qos{rclcpp::KeepLast(depth)};
  qos.best_effort();
  qos.durability_volatile();
  return qos;
}

rclcpp::QoS ReliableSensorQos(std::size_t depth)
{
  rclcpp::QoS qos{rclcpp::KeepLast(depth)};
  qos.reliable();
  qos.durability_volatile();
  return qos;
}

int64_t StampToNanoseconds(const builtin_interfaces::msg::Time& stamp)
{
  return static_cast<int64_t>(stamp.sec) * 1'000'000'000LL + static_cast<int64_t>(stamp.nanosec);
}

std::string FormatTimestamp(std::optional<int64_t> timestampNs)
{
  if (!timestampNs)
    return "none";

  std::ostringstream stream;
  stream << std::fixed << std::setprecision(3)
         << static_cast<double>(*timestampNs) / 1'000'000'000.0;
  return stream.str();
}

int64_t ImageStampNs(const sensor_msgs::msg::Image& imageMsg)
{
  return StampToNanoseconds(imageMsg.header.stamp);
}
} // namespace

MonocularInertialSlamNode::MonocularInertialSlamNode(rclcpp::Node& node)
  : m_node(node),
    m_logger(std::make_unique<rclcpp::Logger>(node.get_logger())),
    m_imgSubscriber(std::make_unique<image_transport::Subscriber>())
{
  m_node.declare_parameter<std::string>(SYSTEM_ID_PARAMETER.data(), DEFAULT_SYSTEM_ID.data());
  m_node.declare_parameter<std::string>(IMAGE_TRANSPORT_PARAMETER.data(),
                                        DEFAULT_IMAGE_TRANSPORT.data());
  m_node.declare_parameter<std::string>(VOCABULARY_FILE_PARAMETER.data(),
                                        DEFAULT_VOCABULARY_FILE.data());
  m_node.declare_parameter<std::string>(SETTINGS_FILE_PARAMETER.data(),
                                        DEFAULT_SETTINGS_FILE.data());
}

MonocularInertialSlamNode::~MonocularInertialSlamNode() = default;

bool MonocularInertialSlamNode::Initialize()
{
  std::string systemId;
  if (!m_node.get_parameter(SYSTEM_ID_PARAMETER.data(), systemId) || systemId.empty())
  {
    RCLCPP_ERROR(*m_logger, "Missing or empty system ID parameter '%s'",
                 SYSTEM_ID_PARAMETER.data());
    return false;
  }

  std::string imageTopic = systemId;
  imageTopic.push_back('_');
  imageTopic.append(IMAGE_TOPIC);

  std::string imuTopic = systemId;
  imuTopic.push_back('_');
  imuTopic.append(IMU_TOPIC);

  std::string mapImageTopic = systemId;
  mapImageTopic.push_back('_');
  mapImageTopic.append(MAP_IMAGE_TOPIC);

  std::string pointCloudTopic = systemId;
  pointCloudTopic.push_back('_');
  pointCloudTopic.append(POINT_CLOUD_TOPIC);

  std::string poseTopic = systemId;
  poseTopic.push_back('_');
  poseTopic.append(POSE_TOPIC);

  RCLCPP_INFO(*m_logger, "System ID: %s", systemId.c_str());
  RCLCPP_INFO(*m_logger, "Image topic: %s", imageTopic.c_str());
  RCLCPP_INFO(*m_logger, "IMU topic: %s", imuTopic.c_str());
  RCLCPP_INFO(*m_logger, "Map image topic: %s", mapImageTopic.c_str());
  RCLCPP_INFO(*m_logger, "Point cloud topic: %s", pointCloudTopic.c_str());
  RCLCPP_INFO(*m_logger, "Pose topic: %s", poseTopic.c_str());

  std::string imageTransport;
  if (!m_node.get_parameter(IMAGE_TRANSPORT_PARAMETER.data(), imageTransport) ||
      imageTransport.empty())
  {
    imageTransport = std::string{DEFAULT_IMAGE_TRANSPORT};
    RCLCPP_WARN(*m_logger, "Image transport parameter '%s' missing or empty, defaulting to '%s'",
                IMAGE_TRANSPORT_PARAMETER.data(), imageTransport.c_str());
  }
  RCLCPP_INFO(*m_logger, "Image transport: %s", imageTransport.c_str());

  std::string vocabularyFile;
  if (!m_node.get_parameter(VOCABULARY_FILE_PARAMETER.data(), vocabularyFile) ||
      vocabularyFile.empty())
  {
    RCLCPP_ERROR(*m_logger, "Vocabulary parameter '%s' missing or empty",
                 VOCABULARY_FILE_PARAMETER.data());
    return false;
  }
  RCLCPP_INFO(*m_logger, "ORB_SLAM3 vocabulary file: %s", vocabularyFile.c_str());

  std::string settingsFile;
  if (!m_node.get_parameter(SETTINGS_FILE_PARAMETER.data(), settingsFile) || settingsFile.empty())
  {
    RCLCPP_ERROR(*m_logger, "Settings parameter '%s' missing or empty",
                 SETTINGS_FILE_PARAMETER.data());
    return false;
  }
  RCLCPP_INFO(*m_logger, "ORB_SLAM3 settings file: %s", settingsFile.c_str());

  m_monocularInertialSlam =
      std::make_unique<SLAM::MonocularInertialSlam>(m_node, pointCloudTopic, poseTopic);
  if (!m_monocularInertialSlam->Initialize(vocabularyFile, settingsFile))
  {
    RCLCPP_ERROR(*m_logger, "Failed to initialize monocular inertial SLAM");
    m_monocularInertialSlam.reset();
    return false;
  }

  {
    std::lock_guard<std::mutex> lock(m_pendingImageMutex);
    m_pendingImages.clear();
    m_lastImageStampNs.reset();
    m_newestStartupImageStampNs.reset();
    m_newestStartupImuStampNs.reset();
    m_lastStartupImageStampNs.reset();
    m_startupArmingCandidateStartNs.reset();
    m_startupArmingBoundaryNs.reset();
    m_stableInputPaused = false;
    m_startupArmed = false;
  }

  StartImageWorker();

  m_imuSubscriber = m_node.create_subscription<sensor_msgs::msg::Imu>(
      imuTopic, ReliableSensorQos(ORB_IMU_SUB_QOS_DEPTH),
      [this](const sensor_msgs::msg::Imu::ConstSharedPtr& msg)
      {
        if (msg)
          OnImu(*msg);
      });

  *m_imgSubscriber = image_transport::create_subscription(
      &m_node, imageTopic,
      [this](const sensor_msgs::msg::Image::ConstSharedPtr& msg)
      {
        if (msg)
          OnImage(*msg);
      },
      imageTransport, rclcpp::QoS{1}.get_rmw_qos_profile());

  RCLCPP_INFO(*m_logger, "Started monocular inertial SLAM");

  return true;
}

void MonocularInertialSlamNode::Deinitialize()
{
  if (m_imgSubscriber)
    m_imgSubscriber->shutdown();

  m_imuSubscriber.reset();
  StopImageWorker();

  {
    std::lock_guard<std::mutex> lock(m_pendingImageMutex);
    m_pendingImages.clear();
    m_lastImageStampNs.reset();
    m_newestStartupImageStampNs.reset();
    m_newestStartupImuStampNs.reset();
    m_lastStartupImageStampNs.reset();
    m_startupArmingCandidateStartNs.reset();
    m_startupArmingBoundaryNs.reset();
    m_imageWorkerWake = false;
    m_stableInputPaused = false;
    m_startupArmed = false;
    m_releasedImageCount = 0;
    m_imuCallbackCount = 0;
    m_imuCallbackTotalNs = 0;
    m_imuCallbackMaxNs = 0;
    m_lastTrackingDiagnosticWallNs.reset();
    m_lastTrackingDiagnosticImageStampNs.reset();
    m_lastTrackingDiagnosticReleasedImageCount = 0;
    m_lastTrackingDiagnosticAcceptedImuCount = 0;
  }

  if (m_monocularInertialSlam)
  {
    m_monocularInertialSlam->Deinitialize();
    m_monocularInertialSlam.reset();
  }
}

void MonocularInertialSlamNode::OnImage(const sensor_msgs::msg::Image& imageMsg)
{
  if (!m_monocularInertialSlam)
    return;

  const int64_t imageStampNs = StampToNanoseconds(imageMsg.header.stamp);
  if (!IsStartupArmed())
  {
    HandleUnarmedStartupImage(imageMsg);
    return;
  }

  std::optional<int64_t> armingBoundaryNs;
  {
    std::lock_guard<std::mutex> lock(m_pendingImageMutex);
    armingBoundaryNs = m_startupArmingBoundaryNs;
  }

  if (armingBoundaryNs && imageStampNs <= *armingBoundaryNs)
    return;

  if (HandleImageDiscontinuity(imageStampNs))
  {
    HandleUnarmedStartupImage(imageMsg);
    return;
  }

  if (IsStableInputPaused() || !HasImuCoverageForImage(imageMsg))
  {
    HoldImageUntilImuCoverage(imageMsg);
    NotifyImageWorker();
    return;
  }

  {
    std::lock_guard<std::mutex> lock(m_pendingImageMutex);
    EnqueuePendingImageLocked(imageMsg);
    m_lastImageStampNs = imageStampNs;
  }

  NotifyImageWorker();
}

void MonocularInertialSlamNode::OnImu(const sensor_msgs::msg::Imu& imuMsg)
{
  if (!m_monocularInertialSlam)
    return;

  const auto callbackStart = std::chrono::steady_clock::now();
  const bool imuDiscontinuityReset = m_monocularInertialSlam->ReceiveImu(imuMsg);
  if (imuDiscontinuityReset)
    EnterStartupUnarmed();

  const SLAM::MonocularInertialSlam::ImuBufferStatus imuStatus =
      m_monocularInertialSlam->GetImuBufferStatus();
  RecordUnarmedStartupImu(imuStatus);

  if (!imuStatus.has_stable_slam_map)
    ClearStableInputPause();

  TryArmStartup();
  NotifyImageWorker();

  const auto callbackEnd = std::chrono::steady_clock::now();
  const int64_t durationNs =
      std::chrono::duration_cast<std::chrono::nanoseconds>(callbackEnd - callbackStart).count();
  RecordImuCallbackDuration(durationNs);
}

void MonocularInertialSlamNode::HandleUnarmedStartupImage(const sensor_msgs::msg::Image& imageMsg)
{
  if (!m_monocularInertialSlam)
    return;

  const int64_t imageStampNs = StampToNanoseconds(imageMsg.header.stamp);

  {
    std::lock_guard<std::mutex> lock(m_pendingImageMutex);
    if (m_lastStartupImageStampNs && imageStampNs <= *m_lastStartupImageStampNs)
      ResetStartupArmingCandidateLocked();
    else
    {
      if (m_lastStartupImageStampNs &&
          imageStampNs - *m_lastStartupImageStampNs > MAX_IMAGE_STAMP_GAP_NS)
      {
        ResetStartupArmingCandidateLocked();
      }

      m_lastStartupImageStampNs = imageStampNs;
      m_newestStartupImageStampNs = imageStampNs;
    }

    m_pendingImages.clear();
    m_lastImageStampNs.reset();
    m_stableInputPaused = false;
  }

  TryArmStartup();
  NotifyImageWorker();
}

void MonocularInertialSlamNode::RecordUnarmedStartupImu(
    const SLAM::MonocularInertialSlam::ImuBufferStatus& imuStatus)
{
  if (!imuStatus.newest_imu_stamp_ns)
    return;

  std::lock_guard<std::mutex> lock(m_pendingImageMutex);
  if (m_startupArmed)
    return;

  if (m_newestStartupImuStampNs && *imuStatus.newest_imu_stamp_ns > *m_newestStartupImuStampNs &&
      *imuStatus.newest_imu_stamp_ns - *m_newestStartupImuStampNs > STARTUP_MAX_IMU_GAP_NS)
  {
    ResetStartupArmingCandidateLocked();
  }

  if (!m_newestStartupImuStampNs || *imuStatus.newest_imu_stamp_ns > *m_newestStartupImuStampNs)
    m_newestStartupImuStampNs = imuStatus.newest_imu_stamp_ns;
}

void MonocularInertialSlamNode::EnterStartupUnarmed()
{
  if (m_monocularInertialSlam)
    m_monocularInertialSlam->DisarmStartup();

  {
    std::lock_guard<std::mutex> lock(m_pendingImageMutex);
    m_pendingImages.clear();
    m_lastImageStampNs.reset();
    m_newestStartupImageStampNs.reset();
    m_newestStartupImuStampNs.reset();
    m_lastStartupImageStampNs.reset();
    m_startupArmingCandidateStartNs.reset();
    m_startupArmingBoundaryNs.reset();
    m_stableInputPaused = false;
    m_startupArmed = false;
  }
}

bool MonocularInertialSlamNode::TryArmStartup()
{
  if (!m_monocularInertialSlam)
    return false;

  std::optional<int64_t> newestImageStampNs;
  {
    std::lock_guard<std::mutex> lock(m_pendingImageMutex);
    if (m_startupArmed)
      return true;

    newestImageStampNs = m_newestStartupImageStampNs;
  }

  const SLAM::MonocularInertialSlam::ImuBufferStatus imuStatus =
      m_monocularInertialSlam->GetImuBufferStatus();
  const std::optional<int64_t> imuWindowStartNs =
      m_monocularInertialSlam->FindContinuousImuWindowStart(STARTUP_IMU_WINDOW_NS,
                                                            STARTUP_MAX_IMU_GAP_NS);
  const int64_t lagNs = newestImageStampNs && imuStatus.newest_imu_stamp_ns
                            ? *newestImageStampNs - *imuStatus.newest_imu_stamp_ns
                            : 0;
  const bool imuReady = imuWindowStartNs.has_value();
  const bool hasImageAndImu = newestImageStampNs && imuStatus.newest_imu_stamp_ns;
  const bool imageReady =
      newestImageStampNs && imuWindowStartNs && *newestImageStampNs >= *imuWindowStartNs;
  const bool imageAheadReady = hasImageAndImu && lagNs <= STARTUP_MAX_IMAGE_AHEAD_FOR_ARM_NS;
  const bool imuAheadReady = hasImageAndImu && lagNs >= -STARTUP_MAX_IMU_AHEAD_FOR_ARM_NS;
  const bool lagReady = imageAheadReady && imuAheadReady;

  if (!imageReady || !imuStatus.has_received_imu || !imuReady || !lagReady)
  {
    {
      std::lock_guard<std::mutex> lock(m_pendingImageMutex);
      ResetStartupArmingCandidateLocked();
    }

    const char* reason = "hysteresis";
    if (!newestImageStampNs)
      reason = "image";
    else if (!imuStatus.has_received_imu || !imuStatus.newest_imu_stamp_ns)
      reason = "imu";
    else if (!imuReady)
      reason = "imu_window";
    else if (!imageReady || !imuAheadReady)
      reason = "imu_ahead";
    else if (!imageAheadReady)
      reason = "image_ahead";

    LogStartupWaitingStatus(newestImageStampNs, imuStatus, imuReady, false, lagNs, reason);
    return false;
  }

  const int64_t armingStampNs = std::min(*newestImageStampNs, *imuStatus.newest_imu_stamp_ns);
  bool armingWindowReady = false;
  {
    std::lock_guard<std::mutex> lock(m_pendingImageMutex);
    if (!m_startupArmingCandidateStartNs || armingStampNs < *m_startupArmingCandidateStartNs)
      m_startupArmingCandidateStartNs = armingStampNs;

    armingWindowReady =
        armingStampNs - *m_startupArmingCandidateStartNs >= STARTUP_ARMING_HYSTERESIS_NS;
  }

  if (!armingWindowReady)
  {
    LogStartupWaitingStatus(newestImageStampNs, imuStatus, imuReady, false, lagNs, "hysteresis");
    return false;
  }

  bool armedNow = false;
  {
    std::lock_guard<std::mutex> lock(m_pendingImageMutex);
    if (!m_startupArmed)
    {
      m_monocularInertialSlam->ArmStartup(*imuWindowStartNs);
      m_pendingImages.clear();
      m_lastImageStampNs.reset();
      m_lastStartupImageStampNs.reset();
      m_startupArmingCandidateStartNs.reset();
      m_startupArmingBoundaryNs = newestImageStampNs;
      m_stableInputPaused = false;
      m_startupArmed = true;
      armedNow = true;
    }
  }

  if (armedNow)
  {
    RCLCPP_INFO(*m_logger, "SLAM armed: img=%s imu=%s lag=%.0fms",
                FormatTimestamp(newestImageStampNs).c_str(),
                FormatTimestamp(imuStatus.newest_imu_stamp_ns).c_str(),
                static_cast<double>(lagNs) / 1.0e6);
  }

  return true;
}

bool MonocularInertialSlamNode::IsStartupArmed() const
{
  std::lock_guard<std::mutex> lock(m_pendingImageMutex);
  return m_startupArmed;
}

bool MonocularInertialSlamNode::HandleImageDiscontinuity(int64_t imageStampNs)
{
  if (!m_monocularInertialSlam)
    return false;

  std::optional<int64_t> previousImageStampNs;
  {
    std::lock_guard<std::mutex> lock(m_pendingImageMutex);
    previousImageStampNs = m_lastImageStampNs;
  }

  if (!previousImageStampNs)
  {
    std::lock_guard<std::mutex> lock(m_pendingImageMutex);
    m_lastImageStampNs = imageStampNs;
    return false;
  }

  const int64_t gapNs = imageStampNs - *previousImageStampNs;
  if (gapNs > 0 && gapNs <= MAX_IMAGE_STAMP_GAP_NS)
  {
    std::lock_guard<std::mutex> lock(m_pendingImageMutex);
    m_lastImageStampNs = imageStampNs;
    return false;
  }

  if (gapNs <= 0)
  {
    RCLCPP_WARN_THROTTLE(*m_logger, *m_node.get_clock(), IMU_READINESS_THROTTLE_MS,
                         "Image order: prev=%.3f curr=%.3f dt=%.0fms",
                         static_cast<double>(*previousImageStampNs) / 1'000'000'000.0,
                         static_cast<double>(imageStampNs) / 1'000'000'000.0,
                         static_cast<double>(gapNs) / 1.0e6);

    {
      std::lock_guard<std::mutex> lock(m_pendingImageMutex);
      m_pendingImages.clear();
      m_lastImageStampNs = imageStampNs;
      m_stableInputPaused = false;
    }

    m_monocularInertialSlam->NotifySensorStreamDiscontinuity("non-monotonic image stamp",
                                                             *previousImageStampNs, imageStampNs);
    EnterStartupUnarmed();
    return true;
  }

  RCLCPP_WARN_THROTTLE(*m_logger, *m_node.get_clock(), IMU_READINESS_THROTTLE_MS,
                       "Image gap: prev=%.3f curr=%.3f gap=%.0fms",
                       static_cast<double>(*previousImageStampNs) / 1'000'000'000.0,
                       static_cast<double>(imageStampNs) / 1'000'000'000.0,
                       static_cast<double>(gapNs) / 1.0e6);

  {
    std::lock_guard<std::mutex> lock(m_pendingImageMutex);
    m_pendingImages.clear();
    m_lastImageStampNs = imageStampNs;
    m_stableInputPaused = false;
  }

  m_monocularInertialSlam->NotifySensorStreamDiscontinuity("image stamp gap", *previousImageStampNs,
                                                           imageStampNs);
  EnterStartupUnarmed();
  return true;
}

bool MonocularInertialSlamNode::HasImuCoverageForImage(
    const sensor_msgs::msg::Image& imageMsg) const
{
  if (!m_monocularInertialSlam)
    return false;

  const int64_t imageStampNs = StampToNanoseconds(imageMsg.header.stamp);
  return m_monocularInertialSlam->HasImuCoverageForImageStamp(imageStampNs);
}

void MonocularInertialSlamNode::EnqueuePendingImageLocked(const sensor_msgs::msg::Image& imageMsg)
{
  const int64_t imageStampNs = ImageStampNs(imageMsg);
  auto insertIt = m_pendingImages.begin();
  while (insertIt != m_pendingImages.end() && ImageStampNs(*insertIt) < imageStampNs)
    ++insertIt;

  if (insertIt != m_pendingImages.end() && ImageStampNs(*insertIt) == imageStampNs)
    *insertIt = imageMsg;
  else
    m_pendingImages.insert(insertIt, imageMsg);

  PrunePendingImagesLocked();
}

void MonocularInertialSlamNode::PrunePendingImagesLocked()
{
  if (m_pendingImages.empty() || !m_monocularInertialSlam)
    return;

  const SLAM::MonocularInertialSlam::ImuBufferStatus imuStatus =
      m_monocularInertialSlam->GetImuBufferStatus();

  while (imuStatus.previous_tracked_image_stamp_ns && !m_pendingImages.empty() &&
         ImageStampNs(m_pendingImages.front()) <= *imuStatus.previous_tracked_image_stamp_ns)
  {
    const int64_t pendingImageStampNs = ImageStampNs(m_pendingImages.front());
    RCLCPP_WARN_THROTTLE(*m_logger, *m_node.get_clock(), IMU_READINESS_THROTTLE_MS,
                         "Drop stale image: img=%.3f prev=%s",
                         static_cast<double>(pendingImageStampNs) / 1'000'000'000.0,
                         FormatTimestamp(imuStatus.previous_tracked_image_stamp_ns).c_str());
    m_pendingImages.pop_front();
  }

  while (!m_pendingImages.empty())
  {
    const int64_t newestPendingStampNs = ImageStampNs(m_pendingImages.back());
    const int64_t oldestPendingStampNs = ImageStampNs(m_pendingImages.front());
    if (newestPendingStampNs - oldestPendingStampNs <= MAX_PENDING_IMAGE_WINDOW_NS)
      break;

    m_pendingImages.pop_front();
  }

  while (m_pendingImages.size() > MAX_PENDING_IMAGE_COUNT)
    m_pendingImages.pop_front();
}

void MonocularInertialSlamNode::HoldImageUntilImuCoverage(const sensor_msgs::msg::Image& imageMsg)
{
  if (!m_monocularInertialSlam)
    return;

  std::size_t pendingQueueSize = 0;

  {
    std::lock_guard<std::mutex> lock(m_pendingImageMutex);
    EnqueuePendingImageLocked(imageMsg);
    pendingQueueSize = m_pendingImages.size();
  }

  const int64_t imageStampNs = StampToNanoseconds(imageMsg.header.stamp);
  const SLAM::MonocularInertialSlam::ImuBufferStatus imuStatus =
      m_monocularInertialSlam->GetImuBufferStatus();

  if (!imuStatus.has_received_imu)
  {
    RCLCPP_WARN_THROTTLE(*m_logger, *m_node.get_clock(), IMU_READINESS_THROTTLE_MS,
                         "Waiting for first IMU");
    return;
  }

  const int64_t lagNs =
      imuStatus.newest_imu_stamp_ns ? imageStampNs - *imuStatus.newest_imu_stamp_ns : 0;

  if (imuStatus.newest_imu_stamp_ns && lagNs >= IMAGE_AHEAD_OF_IMU_WARN_NS)
  {
    const bool receptionStall =
        lagNs > IMAGE_AHEAD_OF_IMU_RESET_NS &&
        (imuStatus.previous_tracked_image_stamp_ns || !imuStatus.has_stable_slam_map);
    if (!receptionStall)
      LogImageAheadOfImuDiagnostics(imuStatus, imageStampNs, lagNs, pendingQueueSize);

    if (receptionStall)
    {
      const bool pauseStableInput = imuStatus.has_stable_slam_map;
      LogImageAheadOfImuDiagnostics(imuStatus, imageStampNs, lagNs, pendingQueueSize);

      if (pauseStableInput)
        EnterStableInputPause(imuStatus, imageStampNs, lagNs, pendingQueueSize);
      else
      {
        m_monocularInertialSlam->NotifySensorStreamDiscontinuity(
            "precision-side IMU reception starvation", *imuStatus.newest_imu_stamp_ns,
            imageStampNs);
        EnterStartupUnarmed();

        {
          std::lock_guard<std::mutex> lock(m_pendingImageMutex);
          m_pendingImages.clear();
          m_newestStartupImageStampNs = imageStampNs;
          m_lastStartupImageStampNs = imageStampNs;
          m_lastImageStampNs.reset();
          m_startupArmingCandidateStartNs.reset();
          m_startupArmingBoundaryNs.reset();
          m_stableInputPaused = false;
          pendingQueueSize = m_pendingImages.size();
        }
      }
    }
  }
}

void MonocularInertialSlamNode::NotifyImageWorker()
{
  {
    std::lock_guard<std::mutex> lock(m_pendingImageMutex);
    m_imageWorkerWake = true;
  }
  m_imageWorkerCv.notify_one();
}

void MonocularInertialSlamNode::StartImageWorker()
{
  {
    std::lock_guard<std::mutex> lock(m_pendingImageMutex);
    m_imageWorkerStop = false;
    m_imageWorkerWake = false;
  }

  m_imageWorkerThread = std::thread(&MonocularInertialSlamNode::ImageWorkerLoop, this);
}

void MonocularInertialSlamNode::StopImageWorker()
{
  {
    std::lock_guard<std::mutex> lock(m_pendingImageMutex);
    m_imageWorkerStop = true;
    m_imageWorkerWake = true;
  }
  m_imageWorkerCv.notify_all();

  if (m_imageWorkerThread.joinable())
    m_imageWorkerThread.join();
}

void MonocularInertialSlamNode::ImageWorkerLoop()
{
  while (true)
  {
    {
      std::unique_lock<std::mutex> lock(m_pendingImageMutex);
      m_imageWorkerCv.wait(lock, [this] { return m_imageWorkerStop || m_imageWorkerWake; });

      if (m_imageWorkerStop)
        break;

      m_imageWorkerWake = false;
    }

    TryReleasePendingImageFromWorker();
  }
}

void MonocularInertialSlamNode::TryReleasePendingImageFromWorker()
{
  if (!m_monocularInertialSlam)
    return;

  if (!IsStartupArmed() && !TryArmStartup())
    return;

  const bool stableInputPaused = IsStableInputPaused();
  std::optional<sensor_msgs::msg::Image> imageToRelease;
  std::size_t pendingQueueSize = 0;

  {
    std::lock_guard<std::mutex> lock(m_pendingImageMutex);
    PrunePendingImagesLocked();

    pendingQueueSize = m_pendingImages.size();
    if (m_pendingImages.empty())
      return;

    auto readyIt = m_pendingImages.end();
    for (auto it = m_pendingImages.begin(); it != m_pendingImages.end(); ++it)
    {
      const int64_t pendingImageStampNs = ImageStampNs(*it);
      const bool hasSafeCoverage =
          stableInputPaused
              ? m_monocularInertialSlam->HasContinuousImuCoverageForImageStamp(pendingImageStampNs)
              : m_monocularInertialSlam->HasImuCoverageForImageStamp(pendingImageStampNs);
      if (hasSafeCoverage)
        readyIt = it;
    }

    if (readyIt == m_pendingImages.end())
      return;

    imageToRelease = std::move(*readyIt);
    m_pendingImages.erase(m_pendingImages.begin(), std::next(readyIt));
  }

  if (imageToRelease)
  {
    const int64_t imageStampNs = StampToNanoseconds(imageToRelease->header.stamp);
    const double imageTimestamp = static_cast<double>(imageStampNs) / 1'000'000'000.0;
    const SLAM::MonocularInertialSlam::ImuBufferStatus imuStatus =
        m_monocularInertialSlam->GetImuBufferStatus();

    const bool wasStableInputPaused = IsStableInputPaused();
    const int64_t lagNs =
        imuStatus.newest_imu_stamp_ns ? imageStampNs - *imuStatus.newest_imu_stamp_ns : 0;
    if (wasStableInputPaused)
    {
      RCLCPP_WARN_THROTTLE(*m_logger, *m_node.get_clock(), IMU_READINESS_THROTTLE_MS,
                           "IMU resume: img=%.3f imu=%s lag=%.0fms q=%zu stable=%d", imageTimestamp,
                           FormatTimestamp(imuStatus.newest_imu_stamp_ns).c_str(),
                           static_cast<double>(lagNs) / 1.0e6, pendingQueueSize,
                           imuStatus.has_stable_slam_map ? 1 : 0);
      ClearStableInputPause();
    }

    RCLCPP_DEBUG(*m_logger, "Release image: img=%.3f imu=%s q=%zu", imageTimestamp,
                 FormatTimestamp(imuStatus.newest_imu_stamp_ns).c_str(), pendingQueueSize);

    RecordReleasedImage(imageStampNs);
    m_monocularInertialSlam->ReceiveImage(*imageToRelease);
  }
}

void MonocularInertialSlamNode::EnterStableInputPause(
    const SLAM::MonocularInertialSlam::ImuBufferStatus& imuStatus,
    int64_t imageStampNs,
    int64_t lagNs,
    std::size_t pendingQueueSize)
{
  bool wasPaused = false;
  {
    std::lock_guard<std::mutex> lock(m_pendingImageMutex);
    wasPaused = m_stableInputPaused;
    m_stableInputPaused = true;
  }

  if (wasPaused)
    return;

  const double imageTimestamp = static_cast<double>(imageStampNs) / 1'000'000'000.0;
  RCLCPP_WARN_THROTTLE(*m_logger, *m_node.get_clock(), IMU_READINESS_THROTTLE_MS,
                       "IMU pause: img=%.3f imu=%s lag=%.0fms q=%zu stable=%d", imageTimestamp,
                       FormatTimestamp(imuStatus.newest_imu_stamp_ns).c_str(),
                       static_cast<double>(lagNs) / 1.0e6, pendingQueueSize,
                       imuStatus.has_stable_slam_map ? 1 : 0);
}

void MonocularInertialSlamNode::ClearStableInputPause()
{
  std::lock_guard<std::mutex> lock(m_pendingImageMutex);
  m_stableInputPaused = false;
}

bool MonocularInertialSlamNode::IsStableInputPaused() const
{
  std::lock_guard<std::mutex> lock(m_pendingImageMutex);
  return m_stableInputPaused;
}

void MonocularInertialSlamNode::RecordReleasedImage(int64_t imageStampNs)
{
  {
    std::lock_guard<std::mutex> lock(m_pendingImageMutex);
    ++m_releasedImageCount;
  }

  LogPeriodicTrackingDiagnostics(imageStampNs);
}

void MonocularInertialSlamNode::RecordImuCallbackDuration(int64_t durationNs)
{
  std::size_t callbackCount = 0;
  int64_t totalNs = 0;
  int64_t maxNs = 0;
  {
    std::lock_guard<std::mutex> lock(m_pendingImageMutex);
    ++m_imuCallbackCount;
    m_imuCallbackTotalNs += durationNs;
    m_imuCallbackMaxNs = std::max(m_imuCallbackMaxNs, durationNs);
    callbackCount = m_imuCallbackCount;
    totalNs = m_imuCallbackTotalNs;
    maxNs = m_imuCallbackMaxNs;
  }

  const double averageUs = callbackCount > 0 ? static_cast<double>(totalNs) /
                                                   static_cast<double>(callbackCount) / 1'000.0
                                             : 0.0;
  RCLCPP_DEBUG_THROTTLE(*m_logger, *m_node.get_clock(), IMU_READINESS_THROTTLE_MS,
                        "IMU cb: n=%zu avg=%.1fus max=%.1fus", callbackCount, averageUs,
                        static_cast<double>(maxNs) / 1'000.0);
}

void MonocularInertialSlamNode::LogPeriodicTrackingDiagnostics(int64_t newestReleasedImageStampNs)
{
  if (!m_monocularInertialSlam)
    return;

  const int64_t nowNs = m_node.now().nanoseconds();
  std::size_t pendingQueueSize = 0;
  std::size_t releasedImageCount = 0;
  std::optional<int64_t> lastDiagnosticWallNs;
  std::optional<int64_t> lastDiagnosticImageStampNs;
  std::size_t lastDiagnosticReleasedImageCount = 0;
  std::size_t lastDiagnosticAcceptedImuCount = 0;
  bool initializeDiagnosticBaseline = false;

  {
    std::lock_guard<std::mutex> lock(m_pendingImageMutex);
    pendingQueueSize = m_pendingImages.size();
    releasedImageCount = m_releasedImageCount;
    lastDiagnosticWallNs = m_lastTrackingDiagnosticWallNs;
    lastDiagnosticImageStampNs = m_lastTrackingDiagnosticImageStampNs;
    lastDiagnosticReleasedImageCount = m_lastTrackingDiagnosticReleasedImageCount;
    lastDiagnosticAcceptedImuCount = m_lastTrackingDiagnosticAcceptedImuCount;

    if (!lastDiagnosticWallNs || nowNs - *lastDiagnosticWallNs < TRACKING_DIAGNOSTIC_PERIOD_NS)
    {
      initializeDiagnosticBaseline = !lastDiagnosticWallNs;
    }
  }

  if (!initializeDiagnosticBaseline &&
      (!lastDiagnosticWallNs || nowNs - *lastDiagnosticWallNs < TRACKING_DIAGNOSTIC_PERIOD_NS))
    return;

  if (initializeDiagnosticBaseline)
  {
    const SLAM::MonocularInertialSlam::ImuBufferStatus imuStatus =
        m_monocularInertialSlam->GetImuBufferStatus();

    std::lock_guard<std::mutex> lock(m_pendingImageMutex);
    m_lastTrackingDiagnosticWallNs = nowNs;
    m_lastTrackingDiagnosticImageStampNs = newestReleasedImageStampNs;
    m_lastTrackingDiagnosticReleasedImageCount = releasedImageCount;
    m_lastTrackingDiagnosticAcceptedImuCount = imuStatus.accepted_count;
    return;
  }

  const SLAM::MonocularInertialSlam::ImuBufferStatus imuStatus =
      m_monocularInertialSlam->GetImuBufferStatus();
  const std::size_t releasedImagesDelta = releasedImageCount - lastDiagnosticReleasedImageCount;
  const std::size_t acceptedImuDelta = imuStatus.accepted_count - lastDiagnosticAcceptedImuCount;
  const int64_t imageStampDeltaNs =
      lastDiagnosticImageStampNs ? newestReleasedImageStampNs - *lastDiagnosticImageStampNs : 0;
  const int64_t wallDeltaNs = lastDiagnosticWallNs ? nowNs - *lastDiagnosticWallNs : 0;
  const int64_t lagNs = imuStatus.newest_imu_stamp_ns
                            ? newestReleasedImageStampNs - *imuStatus.newest_imu_stamp_ns
                            : 0;
  const double imageReleaseRateHz = imageStampDeltaNs > 0
                                        ? static_cast<double>(releasedImagesDelta) /
                                              (static_cast<double>(imageStampDeltaNs) / 1.0e9)
                                        : 0.0;
  const double imuRateHz = wallDeltaNs > 0 ? static_cast<double>(acceptedImuDelta) /
                                                 (static_cast<double>(wallDeltaNs) / 1.0e9)
                                           : 0.0;

  RCLCPP_INFO(*m_logger, "SLAM health: img=%.1fHz imu=%.1fHz q=%zu lag=%.1fms", imageReleaseRateHz,
              imuRateHz, pendingQueueSize, static_cast<double>(lagNs) / 1.0e6);

  {
    std::lock_guard<std::mutex> lock(m_pendingImageMutex);
    m_lastTrackingDiagnosticWallNs = nowNs;
    m_lastTrackingDiagnosticImageStampNs = newestReleasedImageStampNs;
    m_lastTrackingDiagnosticReleasedImageCount = releasedImageCount;
    m_lastTrackingDiagnosticAcceptedImuCount = imuStatus.accepted_count;
  }
}

void MonocularInertialSlamNode::LogImageAheadOfImuDiagnostics(
    const SLAM::MonocularInertialSlam::ImuBufferStatus& imuStatus,
    int64_t imageStampNs,
    int64_t lagNs,
    std::size_t pendingQueueSize) const
{
  const double imageTimestamp = static_cast<double>(imageStampNs) / 1'000'000'000.0;
  const bool resetStall = lagNs > IMAGE_AHEAD_OF_IMU_RESET_NS;
  const int stableMap = imuStatus.has_stable_slam_map ? 1 : 0;

  if (resetStall)
  {
    RCLCPP_ERROR_THROTTLE(
        *m_logger, *m_node.get_clock(), IMU_READINESS_THROTTLE_MS,
        "IMU stall: img=%.3f imu=%s lag=%.0fms q=%zu stable=%d rx=%zu ok=%zu drop=%zu",
        imageTimestamp, FormatTimestamp(imuStatus.newest_imu_stamp_ns).c_str(),
        static_cast<double>(lagNs) / 1.0e6, pendingQueueSize, stableMap, imuStatus.received_count,
        imuStatus.accepted_count, imuStatus.dropped_count);
    return;
  }

  RCLCPP_WARN_THROTTLE(*m_logger, *m_node.get_clock(), IMU_READINESS_THROTTLE_MS,
                       "IMU lag: img=%.3f imu=%s lag=%.0fms q=%zu stable=%d", imageTimestamp,
                       FormatTimestamp(imuStatus.newest_imu_stamp_ns).c_str(),
                       static_cast<double>(lagNs) / 1.0e6, pendingQueueSize, stableMap);
}

void MonocularInertialSlamNode::LogStartupWaitingStatus(
    std::optional<int64_t> newestImageStampNs,
    const SLAM::MonocularInertialSlam::ImuBufferStatus& imuStatus,
    bool imuReady,
    bool armingWindowReady,
    int64_t lagNs,
    const char* reason) const
{
  RCLCPP_INFO_THROTTLE(*m_logger, *m_node.get_clock(), IMU_READINESS_THROTTLE_MS,
                       "Waiting for streams: image=%d imu=%d imu_ready=%d arm_window=%d "
                       "lag=%.0fms reason=%s img=%s imu=%s",
                       newestImageStampNs ? 1 : 0, imuStatus.has_received_imu ? 1 : 0,
                       imuReady ? 1 : 0, armingWindowReady ? 1 : 0,
                       static_cast<double>(lagNs) / 1.0e6, reason,
                       FormatTimestamp(newestImageStampNs).c_str(),
                       FormatTimestamp(imuStatus.newest_imu_stamp_ns).c_str());
}

void MonocularInertialSlamNode::ResetStartupArmingCandidateLocked()
{
  m_startupArmingCandidateStartNs.reset();
}
