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
#include <rclcpp/callback_group.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/subscription_options.hpp>
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
constexpr std::string_view POST_STALL_COOLOFF_MS_PARAMETER = "mono_inertial_post_stall_cooloff_ms";
constexpr int64_t DEFAULT_POST_STALL_COOLOFF_MS = 750;
constexpr std::string_view INIT_RETRY_BACKOFF_MS_PARAMETER = "mono_inertial_init_retry_backoff_ms";
constexpr int64_t DEFAULT_INIT_RETRY_BACKOFF_MS = 1'000;
constexpr std::string_view INIT_RETRY_GUARD_MS_PARAMETER = "mono_inertial_init_retry_guard_ms";
constexpr int64_t DEFAULT_INIT_RETRY_GUARD_MS = 150;
constexpr std::size_t ORB_IMU_SUB_QOS_DEPTH = 512;
constexpr int64_t MAX_IMAGE_STAMP_GAP_NS = 500'000'000;
constexpr int64_t IMAGE_AHEAD_OF_IMU_WARN_NS = 500'000'000;
constexpr int64_t IMAGE_AHEAD_OF_IMU_RESET_NS = 2'000'000'000;
constexpr int64_t MAX_PENDING_IMAGE_WINDOW_NS = 1'000'000'000;
constexpr std::size_t MAX_PENDING_IMAGE_COUNT = 60;
constexpr int IMU_READINESS_THROTTLE_MS = 5'000;
constexpr int INIT_RETRY_LOG_THROTTLE_MS = 1'000;

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

// Samples; minimum continuous IMU samples accepted after a stall boundary
constexpr std::size_t POST_STALL_MIN_IMU_SAMPLES = 10;

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

const char* LagSign(int64_t lagNs)
{
  if (lagNs > 0)
    return "image_ahead";
  if (lagNs < 0)
    return "imu_ahead";
  return "aligned";
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
  m_node.declare_parameter<int64_t>(POST_STALL_COOLOFF_MS_PARAMETER.data(),
                                    DEFAULT_POST_STALL_COOLOFF_MS);
  m_node.declare_parameter<int64_t>(INIT_RETRY_BACKOFF_MS_PARAMETER.data(),
                                    DEFAULT_INIT_RETRY_BACKOFF_MS);
  m_node.declare_parameter<int64_t>(INIT_RETRY_GUARD_MS_PARAMETER.data(),
                                    DEFAULT_INIT_RETRY_GUARD_MS);
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

  int64_t postStallCooloffMs = DEFAULT_POST_STALL_COOLOFF_MS;
  if (!m_node.get_parameter(POST_STALL_COOLOFF_MS_PARAMETER.data(), postStallCooloffMs))
    postStallCooloffMs = DEFAULT_POST_STALL_COOLOFF_MS;

  if (postStallCooloffMs < 0)
  {
    RCLCPP_WARN(*m_logger, "Post-stall cooloff '%s' was negative, clamping to 0ms",
                POST_STALL_COOLOFF_MS_PARAMETER.data());
    postStallCooloffMs = 0;
  }

  m_postStallCooloffNs = postStallCooloffMs * 1'000'000;
  RCLCPP_INFO(*m_logger, "Post-stall cooloff: %lldms", static_cast<long long>(postStallCooloffMs));

  int64_t initRetryBackoffMs = DEFAULT_INIT_RETRY_BACKOFF_MS;
  if (!m_node.get_parameter(INIT_RETRY_BACKOFF_MS_PARAMETER.data(), initRetryBackoffMs))
    initRetryBackoffMs = DEFAULT_INIT_RETRY_BACKOFF_MS;

  if (initRetryBackoffMs < 0)
  {
    RCLCPP_WARN(*m_logger, "Init retry backoff '%s' was negative, clamping to 0ms",
                INIT_RETRY_BACKOFF_MS_PARAMETER.data());
    initRetryBackoffMs = 0;
  }

  m_initRetryBackoffNs = initRetryBackoffMs * 1'000'000;
  RCLCPP_INFO(*m_logger, "Init retry backoff: %lldms", static_cast<long long>(initRetryBackoffMs));

  int64_t initRetryGuardMs = DEFAULT_INIT_RETRY_GUARD_MS;
  if (!m_node.get_parameter(INIT_RETRY_GUARD_MS_PARAMETER.data(), initRetryGuardMs))
    initRetryGuardMs = DEFAULT_INIT_RETRY_GUARD_MS;

  if (initRetryGuardMs < 0)
  {
    RCLCPP_WARN(*m_logger, "Init retry guard '%s' was negative, clamping to 0ms",
                INIT_RETRY_GUARD_MS_PARAMETER.data());
    initRetryGuardMs = 0;
  }

  m_initRetryGuardNs = initRetryGuardMs * 1'000'000;
  RCLCPP_INFO(*m_logger, "Init retry guard: %lldms", static_cast<long long>(initRetryGuardMs));

  m_monocularInertialSlam =
      std::make_unique<SLAM::MonocularInertialSlam>(m_node, pointCloudTopic, poseTopic);
  m_monocularInertialSlam->SetPreStableInitRejectedCallback(
      [this](const std::string& reason, int64_t imageStampNs)
      { EnterInitRetryBackoff(reason, imageStampNs); });
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
    m_postStallRecoveryBoundaryNs.reset();
    m_postStallRecoveryBoundaryWallNs.reset();
    m_lastPostStallImuStampNs.reset();
    m_initRetryBoundaryNs.reset();
    m_initRetryBoundaryWallNs.reset();
    m_initRetryReason.clear();
    m_postStallImuSampleCount = 0;
    m_postStallRecoveryPending = false;
    m_initRetryPending = false;
    m_stableInputPaused = false;
    m_stableInputPauseNewestImuStampNs.reset();
    m_stableInputPauseWallNs.reset();
    m_stableInputPauseReceivedImuCount = 0;
    m_stableInputPauseAcceptedImuCount = 0;
    m_stableInputPauseDroppedImuCount = 0;
    m_startupArmed = false;
    m_initialTrackingDiagnosticPending = false;
  }

  StartImageWorker();

  m_imuCallbackGroup = m_node.create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  m_imageCallbackGroup = m_node.create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::SubscriptionOptions imuOptions;
  imuOptions.callback_group = m_imuCallbackGroup;
  m_imuSubscriber = m_node.create_subscription<sensor_msgs::msg::Imu>(
      imuTopic, ReliableSensorQos(ORB_IMU_SUB_QOS_DEPTH),
      [this](const sensor_msgs::msg::Imu::ConstSharedPtr& msg)
      {
        if (msg)
          OnImu(*msg);
      },
      imuOptions);

  rclcpp::SubscriptionOptions imageOptions;
  imageOptions.callback_group = m_imageCallbackGroup;
  *m_imgSubscriber = image_transport::create_subscription(
      &m_node, imageTopic,
      [this](const sensor_msgs::msg::Image::ConstSharedPtr& msg)
      {
        if (msg)
          OnImage(*msg);
      },
      imageTransport, rclcpp::QoS{1}.get_rmw_qos_profile(), imageOptions);

  RCLCPP_INFO(*m_logger, "Started monocular inertial SLAM");

  return true;
}

void MonocularInertialSlamNode::Deinitialize()
{
  if (m_imgSubscriber)
    m_imgSubscriber->shutdown();

  m_imuSubscriber.reset();
  m_imageCallbackGroup.reset();
  m_imuCallbackGroup.reset();
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
    m_postStallRecoveryBoundaryNs.reset();
    m_postStallRecoveryBoundaryWallNs.reset();
    m_lastPostStallImuStampNs.reset();
    m_initRetryBoundaryNs.reset();
    m_initRetryBoundaryWallNs.reset();
    m_initRetryReason.clear();
    m_postStallImuSampleCount = 0;
    m_postStallRecoveryPending = false;
    m_initRetryPending = false;
    m_imageWorkerWake = false;
    m_stableInputPaused = false;
    m_stableInputPauseNewestImuStampNs.reset();
    m_stableInputPauseWallNs.reset();
    m_stableInputPauseReceivedImuCount = 0;
    m_stableInputPauseAcceptedImuCount = 0;
    m_stableInputPauseDroppedImuCount = 0;
    m_startupArmed = false;
    m_releasedImageCount = 0;
    m_imuCallbackCount = 0;
    m_imuCallbackTotalNs = 0;
    m_imuCallbackMaxNs = 0;
    m_lastTrackingDiagnosticWallNs.reset();
    m_lastTrackingDiagnosticImageStampNs.reset();
    m_lastTrackingDiagnosticReleasedImageCount = 0;
    m_lastTrackingDiagnosticAcceptedImuCount = 0;
    m_initialTrackingDiagnosticPending = false;
  }

  if (m_monocularInertialSlam)
  {
    m_monocularInertialSlam->SetPreStableInitRejectedCallback({});
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
    if (IsPostStallImageTooOldLocked(imageStampNs))
    {
      RCLCPP_WARN_THROTTLE(*m_logger, *m_node.get_clock(), IMU_READINESS_THROTTLE_MS,
                           "Drop old image after IMU recovery: img=%.3f boundary=%.3f",
                           static_cast<double>(imageStampNs) / 1'000'000'000.0,
                           static_cast<double>(*m_postStallRecoveryBoundaryNs) / 1'000'000'000.0);
      return;
    }

    if (IsInitRetryImageTooOldLocked(imageStampNs))
    {
      RCLCPP_WARN_THROTTLE(*m_logger, *m_node.get_clock(), INIT_RETRY_LOG_THROTTLE_MS,
                           "Drop old init retry image: img=%.3f boundary=%.3f",
                           static_cast<double>(imageStampNs) / 1'000'000'000.0,
                           static_cast<double>(*GetInitRetryFreshImageBoundaryLocked()) /
                               1'000'000'000.0);
      return;
    }
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
  RecordPostStallRecoveryImu(imuStatus);
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
    if (m_postStallRecoveryPending)
    {
      m_pendingImages.clear();
      m_lastImageStampNs.reset();
      m_newestStartupImageStampNs.reset();
      m_lastStartupImageStampNs.reset();
      ResetStartupArmingCandidateLocked();
      return;
    }

    if (IsPostStallImageTooOldLocked(imageStampNs))
    {
      RCLCPP_WARN_THROTTLE(*m_logger, *m_node.get_clock(), IMU_READINESS_THROTTLE_MS,
                           "Drop old image after IMU recovery: img=%.3f boundary=%.3f",
                           static_cast<double>(imageStampNs) / 1'000'000'000.0,
                           static_cast<double>(*m_postStallRecoveryBoundaryNs) / 1'000'000'000.0);
      return;
    }

    if (IsInitRetryImageTooOldLocked(imageStampNs))
    {
      RCLCPP_WARN_THROTTLE(*m_logger, *m_node.get_clock(), INIT_RETRY_LOG_THROTTLE_MS,
                           "Drop old init retry image: img=%.3f boundary=%.3f",
                           static_cast<double>(imageStampNs) / 1'000'000'000.0,
                           static_cast<double>(*GetInitRetryFreshImageBoundaryLocked()) /
                               1'000'000'000.0);
      return;
    }

    if (m_initRetryPending && IsInitRetryBackoffActiveLocked())
    {
      m_pendingImages.clear();
      m_lastImageStampNs.reset();
      m_newestStartupImageStampNs.reset();
      m_lastStartupImageStampNs.reset();
      ResetStartupArmingCandidateLocked();

      const int64_t elapsedNs =
          m_initRetryBoundaryWallNs ? m_node.now().nanoseconds() - *m_initRetryBoundaryWallNs : 0;
      const int64_t remainingNs = std::max<int64_t>(0, m_initRetryBackoffNs - elapsedNs);
      RCLCPP_WARN_THROTTLE(*m_logger, *m_node.get_clock(), INIT_RETRY_LOG_THROTTLE_MS,
                           "Init backoff: reason=%s boundary=%s guard=%lldms wait=%lldms",
                           m_initRetryReason.c_str(),
                           FormatTimestamp(m_initRetryBoundaryNs).c_str(),
                           static_cast<long long>(m_initRetryGuardNs / 1'000'000),
                           static_cast<long long>((remainingNs + 999'999) / 1'000'000));
      return;
    }

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
    m_postStallRecoveryBoundaryNs.reset();
    m_postStallRecoveryBoundaryWallNs.reset();
    m_lastPostStallImuStampNs.reset();
    m_initRetryBoundaryNs.reset();
    m_initRetryBoundaryWallNs.reset();
    m_initRetryReason.clear();
    m_postStallImuSampleCount = 0;
    m_postStallRecoveryPending = false;
    m_initRetryPending = false;
    m_stableInputPaused = false;
    m_stableInputPauseNewestImuStampNs.reset();
    m_stableInputPauseWallNs.reset();
    m_stableInputPauseReceivedImuCount = 0;
    m_stableInputPauseAcceptedImuCount = 0;
    m_stableInputPauseDroppedImuCount = 0;
    m_startupArmed = false;
    m_initialTrackingDiagnosticPending = false;
  }
}

void MonocularInertialSlamNode::StartFreshEpochAfterImuStall(
    const SLAM::MonocularInertialSlam::ImuBufferStatus& imuStatus,
    int64_t imageStampNs,
    int64_t lagNs,
    std::size_t pendingQueueSize)
{
  if (!m_monocularInertialSlam || !imuStatus.newest_imu_stamp_ns)
    return;

  m_monocularInertialSlam->NotifySensorStreamDiscontinuity(
      "precision-side IMU reception starvation", *imuStatus.newest_imu_stamp_ns, imageStampNs);

  {
    std::lock_guard<std::mutex> lock(m_pendingImageMutex);
    m_pendingImages.clear();
    m_lastImageStampNs.reset();
    m_newestStartupImageStampNs.reset();
    m_newestStartupImuStampNs.reset();
    m_lastStartupImageStampNs.reset();
    m_startupArmingCandidateStartNs.reset();
    m_startupArmingBoundaryNs.reset();
    m_postStallRecoveryBoundaryNs.reset();
    m_postStallRecoveryBoundaryWallNs.reset();
    m_lastPostStallImuStampNs.reset();
    m_initRetryBoundaryNs.reset();
    m_initRetryBoundaryWallNs.reset();
    m_initRetryReason.clear();
    m_postStallImuSampleCount = 0;
    m_postStallRecoveryPending = true;
    m_initRetryPending = false;
    m_stableInputPaused = false;
    m_stableInputPauseNewestImuStampNs.reset();
    m_stableInputPauseWallNs.reset();
    m_stableInputPauseReceivedImuCount = 0;
    m_stableInputPauseAcceptedImuCount = 0;
    m_stableInputPauseDroppedImuCount = 0;
    m_startupArmed = false;
    m_initialTrackingDiagnosticPending = false;
  }

  RCLCPP_ERROR_THROTTLE(*m_logger, *m_node.get_clock(), IMU_READINESS_THROTTLE_MS,
                        "MI imu_timing: phase=stall img=%.3f newest_imu=%s lag_ms=%.0f "
                        "sign=%s queue=%zu stable=%d rx=%zu ok=%zu drop=%zu",
                        static_cast<double>(imageStampNs) / 1'000'000'000.0,
                        FormatTimestamp(imuStatus.newest_imu_stamp_ns).c_str(),
                        static_cast<double>(lagNs) / 1.0e6, LagSign(lagNs), pendingQueueSize,
                        imuStatus.has_stable_slam_map ? 1 : 0, imuStatus.received_count,
                        imuStatus.accepted_count, imuStatus.dropped_count);
}

void MonocularInertialSlamNode::RecordPostStallRecoveryImu(
    const SLAM::MonocularInertialSlam::ImuBufferStatus& imuStatus)
{
  if (!imuStatus.newest_imu_stamp_ns)
    return;

  std::lock_guard<std::mutex> lock(m_pendingImageMutex);
  if (!m_postStallRecoveryPending && !m_postStallRecoveryBoundaryNs)
    return;

  const int64_t imuStampNs = *imuStatus.newest_imu_stamp_ns;
  if (!m_postStallRecoveryBoundaryNs)
  {
    m_postStallRecoveryBoundaryNs = imuStampNs;
    m_postStallRecoveryBoundaryWallNs = m_node.now().nanoseconds();
    m_lastPostStallImuStampNs = imuStampNs;
    m_postStallImuSampleCount = 1;
    m_postStallRecoveryPending = false;
    ResetStartupArmingCandidateLocked();
    return;
  }

  if (m_lastPostStallImuStampNs && imuStampNs <= *m_lastPostStallImuStampNs)
    return;

  if (m_lastPostStallImuStampNs && imuStampNs - *m_lastPostStallImuStampNs > STARTUP_MAX_IMU_GAP_NS)
  {
    m_postStallRecoveryBoundaryNs = imuStampNs;
    m_postStallRecoveryBoundaryWallNs = m_node.now().nanoseconds();
    m_postStallImuSampleCount = 1;
    ResetStartupArmingCandidateLocked();
  }
  else
  {
    ++m_postStallImuSampleCount;
  }

  m_lastPostStallImuStampNs = imuStampNs;
}

bool MonocularInertialSlamNode::IsPostStallImageTooOldLocked(int64_t imageStampNs) const
{
  return m_postStallRecoveryBoundaryNs && imageStampNs <= *m_postStallRecoveryBoundaryNs;
}

bool MonocularInertialSlamNode::IsPostStallCooloffActiveLocked() const
{
  return m_postStallRecoveryBoundaryWallNs &&
         m_node.now().nanoseconds() - *m_postStallRecoveryBoundaryWallNs < m_postStallCooloffNs;
}

void MonocularInertialSlamNode::EnterInitRetryBackoff(const std::string& reason,
                                                      int64_t rejectedImageStampNs)
{
  if (!m_monocularInertialSlam)
    return;

  const SLAM::MonocularInertialSlam::ImuBufferStatus imuStatus =
      m_monocularInertialSlam->GetImuBufferStatus();
  int64_t retryBoundaryNs = rejectedImageStampNs;
  retryBoundaryNs =
      std::max(retryBoundaryNs, imuStatus.newest_imu_stamp_ns.value_or(retryBoundaryNs));
  retryBoundaryNs = std::max(retryBoundaryNs,
                             imuStatus.previous_tracked_image_stamp_ns.value_or(retryBoundaryNs));

  {
    std::lock_guard<std::mutex> lock(m_pendingImageMutex);
    retryBoundaryNs = std::max(retryBoundaryNs, m_lastImageStampNs.value_or(retryBoundaryNs));
    retryBoundaryNs =
        std::max(retryBoundaryNs, m_newestStartupImageStampNs.value_or(retryBoundaryNs));
    retryBoundaryNs =
        std::max(retryBoundaryNs, m_lastStartupImageStampNs.value_or(retryBoundaryNs));

    if (m_initRetryPending && m_initRetryBoundaryNs &&
        rejectedImageStampNs <= *m_initRetryBoundaryNs)
    {
      return;
    }
  }

  m_monocularInertialSlam->NotifyPreStableMonocularInertialInitRetry(reason, rejectedImageStampNs,
                                                                     retryBoundaryNs);

  {
    std::lock_guard<std::mutex> lock(m_pendingImageMutex);
    m_pendingImages.clear();
    m_lastImageStampNs.reset();
    m_newestStartupImageStampNs.reset();
    m_newestStartupImuStampNs = imuStatus.newest_imu_stamp_ns;
    m_lastStartupImageStampNs.reset();
    ResetStartupArmingCandidateLocked();
    m_startupArmingBoundaryNs.reset();
    m_initRetryBoundaryNs = retryBoundaryNs;
    m_initRetryBoundaryWallNs = m_node.now().nanoseconds();
    m_initRetryReason = reason;
    m_initRetryPending = true;
    m_stableInputPaused = false;
    m_startupArmed = false;
    m_initialTrackingDiagnosticPending = false;
  }

  RCLCPP_WARN_THROTTLE(*m_logger, *m_node.get_clock(), INIT_RETRY_LOG_THROTTLE_MS,
                       "Init backoff: reason=%s boundary=%s guard=%lldms wait=%lldms",
                       reason.c_str(), FormatTimestamp(retryBoundaryNs).c_str(),
                       static_cast<long long>(m_initRetryGuardNs / 1'000'000),
                       static_cast<long long>(m_initRetryBackoffNs / 1'000'000));
}

bool MonocularInertialSlamNode::IsInitRetryBackoffActiveLocked() const
{
  return m_initRetryPending && m_initRetryBoundaryWallNs &&
         m_node.now().nanoseconds() - *m_initRetryBoundaryWallNs < m_initRetryBackoffNs;
}

std::optional<int64_t> MonocularInertialSlamNode::GetInitRetryFreshImageBoundaryLocked() const
{
  if (!m_initRetryPending || !m_initRetryBoundaryNs)
    return std::nullopt;

  return *m_initRetryBoundaryNs + m_initRetryGuardNs;
}

bool MonocularInertialSlamNode::IsInitRetryImageTooOldLocked(int64_t imageStampNs) const
{
  const std::optional<int64_t> freshImageBoundaryNs = GetInitRetryFreshImageBoundaryLocked();
  return freshImageBoundaryNs && imageStampNs <= *freshImageBoundaryNs;
}

bool MonocularInertialSlamNode::TryArmStartup()
{
  if (!m_monocularInertialSlam)
    return false;

  std::optional<int64_t> newestImageStampNs;
  std::optional<int64_t> recoveryBoundaryNs;
  std::optional<int64_t> initRetryBoundaryNs;
  std::optional<int64_t> initRetryFreshImageBoundaryNs;
  std::string initRetryReason;
  bool recoveryPending = false;
  bool recoveryCooloffActive = false;
  bool initRetryPending = false;
  bool initRetryBackoffActive = false;
  std::size_t recoveryImuSampleCount = 0;
  std::size_t pendingQueueSize = 0;
  {
    std::lock_guard<std::mutex> lock(m_pendingImageMutex);
    if (m_startupArmed)
      return true;

    newestImageStampNs = m_newestStartupImageStampNs;
    recoveryBoundaryNs = m_postStallRecoveryBoundaryNs;
    recoveryPending = m_postStallRecoveryPending;
    recoveryCooloffActive = IsPostStallCooloffActiveLocked();
    recoveryImuSampleCount = m_postStallImuSampleCount;
    initRetryBoundaryNs = m_initRetryBoundaryNs;
    initRetryFreshImageBoundaryNs = GetInitRetryFreshImageBoundaryLocked();
    initRetryReason = m_initRetryReason;
    initRetryPending = m_initRetryPending;
    initRetryBackoffActive = IsInitRetryBackoffActiveLocked();
    pendingQueueSize = m_pendingImages.size();
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
  const bool freshEpochRecovery = recoveryPending || recoveryBoundaryNs.has_value();
  const bool recoveryBoundaryReady = !recoveryPending && recoveryBoundaryNs.has_value();
  const bool recoveryImageReady =
      !freshEpochRecovery ||
      (newestImageStampNs && recoveryBoundaryNs && *newestImageStampNs > *recoveryBoundaryNs);
  const bool recoveryImuSamplesReady =
      !freshEpochRecovery || recoveryImuSampleCount >= POST_STALL_MIN_IMU_SAMPLES;
  const bool recoveryWindowReady =
      !freshEpochRecovery ||
      (recoveryBoundaryNs && imuWindowStartNs && *imuWindowStartNs >= *recoveryBoundaryNs);
  const bool recoveryReady =
      !freshEpochRecovery || (recoveryBoundaryReady && !recoveryCooloffActive &&
                              recoveryImageReady && recoveryImuSamplesReady && recoveryWindowReady);
  const bool initRetryImageReady =
      !initRetryPending || (newestImageStampNs && initRetryFreshImageBoundaryNs &&
                            *newestImageStampNs > *initRetryFreshImageBoundaryNs);
  const bool initRetryWindowReady =
      !initRetryPending ||
      (initRetryBoundaryNs && imuWindowStartNs && *imuWindowStartNs >= *initRetryBoundaryNs);
  const bool initRetryReady =
      !initRetryPending || (!initRetryBackoffActive && initRetryImageReady && initRetryWindowReady);

  if (!imageReady || !imuStatus.has_received_imu || !imuReady || !lagReady || !recoveryReady ||
      !initRetryReady)
  {
    {
      std::lock_guard<std::mutex> lock(m_pendingImageMutex);
      ResetStartupArmingCandidateLocked();
    }

    const char* reason = "hysteresis";
    if (initRetryBackoffActive)
      reason = "init_backoff";
    else if (initRetryPending && !initRetryImageReady)
      reason = "init_backoff_image";
    else if (initRetryPending && !initRetryWindowReady)
      reason = "init_backoff_window";
    else if (!newestImageStampNs)
      reason = "image";
    else if (!imuStatus.has_received_imu || !imuStatus.newest_imu_stamp_ns)
      reason = "imu";
    else if (!imuReady)
      reason = "imu_window";
    else if (!imageReady || !imuAheadReady)
      reason = "imu_ahead";
    else if (!imageAheadReady)
      reason = "image_ahead";
    else if (!recoveryBoundaryReady)
      reason = "fresh_epoch_imu";
    else if (recoveryCooloffActive)
      reason = "fresh_epoch_cooloff";
    else if (!recoveryImuSamplesReady)
      reason = "fresh_epoch_samples";
    else if (!recoveryImageReady)
      reason = "fresh_epoch_image";
    else if (!recoveryWindowReady)
      reason = "fresh_epoch_window";

    LogStartupWaitingStatus(newestImageStampNs, imuStatus, imuReady, false, lagNs, pendingQueueSize,
                            reason);
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
    LogStartupWaitingStatus(newestImageStampNs, imuStatus, imuReady, false, lagNs, pendingQueueSize,
                            "hysteresis");
    return false;
  }

  bool armedNow = false;
  bool initRetryArmed = false;
  const int64_t armingWallNs = m_node.now().nanoseconds();
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
      m_postStallRecoveryBoundaryNs.reset();
      m_postStallRecoveryBoundaryWallNs.reset();
      m_lastPostStallImuStampNs.reset();
      initRetryArmed = m_initRetryPending;
      m_initRetryBoundaryNs.reset();
      m_initRetryBoundaryWallNs.reset();
      m_initRetryReason.clear();
      m_postStallImuSampleCount = 0;
      m_postStallRecoveryPending = false;
      m_initRetryPending = false;
      m_stableInputPaused = false;
      m_startupArmed = true;
      m_lastTrackingDiagnosticWallNs = armingWallNs;
      m_lastTrackingDiagnosticImageStampNs = newestImageStampNs;
      m_lastTrackingDiagnosticReleasedImageCount = m_releasedImageCount;
      m_lastTrackingDiagnosticAcceptedImuCount = imuStatus.accepted_count;
      m_initialTrackingDiagnosticPending = true;
      armedNow = true;
    }
  }

  if (armedNow)
  {
    if (initRetryArmed)
    {
      RCLCPP_INFO(*m_logger,
                  "SLAM armed: init_retry=1 reason=%s img=%s boundary=%s imu=%s lag=%.0fms",
                  initRetryReason.c_str(), FormatTimestamp(newestImageStampNs).c_str(),
                  FormatTimestamp(initRetryBoundaryNs).c_str(),
                  FormatTimestamp(imuStatus.newest_imu_stamp_ns).c_str(),
                  static_cast<double>(lagNs) / 1.0e6);
    }
    else if (freshEpochRecovery)
    {
      RCLCPP_INFO(*m_logger, "SLAM armed: fresh_epoch=1 img=%s imu=%s lag=%.0fms",
                  FormatTimestamp(newestImageStampNs).c_str(),
                  FormatTimestamp(imuStatus.newest_imu_stamp_ns).c_str(),
                  static_cast<double>(lagNs) / 1.0e6);
    }
    else
    {
      RCLCPP_INFO(*m_logger, "SLAM armed: img=%s imu=%s lag=%.0fms",
                  FormatTimestamp(newestImageStampNs).c_str(),
                  FormatTimestamp(imuStatus.newest_imu_stamp_ns).c_str(),
                  static_cast<double>(lagNs) / 1.0e6);
    }
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

  while (m_postStallRecoveryBoundaryNs && !m_pendingImages.empty() &&
         ImageStampNs(m_pendingImages.front()) <= *m_postStallRecoveryBoundaryNs)
  {
    const int64_t pendingImageStampNs = ImageStampNs(m_pendingImages.front());
    RCLCPP_WARN_THROTTLE(*m_logger, *m_node.get_clock(), IMU_READINESS_THROTTLE_MS,
                         "Drop old image after IMU recovery: img=%.3f boundary=%.3f",
                         static_cast<double>(pendingImageStampNs) / 1'000'000'000.0,
                         static_cast<double>(*m_postStallRecoveryBoundaryNs) / 1'000'000'000.0);
    m_pendingImages.pop_front();
  }

  const std::optional<int64_t> initRetryFreshImageBoundaryNs =
      GetInitRetryFreshImageBoundaryLocked();
  while (initRetryFreshImageBoundaryNs && !m_pendingImages.empty() &&
         ImageStampNs(m_pendingImages.front()) <= *initRetryFreshImageBoundaryNs)
  {
    const int64_t pendingImageStampNs = ImageStampNs(m_pendingImages.front());
    RCLCPP_WARN_THROTTLE(*m_logger, *m_node.get_clock(), INIT_RETRY_LOG_THROTTLE_MS,
                         "Drop old init retry image: img=%.3f boundary=%.3f",
                         static_cast<double>(pendingImageStampNs) / 1'000'000'000.0,
                         static_cast<double>(*initRetryFreshImageBoundaryNs) / 1'000'000'000.0);
    m_pendingImages.pop_front();
  }

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
    if (!receptionStall && imuStatus.has_stable_slam_map)
      EnterStableInputPause(imuStatus, imageStampNs, lagNs, pendingQueueSize);
    else if (!receptionStall)
      LogImageAheadOfImuDiagnostics(imuStatus, imageStampNs, lagNs, pendingQueueSize);

    if (receptionStall)
    {
      StartFreshEpochAfterImuStall(imuStatus, imageStampNs, lagNs, pendingQueueSize);
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
      std::optional<int64_t> pauseNewestImuStampNs;
      std::optional<int64_t> pauseWallNs;
      std::size_t pauseReceivedImuCount = 0;
      std::size_t pauseAcceptedImuCount = 0;
      std::size_t pauseDroppedImuCount = 0;
      {
        std::lock_guard<std::mutex> lock(m_pendingImageMutex);
        pauseNewestImuStampNs = m_stableInputPauseNewestImuStampNs;
        pauseWallNs = m_stableInputPauseWallNs;
        pauseReceivedImuCount = m_stableInputPauseReceivedImuCount;
        pauseAcceptedImuCount = m_stableInputPauseAcceptedImuCount;
        pauseDroppedImuCount = m_stableInputPauseDroppedImuCount;
      }

      const std::size_t receivedDelta = imuStatus.received_count >= pauseReceivedImuCount
                                            ? imuStatus.received_count - pauseReceivedImuCount
                                            : 0;
      const std::size_t acceptedDelta = imuStatus.accepted_count >= pauseAcceptedImuCount
                                            ? imuStatus.accepted_count - pauseAcceptedImuCount
                                            : 0;
      const std::size_t droppedDelta = imuStatus.dropped_count >= pauseDroppedImuCount
                                           ? imuStatus.dropped_count - pauseDroppedImuCount
                                           : 0;
      const std::optional<int64_t> imuAdvanceNs =
          imuStatus.newest_imu_stamp_ns && pauseNewestImuStampNs
              ? std::make_optional(*imuStatus.newest_imu_stamp_ns - *pauseNewestImuStampNs)
              : std::nullopt;
      const std::optional<int64_t> pauseWallElapsedNs =
          pauseWallNs ? std::make_optional(m_node.now().nanoseconds() - *pauseWallNs)
                      : std::nullopt;

      RCLCPP_WARN_THROTTLE(
          *m_logger, *m_node.get_clock(), IMU_READINESS_THROTTLE_MS,
          "MI imu_timing: phase=resume img=%.3f newest_imu=%s lag_ms=%.0f sign=%s "
          "queue=%zu stable=%d rx=%zu ok=%zu drop=%zu rx_delta=%zu ok_delta=%zu "
          "drop_delta=%zu imu_advance_ms=%.0f wall_ms=%.0f",
          imageTimestamp, FormatTimestamp(imuStatus.newest_imu_stamp_ns).c_str(),
          static_cast<double>(lagNs) / 1.0e6, LagSign(lagNs), pendingQueueSize,
          imuStatus.has_stable_slam_map ? 1 : 0, imuStatus.received_count, imuStatus.accepted_count,
          imuStatus.dropped_count, receivedDelta, acceptedDelta, droppedDelta,
          imuAdvanceNs ? static_cast<double>(*imuAdvanceNs) / 1.0e6 : 0.0,
          pauseWallElapsedNs ? static_cast<double>(*pauseWallElapsedNs) / 1.0e6 : 0.0);
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
    if (!wasPaused)
    {
      m_stableInputPauseNewestImuStampNs = imuStatus.newest_imu_stamp_ns;
      m_stableInputPauseWallNs = m_node.now().nanoseconds();
      m_stableInputPauseReceivedImuCount = imuStatus.received_count;
      m_stableInputPauseAcceptedImuCount = imuStatus.accepted_count;
      m_stableInputPauseDroppedImuCount = imuStatus.dropped_count;
    }
  }

  if (wasPaused)
    return;

  const double imageTimestamp = static_cast<double>(imageStampNs) / 1'000'000'000.0;
  RCLCPP_WARN_THROTTLE(*m_logger, *m_node.get_clock(), IMU_READINESS_THROTTLE_MS,
                       "MI imu_timing: phase=runtime img=%.3f newest_imu=%s lag_ms=%.0f "
                       "sign=%s queue=%zu stable=%d rx=%zu ok=%zu drop=%zu",
                       imageTimestamp, FormatTimestamp(imuStatus.newest_imu_stamp_ns).c_str(),
                       static_cast<double>(lagNs) / 1.0e6, LagSign(lagNs), pendingQueueSize,
                       imuStatus.has_stable_slam_map ? 1 : 0, imuStatus.received_count,
                       imuStatus.accepted_count, imuStatus.dropped_count);
}

void MonocularInertialSlamNode::ClearStableInputPause()
{
  std::lock_guard<std::mutex> lock(m_pendingImageMutex);
  m_stableInputPaused = false;
  m_stableInputPauseNewestImuStampNs.reset();
  m_stableInputPauseWallNs.reset();
  m_stableInputPauseReceivedImuCount = 0;
  m_stableInputPauseAcceptedImuCount = 0;
  m_stableInputPauseDroppedImuCount = 0;
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
  bool initialDiagnosticPending = false;

  {
    std::lock_guard<std::mutex> lock(m_pendingImageMutex);
    pendingQueueSize = m_pendingImages.size();
    releasedImageCount = m_releasedImageCount;
    lastDiagnosticWallNs = m_lastTrackingDiagnosticWallNs;
    lastDiagnosticImageStampNs = m_lastTrackingDiagnosticImageStampNs;
    lastDiagnosticReleasedImageCount = m_lastTrackingDiagnosticReleasedImageCount;
    lastDiagnosticAcceptedImuCount = m_lastTrackingDiagnosticAcceptedImuCount;
    initialDiagnosticPending = m_initialTrackingDiagnosticPending;

    if (!lastDiagnosticWallNs || nowNs - *lastDiagnosticWallNs < TRACKING_DIAGNOSTIC_PERIOD_NS)
    {
      initializeDiagnosticBaseline = !lastDiagnosticWallNs;
    }
  }

  if (!initialDiagnosticPending && !initializeDiagnosticBaseline &&
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
  if (initialDiagnosticPending && (imageStampDeltaNs <= 0 || wallDeltaNs <= 0))
    return;

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

  if (initialDiagnosticPending)
  {
    RCLCPP_INFO(*m_logger, "SLAM health: img=%.1fHz imu=%.1fHz q=%zu lag=%.1fms",
                imageReleaseRateHz, imuRateHz, pendingQueueSize,
                static_cast<double>(lagNs) / 1.0e6);
  }
  else
  {
    RCLCPP_INFO(*m_logger, "SLAM health: img=%.1fHz imu=%.1fHz q=%zu lag=%.1fms",
                imageReleaseRateHz, imuRateHz, pendingQueueSize,
                static_cast<double>(lagNs) / 1.0e6);
  }

  {
    std::lock_guard<std::mutex> lock(m_pendingImageMutex);
    m_lastTrackingDiagnosticWallNs = nowNs;
    m_lastTrackingDiagnosticImageStampNs = newestReleasedImageStampNs;
    m_lastTrackingDiagnosticReleasedImageCount = releasedImageCount;
    m_lastTrackingDiagnosticAcceptedImuCount = imuStatus.accepted_count;
    m_initialTrackingDiagnosticPending = false;
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
    RCLCPP_ERROR_THROTTLE(*m_logger, *m_node.get_clock(), IMU_READINESS_THROTTLE_MS,
                          "MI imu_timing: phase=stall img=%.3f newest_imu=%s lag_ms=%.0f sign=%s "
                          "queue=%zu stable=%d rx=%zu ok=%zu drop=%zu",
                          imageTimestamp, FormatTimestamp(imuStatus.newest_imu_stamp_ns).c_str(),
                          static_cast<double>(lagNs) / 1.0e6, LagSign(lagNs), pendingQueueSize,
                          stableMap, imuStatus.received_count, imuStatus.accepted_count,
                          imuStatus.dropped_count);
    return;
  }

  RCLCPP_WARN_THROTTLE(*m_logger, *m_node.get_clock(), IMU_READINESS_THROTTLE_MS,
                       "MI imu_timing: phase=runtime img=%.3f newest_imu=%s lag_ms=%.0f "
                       "sign=%s queue=%zu stable=%d rx=%zu ok=%zu drop=%zu",
                       imageTimestamp, FormatTimestamp(imuStatus.newest_imu_stamp_ns).c_str(),
                       static_cast<double>(lagNs) / 1.0e6, LagSign(lagNs), pendingQueueSize,
                       stableMap, imuStatus.received_count, imuStatus.accepted_count,
                       imuStatus.dropped_count);
}

void MonocularInertialSlamNode::LogStartupWaitingStatus(
    std::optional<int64_t> newestImageStampNs,
    const SLAM::MonocularInertialSlam::ImuBufferStatus& imuStatus,
    bool imuReady,
    bool armingWindowReady,
    int64_t lagNs,
    std::size_t pendingQueueSize,
    const char* reason) const
{
  RCLCPP_INFO_THROTTLE(*m_logger, *m_node.get_clock(), IMU_READINESS_THROTTLE_MS,
                       "MI imu_timing: phase=startup img=%s newest_imu=%s lag_ms=%.0f "
                       "sign=%s queue=%zu stable=%d rx=%zu ok=%zu drop=%zu reason=%s "
                       "image=%d imu=%d imu_ready=%d arm_window=%d",
                       FormatTimestamp(newestImageStampNs).c_str(),
                       FormatTimestamp(imuStatus.newest_imu_stamp_ns).c_str(),
                       static_cast<double>(lagNs) / 1.0e6, LagSign(lagNs), pendingQueueSize,
                       imuStatus.has_stable_slam_map ? 1 : 0, imuStatus.received_count,
                       imuStatus.accepted_count, imuStatus.dropped_count, reason,
                       newestImageStampNs ? 1 : 0, imuStatus.has_received_imu ? 1 : 0,
                       imuReady ? 1 : 0, armingWindowReady ? 1 : 0);
}

void MonocularInertialSlamNode::ResetStartupArmingCandidateLocked()
{
  m_startupArmingCandidateStartNs.reset();
}
