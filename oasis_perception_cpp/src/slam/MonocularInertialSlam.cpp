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
#include <opencv2/core/persistence.hpp>
#include <rclcpp/logging.hpp>
#include <sophus/se3.hpp>

using namespace OASIS;
using namespace SLAM;

namespace
{
constexpr std::size_t ORB_IMU_BUFFER_MAX_SAMPLES = 1024;
constexpr int ORB_TRACKING_OK = 2;

// Nanoseconds; unarmed startup history kept only for arming health checks
constexpr std::int64_t STARTUP_IMU_BUFFER_WINDOW_NS = 3'000'000'000;

// Samples; hard cap for pathological startup stamps that do not advance
constexpr std::size_t STARTUP_IMU_BUFFER_MAX_SAMPLES = 4096;

constexpr std::int64_t IMU_INTERVAL_GAP_WARN_NS = 30'000'000;
constexpr std::int64_t MAX_IMU_STAMP_GAP_NS = 500'000'000;
constexpr int IMU_DIAGNOSTIC_THROTTLE_MS = 5'000;
constexpr int YAW_DIAGNOSTIC_THROTTLE_MS = 500;
constexpr std::int64_t IMU_MOTION_HISTORY_NS = 1'500'000'000;
constexpr std::int64_t IMU_MOTION_WINDOW_05_NS = 500'000'000;
constexpr std::int64_t IMU_MOTION_WINDOW_1_NS = 1'000'000'000;

// Standard gravity magnitude used for acceleration norm deviation, m/s^2
constexpr double GRAVITY_MPS2 = 9.80665;

// Diagnostic curve marker threshold for total angular rate, rad/s
constexpr double CURVE_GYRO_NORM_THRESHOLD_RADS = 0.35;

// Diagnostic curve marker threshold for message-frame z angular rate, rad/s
constexpr double CURVE_GYRO_Z_THRESHOLD_RADS = 0.25;

// Estimated 1s pitch delta required before plausibility warning, degrees
constexpr double ATTITUDE_COMPARE_EST_WARN_DEG = 15.0;

// Maximum supporting roll/pitch gyro integral for overreaction warning, degrees
constexpr double ATTITUDE_COMPARE_GYRO_SUPPORT_DEG = 6.0;

// Maximum gyro norm for stationary classification, rad/s
constexpr double STATIONARY_GYRO_NORM_THRESHOLD_RADS = 0.04;

// Maximum 1s integrated gyro angle for stationary classification, degrees
constexpr double STATIONARY_GYRO_INT_THRESHOLD_DEG = 2.0;

// Maximum mean acceleration norm deviation from gravity, m/s^2
constexpr double STATIONARY_ACCEL_GRAVITY_DEV_MPS2 = 0.35;

// Stationary pose translation drift warning threshold, meters
constexpr double STATIONARY_DRIFT_TRANSLATION_WARN_M = 0.03;

// Stationary pose rotation drift warning threshold, degrees
constexpr double STATIONARY_DRIFT_ROTATION_WARN_DEG = 3.0;

constexpr double PI = 3.14159265358979323846;
constexpr double RAD_TO_DEG = 180.0 / PI;

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

double WrapAngleRadians(double angleRad)
{
  while (angleRad > PI)
    angleRad -= 2.0 * PI;

  while (angleRad < -PI)
    angleRad += 2.0 * PI;

  return angleRad;
}

Eigen::Vector3d RotationMatrixToRollPitchYaw(const Eigen::Matrix3d& rotation)
{
  const double roll = std::atan2(rotation(2, 1), rotation(2, 2));
  const double pitch = std::atan2(-rotation(2, 0), std::sqrt(rotation(2, 1) * rotation(2, 1) +
                                                             rotation(2, 2) * rotation(2, 2)));
  const double yaw = std::atan2(rotation(1, 0), rotation(0, 0));

  return {roll, pitch, yaw};
}

const char* DominantGyroAxis(const geometry_msgs::msg::Vector3& gyro)
{
  const double absX = std::abs(gyro.x);
  const double absY = std::abs(gyro.y);
  const double absZ = std::abs(gyro.z);

  if (absX >= absY && absX >= absZ)
    return gyro.x >= 0.0 ? "+x" : "-x";

  if (absY >= absX && absY >= absZ)
    return gyro.y >= 0.0 ? "+y" : "-y";

  return gyro.z >= 0.0 ? "+z" : "-z";
}

const char* SignedAxisName(const Eigen::Vector3d& vector)
{
  int axisIndex = 0;
  double axisMagnitude = std::abs(vector.x());

  if (std::abs(vector.y()) > axisMagnitude)
  {
    axisIndex = 1;
    axisMagnitude = std::abs(vector.y());
  }

  if (std::abs(vector.z()) > axisMagnitude)
    axisIndex = 2;

  const double axisValue = vector(axisIndex);
  if (axisIndex == 0)
    return axisValue >= 0.0 ? "+x" : "-x";

  if (axisIndex == 1)
    return axisValue >= 0.0 ? "+y" : "-y";

  return axisValue >= 0.0 ? "+z" : "-z";
}

std::string FormatVector3(const Eigen::Vector3d& vector)
{
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(3) << "(" << vector.x() << ", " << vector.y() << ", "
         << vector.z() << ")";
  return stream.str();
}

std::string FormatUnitVector3(const Eigen::Vector3d& vector)
{
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(1) << "(" << vector.x() << ", " << vector.y() << ", "
         << vector.z() << ")";
  return stream.str();
}

double Norm3(double x, double y, double z)
{
  return std::sqrt(x * x + y * y + z * z);
}

std::optional<Eigen::Matrix4d> ReadOpenCvMatrix4(const std::string& settingsFile, const char* key)
{
  cv::FileStorage fileStorage(settingsFile, cv::FileStorage::READ);
  if (!fileStorage.isOpened())
    return std::nullopt;

  const cv::FileNode matrixNode = fileStorage[key];
  if (matrixNode.empty())
    return std::nullopt;

  const cv::Mat cvMatrix = matrixNode.mat();
  if (cvMatrix.rows != 4 || cvMatrix.cols != 4)
    return std::nullopt;

  Eigen::Matrix4d matrix = Eigen::Matrix4d::Identity();
  for (int row = 0; row < 4; ++row)
  {
    for (int col = 0; col < 4; ++col)
    {
      if (cvMatrix.depth() == CV_64F)
        matrix(row, col) = cvMatrix.at<double>(row, col);
      else
        matrix(row, col) = cvMatrix.at<float>(row, col);
    }
  }

  return matrix;
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

  LogCameraImuTransform(settingsFile);

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
    m_loggedOrbImuSanity = false;
    ResetMotionDiagnosticsLocked();
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
    m_loggedOrbImuSanity = false;
    ResetMotionDiagnosticsLocked();
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
    m_loggedOrbImuSanity = false;
    ResetMotionDiagnosticsLocked();

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

  const double gyroNorm =
      std::sqrt(angularVelocity.x * angularVelocity.x + angularVelocity.y * angularVelocity.y +
                angularVelocity.z * angularVelocity.z);
  LogImuYawDiagnostic(imuMsg, gyroNorm);
  UpdateImuMotionDiagnostics(imuMsg);
  LogOrbImuSanityLocked();

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
    m_loggedOrbImuSanity = false;
    ResetMotionDiagnosticsLocked();
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
    m_loggedOrbImuSanity = false;
    ResetMotionDiagnosticsLocked();
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
    m_loggedOrbImuSanity = false;
    ResetMotionDiagnosticsLocked();
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
    m_loggedOrbImuSanity = false;
    ResetMotionDiagnosticsLocked();
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
    LogPoseYawDiagnostic(*slam, timestampNs, pose, trackingState, trackedMapPoints.size(),
                         mapPoints.size());

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

void MonocularInertialSlam::ResetMotionDiagnosticsLocked()
{
  m_lastPoseYawRad.reset();
  m_latestGyroDiagnostic.reset();
  m_curveActive = false;
  m_lastLoggedCurveActive.reset();
  m_lastImuWindow05Sec.reset();
  m_lastImuWindow1Sec.reset();
  m_lastPoseDelta05Sec.reset();
  m_lastPoseDelta1Sec.reset();
  m_stationaryDiagnosticActive = false;
  m_stationaryReferencePose.reset();
  m_stationaryReferenceState.reset();
  m_imuDiagnosticWindow.clear();
  m_poseAttitudeDiagnosticWindow.clear();
}

void MonocularInertialSlam::LogCameraImuTransform(const std::string& settingsFile)
{
  const std::optional<Eigen::Matrix4d> tbcMatrix = ReadOpenCvMatrix4(settingsFile, "IMU.T_b_c1");
  if (!tbcMatrix)
  {
    RCLCPP_WARN(Logger(), "Unable to read IMU.T_b_c1 from %s", settingsFile.c_str());
    return;
  }

  const Eigen::Matrix4d tcbMatrix = tbcMatrix->inverse();
  const Eigen::Matrix3d rbc = tbcMatrix->block<3, 3>(0, 0);
  const Eigen::Matrix3d rcb = tcbMatrix.block<3, 3>(0, 0);
  m_tbcRotation = rbc;
  m_tcbRotation = rcb;
  const Eigen::Vector3d tbcRpy = RotationMatrixToRollPitchYaw(tbcMatrix->block<3, 3>(0, 0));
  const Eigen::Vector3d tcbRpy = RotationMatrixToRollPitchYaw(tcbMatrix.block<3, 3>(0, 0));
  const Eigen::Vector3d cameraXInBody = rbc * Eigen::Vector3d::UnitX();
  const Eigen::Vector3d cameraYInBody = rbc * Eigen::Vector3d::UnitY();
  const Eigen::Vector3d cameraZInBody = rbc * Eigen::Vector3d::UnitZ();
  const Eigen::Vector3d bodyXInCamera = rcb * Eigen::Vector3d::UnitX();
  const Eigen::Vector3d bodyYInCamera = rcb * Eigen::Vector3d::UnitY();
  const Eigen::Vector3d bodyZInCamera = rcb * Eigen::Vector3d::UnitZ();
  const Eigen::Vector3d gravityDownBaseInCamera = rcb * (-Eigen::Vector3d::UnitZ());
  const Eigen::Vector3d stationarySpecificForceBaseInCamera = rcb * Eigen::Vector3d::UnitZ();

  RCLCPP_INFO(Logger(),
              "ORB IMU extrinsic: Tbc=body<-camera Tcb=camera<-body "
              "tbc=(%.4f,%.4f,%.4f)m rpy=(%.1f,%.1f,%.1f)deg "
              "tcb=(%.4f,%.4f,%.4f)m rpy=(%.1f,%.1f,%.1f)deg",
              (*tbcMatrix)(0, 3), (*tbcMatrix)(1, 3), (*tbcMatrix)(2, 3), tbcRpy.x() * RAD_TO_DEG,
              tbcRpy.y() * RAD_TO_DEG, tbcRpy.z() * RAD_TO_DEG, tcbMatrix(0, 3), tcbMatrix(1, 3),
              tcbMatrix(2, 3), tcbRpy.x() * RAD_TO_DEG, tcbRpy.y() * RAD_TO_DEG,
              tcbRpy.z() * RAD_TO_DEG);
  RCLCPP_INFO(Logger(),
              "ORB axis/gravity self-test: cam_body=(x:%s y:%s z:%s) "
              "body_cam=(x:%s y:%s z:%s) base+z_cam=%s gravity_cam=%s spec_cam=%s",
              SignedAxisName(cameraXInBody), SignedAxisName(cameraYInBody),
              SignedAxisName(cameraZInBody), SignedAxisName(bodyXInCamera),
              SignedAxisName(bodyYInCamera), SignedAxisName(bodyZInCamera),
              SignedAxisName(bodyZInCamera), SignedAxisName(gravityDownBaseInCamera),
              SignedAxisName(stationarySpecificForceBaseInCamera));
}

void MonocularInertialSlam::LogOrbImuSanityLocked()
{
  if (m_loggedOrbImuSanity || !m_lastImuWindow1Sec || !m_tcbRotation)
    return;

  ORB_SLAM3::System* slam = GetSlam();
  if (slam != nullptr && slam->IsImuInitialized())
    return;

  const ImuWindowDiagnostic& window = *m_lastImuWindow1Sec;
  if (window.duration_sec < 0.8)
    return;

  const Eigen::Vector3d accelMeanBase(window.accel_mean_x_mps2, window.accel_mean_y_mps2,
                                      window.accel_mean_z_mps2);
  const Eigen::Vector3d accelMeanCamera = *m_tcbRotation * accelMeanBase;

  RCLCPP_INFO(Logger(),
              "ORB IMU sanity: accel_mean_base=%s m/s2 accel_mean_camera=%s m/s2 "
              "specific_force_expected_camera=-y gravity_expected_camera=+y",
              FormatVector3(accelMeanBase).c_str(), FormatVector3(accelMeanCamera).c_str());
  m_loggedOrbImuSanity = true;
}

void MonocularInertialSlam::LogImuYawDiagnostic(const sensor_msgs::msg::Imu& imuMsg,
                                                double gyroNorm)
{
  const geometry_msgs::msg::Vector3& gyro = imuMsg.angular_velocity;
  const int64_t imuStampNs = StampToNanoseconds(imuMsg.header.stamp);

  m_latestGyroDiagnostic = GyroDiagnosticSample{
      imuStampNs, gyro.x, gyro.y, gyro.z, gyroNorm,
  };

  const double timestamp = static_cast<double>(imuStampNs) / 1'000'000'000.0;
  RCLCPP_DEBUG_THROTTLE(Logger(), Clock(), YAW_DIAGNOSTIC_THROTTLE_MS,
                        "Yaw diag IMU: t=%.3f frame=%s gyro=(%.4f, %.4f, %.4f)rad/s norm=%.4f "
                        "dominant=%s expected_yaw_axis=+z yaw_rate_z=%.4f",
                        timestamp, imuMsg.header.frame_id.c_str(), gyro.x, gyro.y, gyro.z, gyroNorm,
                        DominantGyroAxis(gyro), gyro.z);
}

void MonocularInertialSlam::UpdateImuMotionDiagnostics(const sensor_msgs::msg::Imu& imuMsg)
{
  const int64_t imuStampNs = StampToNanoseconds(imuMsg.header.stamp);
  const geometry_msgs::msg::Vector3& gyro = imuMsg.angular_velocity;
  const geometry_msgs::msg::Vector3& accel = imuMsg.linear_acceleration;

  const ImuDiagnosticSample sample{
      imuStampNs, gyro.x, gyro.y, gyro.z, accel.x, accel.y, accel.z,
  };

  const auto insertIt =
      std::upper_bound(m_imuDiagnosticWindow.begin(), m_imuDiagnosticWindow.end(), imuStampNs,
                       [](int64_t stampNs, const ImuDiagnosticSample& bufferedSample)
                       { return stampNs < bufferedSample.stamp_ns; });
  m_imuDiagnosticWindow.insert(insertIt, sample);

  const int64_t oldestRetainedStampNs = imuStampNs - IMU_MOTION_HISTORY_NS;
  while (!m_imuDiagnosticWindow.empty() &&
         m_imuDiagnosticWindow.front().stamp_ns < oldestRetainedStampNs)
  {
    m_imuDiagnosticWindow.pop_front();
  }

  m_lastImuWindow05Sec = ComputeImuWindowDiagnosticLocked(imuStampNs, IMU_MOTION_WINDOW_05_NS);
  m_lastImuWindow1Sec = ComputeImuWindowDiagnosticLocked(imuStampNs, IMU_MOTION_WINDOW_1_NS);

  const bool curveActive = (m_lastImuWindow1Sec && m_lastImuWindow1Sec->max_gyro_norm_rads >=
                                                       CURVE_GYRO_NORM_THRESHOLD_RADS) ||
                           std::abs(gyro.z) >= CURVE_GYRO_Z_THRESHOLD_RADS;
  m_curveActive = curveActive;

  if (m_lastImuWindow1Sec)
  {
    const ImuWindowDiagnostic& window = *m_lastImuWindow1Sec;
    const double int05XDeg =
        m_lastImuWindow05Sec ? m_lastImuWindow05Sec->gyro_integral_x_rad * RAD_TO_DEG : 0.0;
    const double int05YDeg =
        m_lastImuWindow05Sec ? m_lastImuWindow05Sec->gyro_integral_y_rad * RAD_TO_DEG : 0.0;
    const double int05ZDeg =
        m_lastImuWindow05Sec ? m_lastImuWindow05Sec->gyro_integral_z_rad * RAD_TO_DEG : 0.0;
    const double timestamp = static_cast<double>(imuStampNs) / 1'000'000'000.0;
    RCLCPP_DEBUG_THROTTLE(Logger(), Clock(), YAW_DIAGNOSTIC_THROTTLE_MS,
                          "IMU motion window: t=%.3f w=1.0 n=%zu gx=%.4f gy=%.4f gz=%.4f "
                          "ax=%.3f ay=%.3f az=%.3f int05=(%+.2f, %+.2f, %+.2f)deg "
                          "int10=(%+.2f, %+.2f, %+.2f)deg "
                          "max_g=%.4f accel_dev=%.3f curve_active=%d",
                          timestamp, window.sample_count, window.gyro_rms_x_rads,
                          window.gyro_rms_y_rads, window.gyro_rms_z_rads, window.accel_rms_x_mps2,
                          window.accel_rms_y_mps2, window.accel_rms_z_mps2, int05XDeg, int05YDeg,
                          int05ZDeg, window.gyro_integral_x_rad * RAD_TO_DEG,
                          window.gyro_integral_y_rad * RAD_TO_DEG,
                          window.gyro_integral_z_rad * RAD_TO_DEG, window.max_gyro_norm_rads,
                          window.max_accel_gravity_deviation_mps2, curveActive ? 1 : 0);
  }
}

std::optional<MonocularInertialSlam::ImuWindowDiagnostic> MonocularInertialSlam::
    ComputeImuWindowDiagnosticLocked(int64_t newestStampNs, int64_t windowNs) const
{
  if (m_imuDiagnosticWindow.empty())
    return std::nullopt;

  const int64_t windowStartNs = newestStampNs - windowNs;
  auto beginIt = std::lower_bound(
      m_imuDiagnosticWindow.begin(), m_imuDiagnosticWindow.end(), windowStartNs,
      [](const ImuDiagnosticSample& sample, int64_t stampNs) { return sample.stamp_ns < stampNs; });
  const auto endIt = std::upper_bound(
      m_imuDiagnosticWindow.begin(), m_imuDiagnosticWindow.end(), newestStampNs,
      [](int64_t stampNs, const ImuDiagnosticSample& sample) { return stampNs < sample.stamp_ns; });

  if (beginIt == endIt)
    return std::nullopt;

  ImuWindowDiagnostic diagnostic;
  const ImuDiagnosticSample* previousSample = nullptr;
  for (auto it = beginIt; it != endIt; ++it)
  {
    ++diagnostic.sample_count;

    diagnostic.gyro_rms_x_rads += it->gyro_x_rads * it->gyro_x_rads;
    diagnostic.gyro_rms_y_rads += it->gyro_y_rads * it->gyro_y_rads;
    diagnostic.gyro_rms_z_rads += it->gyro_z_rads * it->gyro_z_rads;
    diagnostic.accel_rms_x_mps2 += it->accel_x_mps2 * it->accel_x_mps2;
    diagnostic.accel_rms_y_mps2 += it->accel_y_mps2 * it->accel_y_mps2;
    diagnostic.accel_rms_z_mps2 += it->accel_z_mps2 * it->accel_z_mps2;
    diagnostic.accel_mean_x_mps2 += it->accel_x_mps2;
    diagnostic.accel_mean_y_mps2 += it->accel_y_mps2;
    diagnostic.accel_mean_z_mps2 += it->accel_z_mps2;

    const double gyroNorm =
        std::sqrt(it->gyro_x_rads * it->gyro_x_rads + it->gyro_y_rads * it->gyro_y_rads +
                  it->gyro_z_rads * it->gyro_z_rads);
    diagnostic.max_gyro_norm_rads = std::max(diagnostic.max_gyro_norm_rads, gyroNorm);

    const double accelNorm =
        std::sqrt(it->accel_x_mps2 * it->accel_x_mps2 + it->accel_y_mps2 * it->accel_y_mps2 +
                  it->accel_z_mps2 * it->accel_z_mps2);
    diagnostic.max_accel_gravity_deviation_mps2 =
        std::max(diagnostic.max_accel_gravity_deviation_mps2, std::abs(accelNorm - GRAVITY_MPS2));

    if (previousSample != nullptr)
    {
      const int64_t elapsedNs = it->stamp_ns - previousSample->stamp_ns;
      if (elapsedNs > 0)
      {
        const double dtSec = static_cast<double>(elapsedNs) / 1'000'000'000.0;
        diagnostic.gyro_integral_x_rad +=
            0.5 * (previousSample->gyro_x_rads + it->gyro_x_rads) * dtSec;
        diagnostic.gyro_integral_y_rad +=
            0.5 * (previousSample->gyro_y_rads + it->gyro_y_rads) * dtSec;
        diagnostic.gyro_integral_z_rad +=
            0.5 * (previousSample->gyro_z_rads + it->gyro_z_rads) * dtSec;
      }
    }

    previousSample = &(*it);
  }

  diagnostic.gyro_rms_x_rads =
      std::sqrt(diagnostic.gyro_rms_x_rads / static_cast<double>(diagnostic.sample_count));
  diagnostic.gyro_rms_y_rads =
      std::sqrt(diagnostic.gyro_rms_y_rads / static_cast<double>(diagnostic.sample_count));
  diagnostic.gyro_rms_z_rads =
      std::sqrt(diagnostic.gyro_rms_z_rads / static_cast<double>(diagnostic.sample_count));
  diagnostic.accel_rms_x_mps2 =
      std::sqrt(diagnostic.accel_rms_x_mps2 / static_cast<double>(diagnostic.sample_count));
  diagnostic.accel_rms_y_mps2 =
      std::sqrt(diagnostic.accel_rms_y_mps2 / static_cast<double>(diagnostic.sample_count));
  diagnostic.accel_rms_z_mps2 =
      std::sqrt(diagnostic.accel_rms_z_mps2 / static_cast<double>(diagnostic.sample_count));
  diagnostic.accel_mean_x_mps2 /= static_cast<double>(diagnostic.sample_count);
  diagnostic.accel_mean_y_mps2 /= static_cast<double>(diagnostic.sample_count);
  diagnostic.accel_mean_z_mps2 /= static_cast<double>(diagnostic.sample_count);
  const double accelMeanNorm =
      std::sqrt(diagnostic.accel_mean_x_mps2 * diagnostic.accel_mean_x_mps2 +
                diagnostic.accel_mean_y_mps2 * diagnostic.accel_mean_y_mps2 +
                diagnostic.accel_mean_z_mps2 * diagnostic.accel_mean_z_mps2);
  diagnostic.mean_accel_gravity_deviation_mps2 = std::abs(accelMeanNorm - GRAVITY_MPS2);

  const auto lastIt = std::prev(endIt);
  diagnostic.duration_sec =
      static_cast<double>(lastIt->stamp_ns - beginIt->stamp_ns) / 1'000'000'000.0;

  return diagnostic;
}

std::optional<MonocularInertialSlam::PoseAttitudeDiagnosticSample> MonocularInertialSlam::
    ComputePoseAttitudeDeltaLocked(int64_t newestStampNs, int64_t windowNs) const
{
  if (m_poseAttitudeDiagnosticWindow.size() < 2)
    return std::nullopt;

  const int64_t windowStartNs = newestStampNs - windowNs;
  const auto currentIt = std::prev(m_poseAttitudeDiagnosticWindow.end());
  auto referenceIt = std::lower_bound(
      m_poseAttitudeDiagnosticWindow.begin(), m_poseAttitudeDiagnosticWindow.end(), windowStartNs,
      [](const PoseAttitudeDiagnosticSample& sample, int64_t stampNs)
      { return sample.stamp_ns < stampNs; });

  if (referenceIt == m_poseAttitudeDiagnosticWindow.end() || referenceIt == currentIt)
  {
    if (currentIt == m_poseAttitudeDiagnosticWindow.begin())
      return std::nullopt;

    referenceIt = std::prev(currentIt);
  }

  return PoseAttitudeDiagnosticSample{
      currentIt->stamp_ns,
      currentIt->x_m - referenceIt->x_m,
      currentIt->y_m - referenceIt->y_m,
      currentIt->z_m - referenceIt->z_m,
      WrapAngleRadians(currentIt->roll_rad - referenceIt->roll_rad),
      WrapAngleRadians(currentIt->pitch_rad - referenceIt->pitch_rad),
      WrapAngleRadians(currentIt->yaw_rad - referenceIt->yaw_rad),
  };
}

void MonocularInertialSlam::LogPoseYawDiagnostic(ORB_SLAM3::System& slam,
                                                 int64_t timestampNs,
                                                 const Eigen::Isometry3f& pose,
                                                 int trackingState,
                                                 std::size_t trackedPoints,
                                                 std::size_t mapPoints)
{
  const Eigen::Isometry3f& tcwPose = pose;
  const Eigen::Isometry3f twcPose = tcwPose.inverse();
  const Eigen::Vector3d tcwRpy = RotationMatrixToRollPitchYaw(tcwPose.linear().cast<double>());
  const Eigen::Vector3d twcRpy = RotationMatrixToRollPitchYaw(twcPose.linear().cast<double>());
  const double tcwRollRad = tcwRpy.x();
  const double tcwPitchRad = tcwRpy.y();
  const double tcwYawRad = tcwRpy.z();
  const double twcRollRad = twcRpy.x();
  const double twcPitchRad = twcRpy.y();
  const double twcYawRad = twcRpy.z();
  const Eigen::Vector3f& twcTranslation = twcPose.translation();

  double deltaYawRad = 0.0;
  bool hasPreviousYaw = false;
  std::optional<GyroDiagnosticSample> latestGyroDiagnostic;
  std::optional<ImuWindowDiagnostic> imuWindow05Sec;
  std::optional<ImuWindowDiagnostic> imuWindow1Sec;
  std::optional<PoseAttitudeDiagnosticSample> poseDelta05Sec;
  std::optional<PoseAttitudeDiagnosticSample> poseDelta1Sec;
  bool curveActive = false;
  bool logCurveTransition = false;
  {
    std::lock_guard<std::mutex> lock(m_imuMutex);
    hasPreviousYaw = m_lastPoseYawRad.has_value();
    if (m_lastPoseYawRad)
      deltaYawRad = WrapAngleRadians(twcYawRad - *m_lastPoseYawRad);
    m_lastPoseYawRad = twcYawRad;

    m_poseAttitudeDiagnosticWindow.push_back(
        PoseAttitudeDiagnosticSample{timestampNs, twcTranslation.x(), twcTranslation.y(),
                                     twcTranslation.z(), twcRollRad, twcPitchRad, twcYawRad});
    const int64_t oldestPoseStampNs = timestampNs - IMU_MOTION_HISTORY_NS;
    while (!m_poseAttitudeDiagnosticWindow.empty() &&
           m_poseAttitudeDiagnosticWindow.front().stamp_ns < oldestPoseStampNs)
    {
      m_poseAttitudeDiagnosticWindow.pop_front();
    }

    poseDelta05Sec = ComputePoseAttitudeDeltaLocked(timestampNs, IMU_MOTION_WINDOW_05_NS);
    poseDelta1Sec = ComputePoseAttitudeDeltaLocked(timestampNs, IMU_MOTION_WINDOW_1_NS);
    m_lastPoseDelta05Sec = poseDelta05Sec;
    m_lastPoseDelta1Sec = poseDelta1Sec;

    latestGyroDiagnostic = m_latestGyroDiagnostic;
    imuWindow05Sec = m_lastImuWindow05Sec;
    imuWindow1Sec = m_lastImuWindow1Sec;
    curveActive = m_curveActive;
    logCurveTransition = !m_lastLoggedCurveActive || *m_lastLoggedCurveActive != curveActive;
    if (logCurveTransition)
      m_lastLoggedCurveActive = curveActive;
  }

  const double timestamp = static_cast<double>(timestampNs) / 1'000'000'000.0;
  const Eigen::Vector3f& tcwTranslation = tcwPose.translation();
  const bool imuInitialized = slam.IsImuInitialized();
  const double deltaYawDeg = hasPreviousYaw ? deltaYawRad * RAD_TO_DEG : 0.0;
  const double gyroZ = latestGyroDiagnostic ? latestGyroDiagnostic->z_rads : 0.0;
  const ORB_SLAM3::InertialStateDiagnostic inertialState = slam.GetInertialStateDiagnostic();

  RCLCPP_DEBUG_THROTTLE(Logger(), Clock(), YAW_DIAGNOSTIC_THROTTLE_MS,
                        "Pose convention diag: t=%.3f returned=Tcw(camera<-world) "
                        "Tcw_xyz=(%.3f, %.3f, %.3f)m Tcw_rpy=(%.2f, %.2f, %.2f)deg "
                        "Twc_xyz=(%.3f, %.3f, %.3f)m Twc_rpy=(%.2f, %.2f, %.2f)deg",
                        timestamp, tcwTranslation.x(), tcwTranslation.y(), tcwTranslation.z(),
                        tcwRollRad * RAD_TO_DEG, tcwPitchRad * RAD_TO_DEG, tcwYawRad * RAD_TO_DEG,
                        twcTranslation.x(), twcTranslation.y(), twcTranslation.z(),
                        twcRollRad * RAD_TO_DEG, twcPitchRad * RAD_TO_DEG, twcYawRad * RAD_TO_DEG);
  RCLCPP_DEBUG_THROTTLE(Logger(), Clock(), YAW_DIAGNOSTIC_THROTTLE_MS,
                        "Attitude diag: t=%.3f yaw=%.2fdeg pitch=%.2fdeg roll=%.2fdeg "
                        "dyaw=%+.2fdeg gyro_z=%+.4f state=%d imu_init=%d pts=%zu map=%zu",
                        timestamp, twcYawRad * RAD_TO_DEG, twcPitchRad * RAD_TO_DEG,
                        twcRollRad * RAD_TO_DEG, deltaYawDeg, gyroZ, trackingState,
                        imuInitialized ? 1 : 0, trackedPoints, mapPoints);
  RCLCPP_DEBUG_THROTTLE(Logger(), Clock(), YAW_DIAGNOSTIC_THROTTLE_MS,
                        "Yaw diag: t=%.3f state=%d imu_init=%d Twc_xyz=(%.3f, %.3f, %.3f)m "
                        "pose_yaw=%.2fdeg pose_pitch=%.2fdeg pose_roll=%.2fdeg dyaw=%+.2fdeg "
                        "gyro=(%.4f, %.4f, %.4f) gyro_norm=%.4f gyro_z=%+.4f pts=%zu map=%zu",
                        timestamp, trackingState, imuInitialized ? 1 : 0, twcTranslation.x(),
                        twcTranslation.y(), twcTranslation.z(), twcYawRad * RAD_TO_DEG,
                        twcPitchRad * RAD_TO_DEG, twcRollRad * RAD_TO_DEG, deltaYawDeg,
                        latestGyroDiagnostic ? latestGyroDiagnostic->x_rads : 0.0,
                        latestGyroDiagnostic ? latestGyroDiagnostic->y_rads : 0.0,
                        latestGyroDiagnostic ? latestGyroDiagnostic->z_rads : 0.0,
                        latestGyroDiagnostic ? latestGyroDiagnostic->norm_rads : 0.0, gyroZ,
                        trackedPoints, mapPoints);

  if (imuInitialized && m_tbcRotation)
  {
    const Eigen::Vector3d gravityWorld = inertialState.gravity_world;
    const Eigen::Vector3d gravityCamera = tcwPose.linear().cast<double>() * gravityWorld;
    const Eigen::Vector3d gravityBody = *m_tbcRotation * gravityCamera;
    RCLCPP_DEBUG_THROTTLE(
        Logger(), Clock(), YAW_DIAGNOSTIC_THROTTLE_MS,
        "MI gravity frames: t=%.3f gravity_world=%s gravity_camera=%s gravity_body=%s "
        "expected_stationary_accel_body=%s expected_gravity_body=%s "
        "specific_force_expected_camera=-y gravity_expected_camera=+y",
        timestamp, FormatVector3(gravityWorld).c_str(), FormatVector3(gravityCamera).c_str(),
        FormatVector3(gravityBody).c_str(), FormatUnitVector3(Eigen::Vector3d::UnitZ()).c_str(),
        FormatUnitVector3(-Eigen::Vector3d::UnitZ()).c_str());
  }

  const bool nearTrackingLoss = trackingState != ORB_TRACKING_OK || trackedPoints < 30;
  if (logCurveTransition || curveActive || nearTrackingLoss)
  {
    const double yawImu1SecDeg =
        imuWindow1Sec ? imuWindow1Sec->gyro_integral_z_rad * RAD_TO_DEG : 0.0;
    const double yawEst1SecDeg = poseDelta1Sec ? poseDelta1Sec->yaw_rad * RAD_TO_DEG : 0.0;
    const double rollErrDeg =
        (imuWindow1Sec && poseDelta1Sec)
            ? (poseDelta1Sec->roll_rad - imuWindow1Sec->gyro_integral_x_rad) * RAD_TO_DEG
            : 0.0;
    const double pitchErrDeg =
        (imuWindow1Sec && poseDelta1Sec)
            ? (poseDelta1Sec->pitch_rad - imuWindow1Sec->gyro_integral_y_rad) * RAD_TO_DEG
            : 0.0;
    const double rpErrDeg = Norm3(rollErrDeg, pitchErrDeg, 0.0);
    const double accelDev = imuWindow1Sec ? imuWindow1Sec->mean_accel_gravity_deviation_mps2 : 0.0;
    const double lagMs =
        latestGyroDiagnostic
            ? static_cast<double>(timestampNs - latestGyroDiagnostic->stamp_ns) / 1'000'000.0
            : -1.0;
    RCLCPP_INFO_THROTTLE(Logger(), Clock(), 1000,
                         "MI motion_health: curve=%d yaw_imu_1s=%+.2f yaw_est_1s=%+.2f rp_err=%.2f "
                         "accel_dev=%.3f ba=%.4f bg=%.4f pts=%zu/%zu lag_ms=%.1f",
                         curveActive ? 1 : 0, yawImu1SecDeg, yawEst1SecDeg, rpErrDeg, accelDev,
                         inertialState.accel_bias.norm(), inertialState.gyro_bias.norm(),
                         trackedPoints, mapPoints, lagMs);
  }

  if (imuInitialized && imuWindow05Sec && poseDelta05Sec)
  {
    const double errRollDeg =
        (poseDelta05Sec->roll_rad - imuWindow05Sec->gyro_integral_x_rad) * RAD_TO_DEG;
    const double errPitchDeg =
        (poseDelta05Sec->pitch_rad - imuWindow05Sec->gyro_integral_y_rad) * RAD_TO_DEG;
    const double errYawDeg =
        (poseDelta05Sec->yaw_rad - imuWindow05Sec->gyro_integral_z_rad) * RAD_TO_DEG;
    RCLCPP_DEBUG_THROTTLE(
        Logger(), Clock(), YAW_DIAGNOSTIC_THROTTLE_MS,
        "Attitude compare: t=%.3f w=0.5 est_drp=(%+.2f, %+.2f, %+.2f)deg "
        "gyro_int=(%+.2f, %+.2f, %+.2f)deg err=(%+.2f, %+.2f, %+.2f)deg "
        "curve_active=%d state=%d pts=%zu map=%zu",
        timestamp, poseDelta05Sec->roll_rad * RAD_TO_DEG, poseDelta05Sec->pitch_rad * RAD_TO_DEG,
        poseDelta05Sec->yaw_rad * RAD_TO_DEG, imuWindow05Sec->gyro_integral_x_rad * RAD_TO_DEG,
        imuWindow05Sec->gyro_integral_y_rad * RAD_TO_DEG,
        imuWindow05Sec->gyro_integral_z_rad * RAD_TO_DEG, errRollDeg, errPitchDeg, errYawDeg,
        curveActive ? 1 : 0, trackingState, trackedPoints, mapPoints);
  }

  if (imuInitialized && imuWindow1Sec && poseDelta1Sec)
  {
    const double estRollDeg = poseDelta1Sec->roll_rad * RAD_TO_DEG;
    const double estPitchDeg = poseDelta1Sec->pitch_rad * RAD_TO_DEG;
    const double estYawDeg = poseDelta1Sec->yaw_rad * RAD_TO_DEG;
    const double gyroRollDeg = imuWindow1Sec->gyro_integral_x_rad * RAD_TO_DEG;
    const double gyroPitchDeg = imuWindow1Sec->gyro_integral_y_rad * RAD_TO_DEG;
    const double gyroYawDeg = imuWindow1Sec->gyro_integral_z_rad * RAD_TO_DEG;
    const double errRollDeg = estRollDeg - gyroRollDeg;
    const double errPitchDeg = estPitchDeg - gyroPitchDeg;
    const double errYawDeg = estYawDeg - gyroYawDeg;

    RCLCPP_DEBUG_THROTTLE(Logger(), Clock(), YAW_DIAGNOSTIC_THROTTLE_MS,
                          "Attitude compare: t=%.3f w=1.0 est_drp=(%+.2f, %+.2f, %+.2f)deg "
                          "gyro_int=(%+.2f, %+.2f, %+.2f)deg err=(%+.2f, %+.2f, %+.2f)deg "
                          "curve_active=%d state=%d pts=%zu map=%zu",
                          timestamp, estRollDeg, estPitchDeg, estYawDeg, gyroRollDeg, gyroPitchDeg,
                          gyroYawDeg, errRollDeg, errPitchDeg, errYawDeg, curveActive ? 1 : 0,
                          trackingState, trackedPoints, mapPoints);

    const double rollPitchGyroSupportDeg = std::max(std::abs(gyroRollDeg), std::abs(gyroPitchDeg));
    if (trackingState == ORB_TRACKING_OK && std::abs(estPitchDeg) > ATTITUDE_COMPARE_EST_WARN_DEG &&
        rollPitchGyroSupportDeg < ATTITUDE_COMPARE_GYRO_SUPPORT_DEG)
    {
      RCLCPP_WARN(Logger(),
                  "Attitude overreaction: Twc_delta=(%+.2f, %+.2f, %+.2f)deg "
                  "gyro_int=(%+.2f, %+.2f, %+.2f)deg accel_dev=%.3f curve_active=%d "
                  "state=%d pts=%zu map=%zu",
                  estRollDeg, estPitchDeg, estYawDeg, gyroRollDeg, gyroPitchDeg, gyroYawDeg,
                  imuWindow1Sec->max_accel_gravity_deviation_mps2, curveActive ? 1 : 0,
                  trackingState, trackedPoints, mapPoints);
    }
  }

  LogStationaryDiagnostics(slam, timestampNs, pose, trackingState, trackedPoints, mapPoints);
}

void MonocularInertialSlam::LogTrackingFailureDiagnostics(const char* failureReasonName,
                                                          int64_t imageStampNs,
                                                          int trackingState,
                                                          std::size_t trackedPoints,
                                                          std::size_t mapPoints)
{
  std::optional<ImuWindowDiagnostic> imuWindow1Sec;
  std::optional<PoseAttitudeDiagnosticSample> poseDelta1Sec;
  std::optional<int64_t> lastAcceptedImuStampNs;
  bool curveActive = false;
  {
    std::lock_guard<std::mutex> lock(m_imuMutex);
    imuWindow1Sec = m_lastImuWindow1Sec;
    poseDelta1Sec = m_lastPoseDelta1Sec;
    lastAcceptedImuStampNs = m_lastAcceptedImuStampNs;
    curveActive = m_curveActive;
  }

  const double imageTimestamp = static_cast<double>(imageStampNs) / 1'000'000'000.0;
  const double imageImuLagMs =
      lastAcceptedImuStampNs ? static_cast<double>(imageStampNs - *lastAcceptedImuStampNs) / 1.0e6
                             : 0.0;
  const double gyroIntXDeg = imuWindow1Sec ? imuWindow1Sec->gyro_integral_x_rad * RAD_TO_DEG : 0.0;
  const double gyroIntYDeg = imuWindow1Sec ? imuWindow1Sec->gyro_integral_y_rad * RAD_TO_DEG : 0.0;
  const double gyroIntZDeg = imuWindow1Sec ? imuWindow1Sec->gyro_integral_z_rad * RAD_TO_DEG : 0.0;
  const double poseDeltaRollDeg = poseDelta1Sec ? poseDelta1Sec->roll_rad * RAD_TO_DEG : 0.0;
  const double poseDeltaPitchDeg = poseDelta1Sec ? poseDelta1Sec->pitch_rad * RAD_TO_DEG : 0.0;
  const double poseDeltaYawDeg = poseDelta1Sec ? poseDelta1Sec->yaw_rad * RAD_TO_DEG : 0.0;
  const double accelDev = imuWindow1Sec ? imuWindow1Sec->max_accel_gravity_deviation_mps2 : 0.0;

  RCLCPP_WARN(Logger(),
              "Tracking failure diag: reason=%s t=%.3f curve=%d state=%d "
              "imu_int_1s=(%+.2f, %+.2f, %+.2f)deg "
              "est_drp_1s=(%+.2f, %+.2f, %+.2f)deg accel_dev=%.3f "
              "pts=%zu map=%zu ref_kf_matches=unavailable image_imu_lag=%.1fms",
              failureReasonName, imageTimestamp, curveActive ? 1 : 0, trackingState, gyroIntXDeg,
              gyroIntYDeg, gyroIntZDeg, poseDeltaRollDeg, poseDeltaPitchDeg, poseDeltaYawDeg,
              accelDev, trackedPoints, mapPoints, imageImuLagMs);
}

void MonocularInertialSlam::LogStationaryDiagnostics(ORB_SLAM3::System& slam,
                                                     int64_t timestampNs,
                                                     const Eigen::Isometry3f& pose,
                                                     int trackingState,
                                                     std::size_t trackedPoints,
                                                     std::size_t mapPoints)
{
  const auto poseSampleFromTcw = [](int64_t sampleTimestampNs, const Eigen::Isometry3f& tcwPose)
  {
    const Eigen::Isometry3f twcPose = tcwPose.inverse();
    const Eigen::Vector3d rpy = RotationMatrixToRollPitchYaw(twcPose.linear().cast<double>());
    const Eigen::Vector3f& translation = twcPose.translation();

    return PoseAttitudeDiagnosticSample{
        sampleTimestampNs, translation.x(), translation.y(), translation.z(),
        rpy.x(),           rpy.y(),         rpy.z()};
  };
  const auto poseDelta =
      [](const PoseAttitudeDiagnosticSample& reference, const PoseAttitudeDiagnosticSample& current)
  {
    return PoseAttitudeDiagnosticSample{
        current.stamp_ns,
        current.x_m - reference.x_m,
        current.y_m - reference.y_m,
        current.z_m - reference.z_m,
        WrapAngleRadians(current.roll_rad - reference.roll_rad),
        WrapAngleRadians(current.pitch_rad - reference.pitch_rad),
        WrapAngleRadians(current.yaw_rad - reference.yaw_rad),
    };
  };
  const auto formatPoseDelta = [](const PoseAttitudeDiagnosticSample& delta)
  {
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(3) << "dxyz=(" << delta.x_m << ", " << delta.y_m
           << ", " << delta.z_m << ")m drpy=(" << delta.roll_rad * RAD_TO_DEG << ", "
           << delta.pitch_rad * RAD_TO_DEG << ", " << delta.yaw_rad * RAD_TO_DEG << ")deg";
    return stream.str();
  };
  const auto formatOrbPoseDelta = [&poseSampleFromTcw, &poseDelta, &formatPoseDelta](
                                      const ORB_SLAM3::InertialStateDiagnostic& state,
                                      const Sophus::SE3f& candidatePose, bool valid)
  {
    if (!valid || !state.valid_last_pose)
      return std::string("unavailable");

    Eigen::Isometry3f referencePose = Eigen::Isometry3f::Identity();
    referencePose.matrix() = state.last_pose.matrix();
    Eigen::Isometry3f currentPose = Eigen::Isometry3f::Identity();
    currentPose.matrix() = candidatePose.matrix();

    return formatPoseDelta(
        poseDelta(poseSampleFromTcw(0, referencePose), poseSampleFromTcw(0, currentPose)));
  };

  const bool imuInitialized = slam.IsImuInitialized();
  std::optional<ImuWindowDiagnostic> imuWindow1Sec;
  {
    std::lock_guard<std::mutex> lock(m_imuMutex);
    imuWindow1Sec = m_lastImuWindow1Sec;
  }

  if (!imuInitialized || !imuWindow1Sec || imuWindow1Sec->duration_sec < 0.8)
  {
    std::lock_guard<std::mutex> lock(m_imuMutex);
    m_stationaryDiagnosticActive = false;
    m_stationaryReferencePose.reset();
    m_stationaryReferenceState.reset();
    return;
  }

  const double gyroIntegralNormDeg =
      Norm3(imuWindow1Sec->gyro_integral_x_rad, imuWindow1Sec->gyro_integral_y_rad,
            imuWindow1Sec->gyro_integral_z_rad) *
      RAD_TO_DEG;
  const bool likelyStationary =
      imuWindow1Sec->max_gyro_norm_rads <= STATIONARY_GYRO_NORM_THRESHOLD_RADS &&
      gyroIntegralNormDeg <= STATIONARY_GYRO_INT_THRESHOLD_DEG &&
      imuWindow1Sec->mean_accel_gravity_deviation_mps2 <= STATIONARY_ACCEL_GRAVITY_DEV_MPS2;

  if (!likelyStationary)
  {
    std::lock_guard<std::mutex> lock(m_imuMutex);
    m_stationaryDiagnosticActive = false;
    m_stationaryReferencePose.reset();
    m_stationaryReferenceState.reset();
    return;
  }

  const ORB_SLAM3::InertialStateDiagnostic state = slam.GetInertialStateDiagnostic();
  const PoseAttitudeDiagnosticSample currentPose = poseSampleFromTcw(timestampNs, pose);

  PoseAttitudeDiagnosticSample referencePose;
  ORB_SLAM3::InertialStateDiagnostic referenceState;
  {
    std::lock_guard<std::mutex> lock(m_imuMutex);
    if (!m_stationaryDiagnosticActive || !m_stationaryReferencePose || !m_stationaryReferenceState)
    {
      m_stationaryDiagnosticActive = true;
      m_stationaryReferencePose = currentPose;
      m_stationaryReferenceState = state;
    }
    referencePose = *m_stationaryReferencePose;
    referenceState = *m_stationaryReferenceState;
  }

  const PoseAttitudeDiagnosticSample stationaryPoseDelta = poseDelta(referencePose, currentPose);
  const double poseTranslationNorm =
      Norm3(stationaryPoseDelta.x_m, stationaryPoseDelta.y_m, stationaryPoseDelta.z_m);
  const double poseRotationNormDeg =
      Norm3(stationaryPoseDelta.roll_rad, stationaryPoseDelta.pitch_rad,
            stationaryPoseDelta.yaw_rad) *
      RAD_TO_DEG;

  const Eigen::Vector3f dVelocity = state.velocity - referenceState.velocity;
  const Eigen::Vector3f dGyroBias = state.gyro_bias - referenceState.gyro_bias;
  const Eigen::Vector3f dAccelBias = state.accel_bias - referenceState.accel_bias;
  const double dScale = state.scale - referenceState.scale;
  const double dGravityDeg = std::acos(std::clamp(referenceState.gravity_world.normalized().dot(
                                                      state.gravity_world.normalized()),
                                                  -1.0, 1.0)) *
                             RAD_TO_DEG;
  const Eigen::Vector3d gravityWorld = state.gravity_world;
  const Eigen::Vector3d gravityCamera = pose.linear().cast<double>() * gravityWorld;
  Eigen::Vector3d gravityBody = Eigen::Vector3d::Constant(NAN);
  if (m_tbcRotation)
    gravityBody = *m_tbcRotation * gravityCamera;

  const double timestamp = static_cast<double>(timestampNs) / 1'000'000'000.0;
  RCLCPP_INFO_THROTTLE(
      Logger(), Clock(), IMU_DIAGNOSTIC_THROTTLE_MS,
      "MI stationary: t=%.3f dt=%.2f %s vel=(%.3f, %.3f, %.3f)m/s "
      "dvel=(%.3f, %.3f, %.3f)m/s scale=%.6f dscale=%+.6f "
      "gravity_world=%s gravity_camera=%s gravity_body=%s dgravity_deg=%.3f "
      "expected_stationary_accel_body=%s expected_gravity_body=%s "
      "bg=(%.5f, %.5f, %.5f) dbg=(%+.5f, %+.5f, %+.5f) "
      "ba=(%.5f, %.5f, %.5f) dba=(%+.5f, %+.5f, %+.5f) "
      "gyro_int_1s=(%+.3f, %+.3f, %+.3f)deg "
      "accel_mean=(%.3f, %.3f, %.3f)m/s2 accel_dev=%.3f "
      "visual_pose_delta=%s imu_predicted_delta=%s optimized_pose_delta=%s "
      "inliers=%d local_matches=%d map=%zu pts=%zu visual_only=%d",
      timestamp, imuWindow1Sec->duration_sec, formatPoseDelta(stationaryPoseDelta).c_str(),
      state.velocity.x(), state.velocity.y(), state.velocity.z(), dVelocity.x(), dVelocity.y(),
      dVelocity.z(), state.scale, dScale, FormatVector3(gravityWorld).c_str(),
      FormatVector3(gravityCamera).c_str(), FormatVector3(gravityBody).c_str(), dGravityDeg,
      FormatUnitVector3(Eigen::Vector3d::UnitZ()).c_str(),
      FormatUnitVector3(-Eigen::Vector3d::UnitZ()).c_str(), state.gyro_bias.x(),
      state.gyro_bias.y(), state.gyro_bias.z(), dGyroBias.x(), dGyroBias.y(), dGyroBias.z(),
      state.accel_bias.x(), state.accel_bias.y(), state.accel_bias.z(), dAccelBias.x(),
      dAccelBias.y(), dAccelBias.z(), imuWindow1Sec->gyro_integral_x_rad * RAD_TO_DEG,
      imuWindow1Sec->gyro_integral_y_rad * RAD_TO_DEG,
      imuWindow1Sec->gyro_integral_z_rad * RAD_TO_DEG, imuWindow1Sec->accel_mean_x_mps2,
      imuWindow1Sec->accel_mean_y_mps2, imuWindow1Sec->accel_mean_z_mps2,
      imuWindow1Sec->mean_accel_gravity_deviation_mps2,
      formatOrbPoseDelta(state, state.visual_prediction_pose, state.valid_visual_prediction)
          .c_str(),
      formatOrbPoseDelta(state, state.imu_prediction_pose, state.valid_imu_prediction).c_str(),
      formatOrbPoseDelta(state, state.optimized_pose, state.valid_optimized_pose).c_str(),
      state.inliers, state.local_matches, mapPoints, trackedPoints,
      state.visual_only_after_init ? 1 : 0);

  if ((poseTranslationNorm >= STATIONARY_DRIFT_TRANSLATION_WARN_M ||
       poseRotationNormDeg >= STATIONARY_DRIFT_ROTATION_WARN_DEG) &&
      gyroIntegralNormDeg <= STATIONARY_GYRO_INT_THRESHOLD_DEG)
  {
    RCLCPP_WARN_THROTTLE(Logger(), Clock(), IMU_DIAGNOSTIC_THROTTLE_MS,
                         "MI stationary drift: %s gyro_int_1s=(%+.3f, %+.3f, %+.3f)deg "
                         "vel=(%.3f, %.3f, %.3f)m/s scale=%.6f dgravity=%.3fdeg "
                         "bg=(%.5f, %.5f, %.5f) ba=(%.5f, %.5f, %.5f) inliers=%d "
                         "state=%d pts=%zu map=%zu",
                         formatPoseDelta(stationaryPoseDelta).c_str(),
                         imuWindow1Sec->gyro_integral_x_rad * RAD_TO_DEG,
                         imuWindow1Sec->gyro_integral_y_rad * RAD_TO_DEG,
                         imuWindow1Sec->gyro_integral_z_rad * RAD_TO_DEG, state.velocity.x(),
                         state.velocity.y(), state.velocity.z(), state.scale, dGravityDeg,
                         state.gyro_bias.x(), state.gyro_bias.y(), state.gyro_bias.z(),
                         state.accel_bias.x(), state.accel_bias.y(), state.accel_bias.z(),
                         state.inliers, trackingState, trackedPoints, mapPoints);
  }
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

  if (isTransition && hasFailureReason)
  {
    LogTrackingFailureDiagnostics(failureReasonName, imageStampNs, trackingState, trackedPoints,
                                  mapPoints);
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
