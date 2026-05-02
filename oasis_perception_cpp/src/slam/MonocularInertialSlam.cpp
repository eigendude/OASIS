/*
 *  Copyright (C) 2022-2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "MonocularInertialSlam.h"

#include "OrbImuSettingsValidator.h"
#include "ros/RosUtils.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <optional>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <ImuTypes.h>
#include <geometry_msgs/msg/vector3.hpp>
#include <rclcpp/logging.hpp>
#include <sophus/se3.hpp>

using namespace OASIS;
using namespace SLAM;

namespace
{
using SteadyTime = std::chrono::steady_clock::time_point;

// Maximum angular velocity accepted by the ROS boundary, in rad/s
constexpr double MAX_ANGULAR_VELOCITY_NORM = 100.0;

// Maximum linear acceleration accepted by the ROS boundary, in m/s^2
constexpr double MAX_LINEAR_ACCELERATION_NORM = 200.0;

// IMU sample gap that is suspicious but still passed to ORB-SLAM3, in seconds
constexpr double LARGE_IMU_DT_SECONDS = 0.1;

// Number of nanoseconds in one normalized ROS Time second
constexpr unsigned int NANOSECONDS_PER_SECOND = 1000000000U;

constexpr std::size_t ACCEPTED_IMU_SAMPLE_LOG_LIMIT = 20;
constexpr std::size_t IMAGE_IMU_BATCH_LOG_LIMIT = 20;

constexpr auto IMU_DROP_LOG_INTERVAL = std::chrono::seconds(1);
constexpr auto IMU_LARGE_DT_LOG_INTERVAL = std::chrono::seconds(1);
constexpr auto EMPTY_IMU_FRAME_LOG_INTERVAL = std::chrono::seconds(1);
constexpr auto FUTURE_IMU_RETAINED_LOG_INTERVAL = std::chrono::seconds(1);
constexpr auto IMU_SUMMARY_LOG_INTERVAL = std::chrono::seconds(5);

bool IsFiniteMatrix4(const Eigen::Matrix4f& matrix)
{
  return matrix.allFinite();
}

bool IsFiniteDouble(double value)
{
  return std::isfinite(value);
}

bool IsFiniteVector3(const geometry_msgs::msg::Vector3& vector)
{
  return IsFiniteDouble(vector.x) && IsFiniteDouble(vector.y) && IsFiniteDouble(vector.z);
}

double VectorNorm(const geometry_msgs::msg::Vector3& vector)
{
  return std::sqrt(vector.x * vector.x + vector.y * vector.y + vector.z * vector.z);
}

double VectorNorm(const Eigen::Vector3f& vector)
{
  return static_cast<double>(vector.cast<double>().norm());
}

bool ExceedsMagnitudeLimit(const geometry_msgs::msg::Vector3& vector, double maxNorm)
{
  return std::abs(vector.x) > maxNorm || std::abs(vector.y) > maxNorm ||
         std::abs(vector.z) > maxNorm || VectorNorm(vector) > maxNorm;
}

bool ShouldLog(SteadyTime& lastLogTime,
               SteadyTime now,
               std::chrono::steady_clock::duration interval)
{
  if (lastLogTime == SteadyTime{} || now - lastLogTime >= interval)
  {
    lastLogTime = now;
    return true;
  }

  return false;
}

std::string FormatOptionalDouble(const std::optional<double>& value)
{
  if (!value)
    return "n/a";

  std::ostringstream stream;
  stream << std::fixed << std::setprecision(6) << *value;
  return stream.str();
}

std::string FormatVector3(double x, double y, double z)
{
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(6) << "[" << x << ", " << y << ", " << z << "]";
  return stream.str();
}

std::string FormatVector3(const geometry_msgs::msg::Vector3& vector)
{
  return FormatVector3(vector.x, vector.y, vector.z);
}

std::string FormatRange(const std::optional<double>& minValue,
                        const std::optional<double>& maxValue)
{
  if (!minValue || !maxValue)
    return "n/a";

  std::ostringstream stream;
  stream << std::fixed << std::setprecision(6) << *minValue << ".." << *maxValue;
  return stream.str();
}

std::string SummarizePoseMatrix(const Eigen::Matrix4f& matrix)
{
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(3) << "[r00=" << matrix(0, 0)
         << ", r11=" << matrix(1, 1) << ", r22=" << matrix(2, 2) << ", tx=" << matrix(0, 3)
         << ", ty=" << matrix(1, 3) << ", tz=" << matrix(2, 3) << "]";
  return stream.str();
}

std::string SummarizeImuBatch(const std::vector<ORB_SLAM3::IMU::Point>& imuMeasurements)
{
  if (imuMeasurements.empty())
    return "first_imu=n/a, last_imu=n/a, dt_range=n/a, gyro_norm_range=n/a, "
           "accel_norm_range=n/a, contains_nonfinite=false";

  bool containsNonfinite = false;
  std::optional<double> minDt;
  std::optional<double> maxDt;
  std::optional<double> minGyroNorm;
  std::optional<double> maxGyroNorm;
  std::optional<double> minAccelNorm;
  std::optional<double> maxAccelNorm;

  for (std::size_t index = 0; index < imuMeasurements.size(); ++index)
  {
    const ORB_SLAM3::IMU::Point& measurement = imuMeasurements[index];
    const bool finite =
        measurement.a.allFinite() && measurement.w.allFinite() && IsFiniteDouble(measurement.t);
    containsNonfinite = containsNonfinite || !finite;

    if (finite)
    {
      const double gyroNorm = VectorNorm(measurement.w);
      const double accelNorm = VectorNorm(measurement.a);

      minGyroNorm = minGyroNorm ? std::min(*minGyroNorm, gyroNorm) : gyroNorm;
      maxGyroNorm = maxGyroNorm ? std::max(*maxGyroNorm, gyroNorm) : gyroNorm;
      minAccelNorm = minAccelNorm ? std::min(*minAccelNorm, accelNorm) : accelNorm;
      maxAccelNorm = maxAccelNorm ? std::max(*maxAccelNorm, accelNorm) : accelNorm;
    }

    if (index > 0)
    {
      const double dt = measurement.t - imuMeasurements[index - 1].t;
      if (IsFiniteDouble(dt))
      {
        minDt = minDt ? std::min(*minDt, dt) : dt;
        maxDt = maxDt ? std::max(*maxDt, dt) : dt;
      }
      else
      {
        containsNonfinite = true;
      }
    }
  }

  std::ostringstream stream;
  stream << std::fixed << std::setprecision(6) << "first_imu=" << imuMeasurements.front().t
         << ", last_imu=" << imuMeasurements.back().t << ", dt_range=" << FormatRange(minDt, maxDt)
         << ", gyro_norm_range=" << FormatRange(minGyroNorm, maxGyroNorm)
         << ", accel_norm_range=" << FormatRange(minAccelNorm, maxAccelNorm)
         << ", contains_nonfinite=" << (containsNonfinite ? "true" : "false");
  return stream.str();
}

} // namespace

MonocularInertialSlam::MonocularInertialSlam(rclcpp::Node& node,
                                             const std::string& pointCloudTopic,
                                             const std::string& poseTopic)
  : MonocularSlamBase(node, pointCloudTopic, poseTopic),
    m_lastImuSummaryLogTime(std::chrono::steady_clock::now())
{
}

MonocularInertialSlam::~MonocularInertialSlam() = default;

bool MonocularInertialSlam::Initialize(const std::string& vocabularyFile,
                                       const std::string& settingsFile)
{
  if (!ValidateOrbImuSettings(settingsFile, Logger()))
    return false;

  return InitializeSystem(vocabularyFile, settingsFile, ORB_SLAM3::System::IMU_MONOCULAR);
}

void MonocularInertialSlam::Deinitialize()
{
  DeinitializeSystem();
}

void MonocularInertialSlam::ImuCallback(const sensor_msgs::msg::Imu::ConstSharedPtr& imuMsg)
{
  if (!imuMsg || !HasSlam())
    return;

  const std_msgs::msg::Header& header = imuMsg->header;
  const double timestamp = ROS::RosUtils::HeaderStampToSeconds(header);

  const geometry_msgs::msg::Vector3& angularVelocity = imuMsg->angular_velocity;
  const geometry_msgs::msg::Vector3& linearAcceleration = imuMsg->linear_acceleration;

  const double ax = linearAcceleration.x;
  const double ay = linearAcceleration.y;
  const double az = linearAcceleration.z;

  const double gx = angularVelocity.x;
  const double gy = angularVelocity.y;
  const double gz = angularVelocity.z;

  const SteadyTime now = std::chrono::steady_clock::now();

  std::lock_guard<std::mutex> lock(m_imuMutex);

  ++m_receivedImuCount;

  const std::optional<double> previousReceivedTimestamp = m_previousReceivedImuTimestamp;
  const std::optional<double> previousAcceptedTimestamp = m_previousAcceptedImuTimestamp;
  const std::optional<double> receivedDt =
      previousReceivedTimestamp ? std::optional<double>{timestamp - *previousReceivedTimestamp}
                                : std::nullopt;
  const std::optional<double> acceptedDt =
      previousAcceptedTimestamp ? std::optional<double>{timestamp - *previousAcceptedTimestamp}
                                : std::nullopt;

  m_previousReceivedImuTimestamp = timestamp;

  auto warnAndDrop = [&](const char* reason, std::optional<double> dt, std::uint64_t& counter)
  {
    ++counter;

    if (ShouldLog(m_lastImuDropLogTime, now, IMU_DROP_LOG_INTERVAL))
    {
      RCLCPP_WARN(Logger(),
                  "Dropping IMU sample before ORB-SLAM3: %s "
                  "(stamp=%.6f, dt=%s, received_dt=%s, angular_velocity=%s, "
                  "linear_acceleration=%s, accepted=%llu, dropped_invalid_stamp=%llu, "
                  "dropped_nonfinite=%llu, dropped_nonmonotonic=%llu, "
                  "dropped_absurd_magnitude=%llu)",
                  reason, timestamp, FormatOptionalDouble(dt).c_str(),
                  FormatOptionalDouble(receivedDt).c_str(), FormatVector3(angularVelocity).c_str(),
                  FormatVector3(linearAcceleration).c_str(),
                  static_cast<unsigned long long>(m_acceptedImuCount),
                  static_cast<unsigned long long>(m_droppedInvalidStampImuCount),
                  static_cast<unsigned long long>(m_droppedNonfiniteImuCount),
                  static_cast<unsigned long long>(m_droppedNonmonotonicImuCount),
                  static_cast<unsigned long long>(m_droppedAbsurdMagnitudeImuCount));
    }

    LogImuSummaryIfDue(now);
  };

  if (header.stamp.sec == 0 && header.stamp.nanosec == 0)
  {
    warnAndDrop("zero header stamp", acceptedDt, m_droppedInvalidStampImuCount);
    return;
  }

  if (header.stamp.nanosec >= NANOSECONDS_PER_SECOND)
  {
    warnAndDrop("invalid header stamp nanoseconds", acceptedDt, m_droppedInvalidStampImuCount);
    return;
  }

  if (!IsFiniteDouble(timestamp) || timestamp <= 0.0)
  {
    warnAndDrop("invalid header stamp", acceptedDt, m_droppedInvalidStampImuCount);
    return;
  }

  if (!IsFiniteVector3(angularVelocity) || !IsFiniteVector3(linearAcceleration))
  {
    warnAndDrop("non-finite angular velocity or linear acceleration", acceptedDt,
                m_droppedNonfiniteImuCount);
    return;
  }

  if (acceptedDt && *acceptedDt <= 0.0)
  {
    warnAndDrop("non-monotonic stamp relative to previous accepted IMU", acceptedDt,
                m_droppedNonmonotonicImuCount);
    return;
  }

  const double angularVelocityNorm = VectorNorm(angularVelocity);
  const double linearAccelerationNorm = VectorNorm(linearAcceleration);
  if (ExceedsMagnitudeLimit(angularVelocity, MAX_ANGULAR_VELOCITY_NORM) ||
      ExceedsMagnitudeLimit(linearAcceleration, MAX_LINEAR_ACCELERATION_NORM))
  {
    warnAndDrop("absurd IMU magnitude", acceptedDt, m_droppedAbsurdMagnitudeImuCount);
    return;
  }

  if (acceptedDt && *acceptedDt > LARGE_IMU_DT_SECONDS)
  {
    ++m_largeDtImuCount;
    if (ShouldLog(m_lastLargeImuDtLogTime, now, IMU_LARGE_DT_LOG_INTERVAL))
    {
      RCLCPP_WARN(Logger(),
                  "Accepting IMU sample with large dt before ORB-SLAM3 "
                  "(stamp=%.6f, dt=%.6f s, threshold=%.3f s, angular_velocity=%s, "
                  "linear_acceleration=%s)",
                  timestamp, *acceptedDt, LARGE_IMU_DT_SECONDS,
                  FormatVector3(angularVelocity).c_str(),
                  FormatVector3(linearAcceleration).c_str());
    }
  }

  const cv::Point3f acc(ax, ay, az);
  const cv::Point3f gyr(gx, gy, gz);

  m_imuMeasurements.push_back(ORB_SLAM3::IMU::Point(acc, gyr, timestamp));
  m_previousAcceptedImuTimestamp = timestamp;
  ++m_acceptedImuCount;

  if (m_loggedAcceptedImuCount < ACCEPTED_IMU_SAMPLE_LOG_LIMIT)
  {
    ++m_loggedAcceptedImuCount;
    RCLCPP_INFO(Logger(),
                "Accepted IMU sample %zu before ORB-SLAM3 "
                "(stamp=%.6f, dt=%s, angular_velocity=%s, linear_acceleration=%s, "
                "acceleration_norm=%.6f)",
                m_loggedAcceptedImuCount, timestamp, FormatOptionalDouble(acceptedDt).c_str(),
                FormatVector3(angularVelocity).c_str(), FormatVector3(linearAcceleration).c_str(),
                linearAccelerationNorm);
  }

  RCLCPP_DEBUG(Logger(),
               "Accepted IMU sample before ORB-SLAM3 "
               "(stamp=%.6f, dt=%s, angular_velocity=%s, linear_acceleration=%s, "
               "angular_velocity_norm=%.6f, acceleration_norm=%.6f)",
               timestamp, FormatOptionalDouble(acceptedDt).c_str(),
               FormatVector3(angularVelocity).c_str(), FormatVector3(linearAcceleration).c_str(),
               angularVelocityNorm, linearAccelerationNorm);

  LogImuSummaryIfDue(now);
}

std::optional<Eigen::Isometry3f> MonocularInertialSlam::TrackFrame(const cv::Mat& rgbImage,
                                                                   double timestamp)
{
  ORB_SLAM3::System* slam = GetSlam();
  if (slam == nullptr)
    return std::nullopt;

  std::vector<ORB_SLAM3::IMU::Point> imuMeasurements;
  std::optional<double> newestAcceptedImuTimestamp;
  std::optional<double> previousImageTimestamp;
  std::size_t imageImuLogIndex = 0;
  std::size_t prunedOldImuCount = 0;
  std::size_t retainedFutureImuCount = 0;
  std::optional<double> oldestRetainedFutureImuTimestamp;

  {
    std::lock_guard<std::mutex> lock(m_imuMutex);
    previousImageTimestamp = m_previousAcceptedImageTimestamp;
    newestAcceptedImuTimestamp = m_previousAcceptedImuTimestamp;
    ++m_loggedImageImuCount;
    imageImuLogIndex = m_loggedImageImuCount;

    std::vector<ORB_SLAM3::IMU::Point> retainedImuMeasurements;
    imuMeasurements.reserve(m_imuMeasurements.size());
    retainedImuMeasurements.reserve(m_imuMeasurements.size());

    for (const ORB_SLAM3::IMU::Point& measurement : m_imuMeasurements)
    {
      if (previousImageTimestamp && measurement.t <= *previousImageTimestamp)
      {
        ++prunedOldImuCount;
        continue;
      }

      if (measurement.t <= timestamp)
      {
        imuMeasurements.push_back(measurement);
        continue;
      }

      retainedImuMeasurements.push_back(measurement);
    }

    m_imuMeasurements = std::move(retainedImuMeasurements);
    retainedFutureImuCount = m_imuMeasurements.size();
    if (!m_imuMeasurements.empty())
      oldestRetainedFutureImuTimestamp = m_imuMeasurements.front().t;

    m_previousAcceptedImageTimestamp = timestamp;
  }

  const std::optional<double> selectedFirstImuTimestamp =
      imuMeasurements.empty() ? std::nullopt : std::optional<double>{imuMeasurements.front().t};
  const std::optional<double> selectedLastImuTimestamp =
      imuMeasurements.empty() ? std::nullopt : std::optional<double>{imuMeasurements.back().t};
  const bool emptyImuBatch = imuMeasurements.empty();
  const SteadyTime now = std::chrono::steady_clock::now();

  if (imageImuLogIndex <= IMAGE_IMU_BATCH_LOG_LIMIT)
  {
    RCLCPP_INFO(Logger(),
                "Image IMU batch %zu before ORB-SLAM3 "
                "(image_stamp=%.6f, previous_image_stamp=%s, selected_imu_count=%zu, "
                "selected_first_imu=%s, selected_last_imu=%s, retained_future_imu_count=%zu, "
                "oldest_retained_future_imu=%s, newest_accepted_imu=%s, pruned_old_imu=%zu, "
                "empty=%s)",
                imageImuLogIndex, timestamp, FormatOptionalDouble(previousImageTimestamp).c_str(),
                imuMeasurements.size(), FormatOptionalDouble(selectedFirstImuTimestamp).c_str(),
                FormatOptionalDouble(selectedLastImuTimestamp).c_str(), retainedFutureImuCount,
                FormatOptionalDouble(oldestRetainedFutureImuTimestamp).c_str(),
                FormatOptionalDouble(newestAcceptedImuTimestamp).c_str(), prunedOldImuCount,
                emptyImuBatch ? "true" : "false");
  }

  RCLCPP_DEBUG(Logger(),
               "Image IMU batch before ORB-SLAM3 "
               "(image_stamp=%.6f, previous_image_stamp=%s, selected_imu_count=%zu, "
               "selected_first_imu=%s, selected_last_imu=%s, retained_future_imu_count=%zu, "
               "oldest_retained_future_imu=%s, newest_accepted_imu=%s, pruned_old_imu=%zu, "
               "empty=%s)",
               timestamp, FormatOptionalDouble(previousImageTimestamp).c_str(),
               imuMeasurements.size(), FormatOptionalDouble(selectedFirstImuTimestamp).c_str(),
               FormatOptionalDouble(selectedLastImuTimestamp).c_str(), retainedFutureImuCount,
               FormatOptionalDouble(oldestRetainedFutureImuTimestamp).c_str(),
               FormatOptionalDouble(newestAcceptedImuTimestamp).c_str(), prunedOldImuCount,
               emptyImuBatch ? "true" : "false");

  if (emptyImuBatch && ShouldLog(m_lastEmptyImuFrameLogTime, now, EMPTY_IMU_FRAME_LOG_INTERVAL))
  {
    RCLCPP_WARN(Logger(),
                "Image frame has no selected IMU measurements before ORB-SLAM3 "
                "(image_stamp=%.6f, previous_image_stamp=%s, latest_accepted_imu=%s, "
                "retained_future_imu_count=%zu, oldest_retained_future_imu=%s)",
                timestamp, FormatOptionalDouble(previousImageTimestamp).c_str(),
                FormatOptionalDouble(newestAcceptedImuTimestamp).c_str(), retainedFutureImuCount,
                FormatOptionalDouble(oldestRetainedFutureImuTimestamp).c_str());
  }

  if (retainedFutureImuCount > 0 &&
      ShouldLog(m_lastFutureImuRetainedLogTime, now, FUTURE_IMU_RETAINED_LOG_INTERVAL))
  {
    RCLCPP_WARN(Logger(),
                "Retaining future IMU measurements for a later image frame "
                "(image_stamp=%.6f, retained_future_imu_count=%zu, "
                "oldest_retained_future_imu=%s)",
                timestamp, retainedFutureImuCount,
                FormatOptionalDouble(oldestRetainedFutureImuTimestamp).c_str());
  }

  RCLCPP_DEBUG(Logger(),
               "TrackMonocular IMU debug before ORB-SLAM3 "
               "(image_stamp=%.6f, imu_measurements=%zu, %s)",
               timestamp, imuMeasurements.size(), SummarizeImuBatch(imuMeasurements).c_str());

  try
  {
    const Sophus::SE3f sophusPose = slam->TrackMonocular(rgbImage, timestamp, imuMeasurements);
    const Eigen::Matrix4f poseMatrix = sophusPose.matrix();

    if (!IsFiniteMatrix4(poseMatrix))
    {
      const int trackingState = slam->GetTrackingState();
      const std::size_t trackedPoints = slam->GetTrackedMapPoints().size();

      RCLCPP_ERROR(Logger(),
                   "Rejecting non-finite inertial SLAM pose in TrackMonocular at %.6f "
                   "(state=%d, tracked_points=%zu, pose=%s)",
                   timestamp, trackingState, trackedPoints,
                   SummarizePoseMatrix(poseMatrix).c_str());
      ResetActiveMap();
      return std::nullopt;
    }

    Eigen::Isometry3f pose = Eigen::Isometry3f::Identity();
    pose.matrix() = poseMatrix;

    return pose;
  }
  catch (const std::exception& ex)
  {
    RCLCPP_ERROR(Logger(), "Recoverable inertial SLAM failure in TrackMonocular at %.6f: %s",
                 timestamp, ex.what());
    ResetActiveMap();
    return std::nullopt;
  }
  catch (...)
  {
    RCLCPP_ERROR(Logger(),
                 "Recoverable inertial SLAM failure in TrackMonocular at %.6f: "
                 "unknown exception",
                 timestamp);
    ResetActiveMap();
    return std::nullopt;
  }
}

void MonocularInertialSlam::OnPostTrack()
{
}

void MonocularInertialSlam::LogImuSummaryIfDue(std::chrono::steady_clock::time_point now)
{
  if (!ShouldLog(m_lastImuSummaryLogTime, now, IMU_SUMMARY_LOG_INTERVAL))
    return;

  RCLCPP_INFO(Logger(),
              "IMU diagnostics summary before ORB-SLAM3 "
              "(received=%llu, accepted=%llu, dropped_invalid_stamp=%llu, "
              "dropped_nonfinite=%llu, dropped_nonmonotonic=%llu, "
              "dropped_absurd_magnitude=%llu, large_dt_accepted=%llu, "
              "queued=%zu, previous_received=%s, previous_accepted=%s, "
              "previous_image=%s)",
              static_cast<unsigned long long>(m_receivedImuCount),
              static_cast<unsigned long long>(m_acceptedImuCount),
              static_cast<unsigned long long>(m_droppedInvalidStampImuCount),
              static_cast<unsigned long long>(m_droppedNonfiniteImuCount),
              static_cast<unsigned long long>(m_droppedNonmonotonicImuCount),
              static_cast<unsigned long long>(m_droppedAbsurdMagnitudeImuCount),
              static_cast<unsigned long long>(m_largeDtImuCount), m_imuMeasurements.size(),
              FormatOptionalDouble(m_previousReceivedImuTimestamp).c_str(),
              FormatOptionalDouble(m_previousAcceptedImuTimestamp).c_str(),
              FormatOptionalDouble(m_previousAcceptedImageTimestamp).c_str());
}
