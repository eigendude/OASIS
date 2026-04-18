/*
 *  Copyright (C) 2022-2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "MonocularInertialSlam.h"

#include "ros/RosUtils.h"

#include <iomanip>
#include <optional>
#include <sstream>

#include <geometry_msgs/msg/vector3.hpp>
#include <sophus/se3.hpp>

using namespace OASIS;
using namespace SLAM;

namespace
{
bool IsFiniteMatrix4(const Eigen::Matrix4f& matrix)
{
  return matrix.allFinite();
}

std::string SummarizePoseMatrix(const Eigen::Matrix4f& matrix)
{
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(3) << "[r00=" << matrix(0, 0)
         << ", r11=" << matrix(1, 1) << ", r22=" << matrix(2, 2) << ", tx=" << matrix(0, 3)
         << ", ty=" << matrix(1, 3) << ", tz=" << matrix(2, 3) << "]";
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
  const geometry_msgs::msg::Vector3& linearAceleration = imuMsg->linear_acceleration;

  const double ax = linearAceleration.x;
  const double ay = linearAceleration.y;
  const double az = linearAceleration.z;

  const double gx = angularVelocity.x;
  const double gy = angularVelocity.y;
  const double gz = angularVelocity.z;

  const cv::Point3f acc(ax, ay, az);
  const cv::Point3f gyr(gx, gy, gz);

  m_imuMeasurements.push_back(ORB_SLAM3::IMU::Point(acc, gyr, timestamp));
}

std::optional<Eigen::Isometry3f> MonocularInertialSlam::TrackFrame(const cv::Mat& rgbImage,
                                                                   double timestamp)
{
  ORB_SLAM3::System* slam = GetSlam();
  if (slam == nullptr)
    return std::nullopt;

  try
  {
    const Sophus::SE3f sophusPose = slam->TrackMonocular(rgbImage, timestamp, m_imuMeasurements);
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
  m_imuMeasurements.clear();
}
