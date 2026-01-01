/*
 *  Copyright (C) 2022-2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "MonocularInertialSlam.h"

#include "ros/RosUtils.h"

#include <geometry_msgs/msg/vector3.hpp>
#include <sophus/se3.hpp>

using namespace OASIS;
using namespace SLAM;

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

  std::lock_guard<std::mutex> lock(m_imuMutex);

  // Enforce monotonic IMU time (drop out-of-order)
  if (m_lastImuTime && timestamp <= *m_lastImuTime)
    return;
  m_lastImuTime = timestamp;

  m_imuMeasurements.emplace_back(acc, gyr, timestamp);

  // Prune by age (relative to newest IMU time)
  const double min_t = timestamp - kImuMaxAgeSec;
  while (!m_imuMeasurements.empty() && m_imuMeasurements.front().t < min_t)
    m_imuMeasurements.pop_front();

  // Safety cap by count
  while (m_imuMeasurements.size() > kImuMaxCount)
    m_imuMeasurements.pop_front();
}

Eigen::Isometry3f MonocularInertialSlam::TrackFrame(const cv::Mat& rgbImage, double timestamp)
{
  ORB_SLAM3::System* slam = GetSlam();
  if (slam == nullptr)
    return Eigen::Isometry3f::Identity();

  std::vector<ORB_SLAM3::IMU::Point> imu_for_frame;
  {
    std::lock_guard<std::mutex> lock(m_imuMutex);

    const double t_prev = m_lastImgTime.value_or(timestamp);

    // If this is the first frame, you can either:
    //  (a) pass empty IMU and let ORB init, or
    //  (b) take a short warmup window. I usually do (a).
    if (!m_lastImgTime)
    {
      // Drop any future IMU? no; just keep buffer.
      m_lastImgTime = timestamp;
    }
    else
    {
      // Drop IMUs at/before previous image time (already accounted for)
      while (!m_imuMeasurements.empty() && m_imuMeasurements.front().t <= t_prev)
        m_imuMeasurements.pop_front();

      // Collect IMUs up to this image time
      while (!m_imuMeasurements.empty() && m_imuMeasurements.front().t <= timestamp)
      {
        imu_for_frame.push_back(m_imuMeasurements.front());
        m_imuMeasurements.pop_front();
      }

      m_lastImgTime = timestamp;
    }
  }

  // Optional: if you want to be strict, skip frames without IMU once initialized:
  // if (m_lastImgTime && imu_for_frame.empty()) { ... }

  const Sophus::SE3f sophusPose = slam->TrackMonocular(rgbImage, timestamp, imu_for_frame);

  Eigen::Isometry3f pose = Eigen::Isometry3f::Identity();
  pose.matrix() = sophusPose.matrix();

  return pose;
}
