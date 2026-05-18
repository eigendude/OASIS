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
#include <iomanip>
#include <optional>
#include <sstream>
#include <utility>

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

void MonocularInertialSlam::ReceiveImu(const sensor_msgs::msg::Imu& imuMsg)
{
  // TODO
  (void)imuMsg;
}

void MonocularInertialSlam::ReceiveImageWithImuMeasurements(
    const sensor_msgs::msg::Image& imageMsg, const std::vector<sensor_msgs::msg::Imu>& imuMessages)
{
  // TODO
}

std::optional<Eigen::Isometry3f> MonocularInertialSlam::TrackFrame(const cv::Mat& rgbImage,
                                                                   int64_t timestampNs)
{
  const double timestamp = static_cast<double>(timestampNs) / 1'000'000'000.0;

  ORB_SLAM3::System* slam = GetSlam();
  if (slam == nullptr)
    return std::nullopt;

  // TODO
  return std::nullopt;
}

void MonocularInertialSlam::OnPostTrack()
{
  // TODO
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
