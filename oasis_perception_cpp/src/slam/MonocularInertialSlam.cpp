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
#include <oasis_msgs/msg/i2_c_imu.hpp>
#include <sensor_msgs/msg/imu.hpp>
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

void MonocularInertialSlam::ImuCallback(const oasis_msgs::msg::I2CImu::ConstSharedPtr& msg)
{
  if (!HasSlam())
    return;

  const sensor_msgs::msg::Imu& imuMsg = msg->imu;

  const std_msgs::msg::Header& header = imuMsg.header;
  const double timestamp = ROS::RosUtils::HeaderStampToSeconds(header);

  const geometry_msgs::msg::Vector3& angularVelocity = imuMsg.angular_velocity;
  const geometry_msgs::msg::Vector3& linearAceleration = imuMsg.linear_acceleration;

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

Eigen::Isometry3f MonocularInertialSlam::TrackFrame(const cv::Mat& rgbImage, double timestamp)
{
  ORB_SLAM3::System* slam = GetSlam();
  if (slam == nullptr)
    return Eigen::Isometry3f::Identity();

  const Sophus::SE3f sophusPose = slam->TrackMonocular(rgbImage, timestamp, m_imuMeasurements);

  Eigen::Isometry3f pose = Eigen::Isometry3f::Identity();
  pose.matrix() = sophusPose.matrix();

  return pose;
}

void MonocularInertialSlam::OnPostTrack()
{
  m_imuMeasurements.clear();
}
