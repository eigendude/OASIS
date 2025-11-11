/*
 *  Copyright (C) 2022-2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#pragma once

#include "MonocularSlamBase.h"

#include <vector>

#include <oasis_msgs/msg/i2_c_imu.hpp>

namespace ORB_SLAM3
{
namespace IMU
{
class Point;
} // namespace IMU
} // namespace ORB_SLAM3

namespace OASIS
{
namespace SLAM
{

class MonocularInertialSlam : public MonocularSlamBase
{
public:
  MonocularInertialSlam(rclcpp::Node& node, const std::string& mapImageTopic);
  ~MonocularInertialSlam() override;

  bool Initialize(const std::string& vocabularyFile, const std::string& settingsFile);
  void Deinitialize();

  void ImuCallback(const oasis_msgs::msg::I2CImu::ConstSharedPtr& msg);

protected:
  Eigen::Isometry3f TrackFrame(const cv::Mat& rgbImage, double timestamp) override;
  void OnPostTrack() override;

private:
  std::vector<ORB_SLAM3::IMU::Point> m_imuMeasurements;
};

} // namespace SLAM
} // namespace OASIS
