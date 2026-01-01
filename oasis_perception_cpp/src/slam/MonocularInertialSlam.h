/*
 *  Copyright (C) 2022-2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#pragma once

#include "MonocularSlamBase.h"

#include <deque>
#include <mutex>
#include <optional>
#include <vector>

#include <sensor_msgs/msg/imu.hpp>

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
  MonocularInertialSlam(rclcpp::Node& node,
                        const std::string& pointCloudTopic,
                        const std::string& poseTopic);
  ~MonocularInertialSlam() override;

  bool Initialize(const std::string& vocabularyFile, const std::string& settingsFile);
  void Deinitialize();

  void ImuCallback(const sensor_msgs::msg::Imu::ConstSharedPtr& msg);

protected:
  Eigen::Isometry3f TrackFrame(const cv::Mat& rgbImage, double timestamp) override;

private:
  // IMU parameters
  std::deque<ORB_SLAM3::IMU::Point> m_imuMeasurements;
  std::optional<double> m_lastImgTime;
  std::optional<double> m_lastImuTime;

  // oOptional: cap memory / age
  static constexpr double kImuMaxAgeSec = 10.0; // Keep last 10s of IMU
  static constexpr std::size_t kImuMaxCount = 2000; // Safety cap (50 Hz * 40s)

  // Threading parameters
  std::mutex m_imuMutex;
};

} // namespace SLAM
} // namespace OASIS
