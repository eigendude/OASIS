/*
 *  Copyright (C) 2022-2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#pragma once

#include "MonocularSlamBase.h"

#include <mutex>
#include <optional>
#include <vector>

#include <sensor_msgs/msg/image.hpp>
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

  // Lifecycle functions
  bool Initialize(const std::string& vocabularyFile, const std::string& settingsFile);
  void Deinitialize();

  // Node interface
  void ReceiveImu(const sensor_msgs::msg::Imu& imuMsg);
  void ReceiveImageWithImuMeasurements(const sensor_msgs::msg::Image& imageMsg,
                                       const std::vector<sensor_msgs::msg::Imu>& imuMessages);

protected:
  // Implementation of MonocularSlamBase
  std::optional<Eigen::Isometry3f> TrackFrame(const cv::Mat& rgbImage,
                                              int64_t timestampNs) override;
  void OnPostTrack() override;

private:
  // Utility functions
  static ORB_SLAM3::IMU::Point ToOrbImuPoint(const sensor_msgs::msg::Imu& imuMsg);
};

} // namespace SLAM
} // namespace OASIS
