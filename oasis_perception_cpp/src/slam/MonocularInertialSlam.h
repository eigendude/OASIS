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

  bool Initialize(const std::string& vocabularyFile, const std::string& settingsFile);
  void Deinitialize();

  void ReceiveImageWithImuMeasurements(const sensor_msgs::msg::Image::ConstSharedPtr& imageMsg,
                                       const std::vector<sensor_msgs::msg::Imu>& imuMessages);

protected:
  std::optional<Eigen::Isometry3f> TrackFrame(const cv::Mat& rgbImage, double timestamp) override;
  void OnPostTrack() override;

private:
  struct PendingImuBatch
  {
    double timestamp = 0.0;
    std::vector<ORB_SLAM3::IMU::Point> measurements;
  };

  static ORB_SLAM3::IMU::Point ToOrbImuPoint(const sensor_msgs::msg::Imu& imuMsg);

  std::vector<ORB_SLAM3::IMU::Point> TakePendingImuMeasurements(double timestamp);

  std::mutex m_imuMeasurementsMutex;
  std::vector<PendingImuBatch> m_pendingImuBatches;
  std::vector<ORB_SLAM3::IMU::Point> m_imuMeasurements;
};

} // namespace SLAM
} // namespace OASIS
