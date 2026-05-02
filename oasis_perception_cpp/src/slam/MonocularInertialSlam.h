/*
 *  Copyright (C) 2022-2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#pragma once

#include "MonocularSlamBase.h"

#include <chrono>
#include <cstddef>
#include <cstdint>
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
  std::optional<Eigen::Isometry3f> TrackFrame(const cv::Mat& rgbImage, double timestamp) override;
  void OnPostTrack() override;

private:
  void LogImuSummaryIfDue(std::chrono::steady_clock::time_point now);

  std::mutex m_imuMutex;
  std::vector<ORB_SLAM3::IMU::Point> m_imuMeasurements;

  std::optional<double> m_previousReceivedImuTimestamp;
  std::optional<double> m_previousAcceptedImuTimestamp;
  std::optional<double> m_previousAcceptedImageTimestamp;

  std::chrono::steady_clock::time_point m_lastImuDropLogTime;
  std::chrono::steady_clock::time_point m_lastLargeImuDtLogTime;
  std::chrono::steady_clock::time_point m_lastEmptyImuFrameLogTime;
  std::chrono::steady_clock::time_point m_lastFutureImuRetainedLogTime;
  std::chrono::steady_clock::time_point m_lastImuSummaryLogTime;

  std::uint64_t m_receivedImuCount = 0;
  std::uint64_t m_acceptedImuCount = 0;
  std::uint64_t m_droppedInvalidStampImuCount = 0;
  std::uint64_t m_droppedNonfiniteImuCount = 0;
  std::uint64_t m_droppedNonmonotonicImuCount = 0;
  std::uint64_t m_droppedAbsurdMagnitudeImuCount = 0;
  std::uint64_t m_largeDtImuCount = 0;
  std::size_t m_loggedAcceptedImuCount = 0;
  std::size_t m_loggedImageImuCount = 0;
};

} // namespace SLAM
} // namespace OASIS
