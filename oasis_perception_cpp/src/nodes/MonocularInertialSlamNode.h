/*
 *  Copyright (C) 2025-2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */
#pragma once

#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <optional>
#include <vector>

#include <builtin_interfaces/msg/time.hpp>
#include <image_transport/subscriber.hpp>
#include <rclcpp/subscription.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>

namespace rclcpp
{
class Logger;
class Node;
} // namespace rclcpp

namespace OASIS
{
namespace SLAM
{
class MonocularInertialSlam;
}

namespace ROS
{
class MonocularInertialSlamNode
{
public:
  MonocularInertialSlamNode(rclcpp::Node& node);
  ~MonocularInertialSlamNode();

  bool Initialize();
  void Deinitialize();

private:
  // ROS interface
  void OnImage(const sensor_msgs::msg::Image::ConstSharedPtr& msg);
  void OnImu(const sensor_msgs::msg::Imu::ConstSharedPtr& msg);
  std::vector<sensor_msgs::msg::Imu> TakeImuSamplesForImage(std::int64_t image_stamp_ns);
  void PruneImuBuffer(std::int64_t previous_image_stamp_ns);
  void DiagnoseImuBatch(const std::vector<sensor_msgs::msg::Imu>& imu_samples,
                        std::int64_t previous_image_stamp_ns,
                        std::int64_t image_stamp_ns);
  static std::int64_t StampToNanoseconds(const builtin_interfaces::msg::Time& stamp);

  // Construction parameters
  rclcpp::Node& m_node;

  // ROS parameters
  std::unique_ptr<rclcpp::Logger> m_logger;
  std::unique_ptr<image_transport::Subscriber> m_imgSubscriber;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr m_imuSubscriber;

  // Video parameters
  std::unique_ptr<SLAM::MonocularInertialSlam> m_monocularInertialSlam;

  // IMU buffering
  std::mutex m_imuBufferMutex;
  std::deque<sensor_msgs::msg::Imu> m_imuBuffer;
  std::optional<std::int64_t> m_lastImageStampNs;
  std::uint64_t m_emptyImuBufferImageCount{0};
  std::uint64_t m_emptyImuIntervalCount{0};
  std::uint64_t m_imuBufferDropCount{0};
};
} // namespace ROS
} // namespace OASIS
