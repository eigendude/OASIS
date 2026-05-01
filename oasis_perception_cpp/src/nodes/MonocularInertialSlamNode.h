/*
 *  Copyright (C) 2025-2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */
#pragma once

#include <memory>

#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
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
  void OnAccel(const geometry_msgs::msg::AccelWithCovarianceStamped::ConstSharedPtr& msg);
  void TryPublishSyncedImu();

  // Construction parameters
  rclcpp::Node& m_node;

  // ROS parameters
  std::unique_ptr<rclcpp::Logger> m_logger;
  std::unique_ptr<image_transport::Subscriber> m_imgSubscriber;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr m_imuSubscriber;
  rclcpp::Subscription<geometry_msgs::msg::AccelWithCovarianceStamped>::SharedPtr m_accelSubscriber;

  // IMU pairing state
  sensor_msgs::msg::Imu::ConstSharedPtr m_latestImuMsg;
  geometry_msgs::msg::AccelWithCovarianceStamped::ConstSharedPtr m_latestAccelMsg;

  // Video parameters
  std::unique_ptr<SLAM::MonocularInertialSlam> m_monocularInertialSlam;
};
} // namespace ROS
} // namespace OASIS
