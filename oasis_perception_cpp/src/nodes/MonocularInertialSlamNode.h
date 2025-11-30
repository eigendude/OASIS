/*
 *  Copyright (C) 2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */
#pragma once

#include <memory>

#include <image_transport/subscriber.hpp>
#include <oasis_msgs/msg/i2_c_imu.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/time.hpp>
#include <sensor_msgs/msg/image.hpp>

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
  void OnImu(const oasis_msgs::msg::I2CImu::ConstSharedPtr& msg);

  // Construction parameters
  rclcpp::Node& m_node;

  // ROS parameters
  std::unique_ptr<rclcpp::Logger> m_logger;
  std::unique_ptr<image_transport::Subscriber> m_imgSubscriber;
  rclcpp::Subscription<oasis_msgs::msg::I2CImu>::SharedPtr m_imuSubscriber;
  double m_maxTrackingFps = 0.0;
  rclcpp::Time m_lastFrameTime;

  // Video parameters
  std::unique_ptr<SLAM::MonocularInertialSlam> m_monocularInertialSlam;
};
} // namespace ROS
} // namespace OASIS
