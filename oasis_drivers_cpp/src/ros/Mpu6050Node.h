/*
 *  Copyright (C) 2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#pragma once

#include <memory>

#include <MPU6050.h>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/timer.hpp>
#include <sensor_msgs/msg/imu.hpp>

namespace OASIS
{
namespace ROS
{

class Mpu6050Node : public rclcpp::Node
{
public:
  Mpu6050Node();

private:
  void PublishImu();

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr m_publisher;
  rclcpp::TimerBase::SharedPtr m_timer;
  std::unique_ptr<MPU6050> m_mpu6050;

  double m_accelScale = 0.0;
  double m_gyroScale = 0.0;
  bool m_isConnected = false;
};

} // namespace ROS
} // namespace OASIS
