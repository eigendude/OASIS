/*
 *  Copyright (C) 2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#pragma once

#include <chrono>
#include <memory>
#include <string>

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

  bool Initialize();
  void Deinitialize();

private:
  void PublishImu();

  // ROS parameters
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr m_publisher;
  rclcpp::TimerBase::SharedPtr m_timer;

  // IMU parameters
  std::unique_ptr<MPU6050> m_mpu6050;
  std::string m_i2cDevice;
  std::chrono::duration<double> m_publishPeriod;

  // IMU state
  double m_accelScale = 0.0;
  double m_gyroScale = 0.0;
};

} // namespace ROS
} // namespace OASIS
