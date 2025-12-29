/*
 *  Copyright (C) 2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#pragma once

#include "imu/Mpu6050ImuProcessor.h"

#include <chrono>
#include <memory>
#include <string>

#include <MPU6050.h>
#include <oasis_msgs/msg/conductor_state.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
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
  void OnConductorState(const oasis_msgs::msg::ConductorState& msg);

  // ROS parameters
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr m_imuPublisher;
  rclcpp::Subscription<oasis_msgs::msg::ConductorState>::SharedPtr m_conductorStateSub;
  rclcpp::TimerBase::SharedPtr m_timer;

  // IMU parameters
  std::unique_ptr<MPU6050> m_mpu6050;
  std::string m_i2cDevice;
  std::chrono::duration<double> m_publishPeriod;
  IMU::Mpu6050ImuProcessor m_imuProcessor;
  rclcpp::Time m_lastSampleTime{};
  bool m_hasLastSampleTime{false};

  // Station parameters
  double m_dutyCycleInput{0.0};
};

} // namespace ROS
} // namespace OASIS
