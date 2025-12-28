/*
 *  Copyright (C) 2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#pragma once

#include "MotorIntent.h"
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
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

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
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr m_publisher;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr m_debugPublisher;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr m_statusPublisher;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr m_mappingDebugPublisher;
  rclcpp::Subscription<oasis_msgs::msg::ConductorState>::SharedPtr m_conductorStateSub;
  rclcpp::TimerBase::SharedPtr m_timer;

  // IMU parameters
  std::unique_ptr<MPU6050> m_mpu6050;
  std::string m_i2cDevice;
  std::chrono::duration<double> m_publishPeriod;
  std::unique_ptr<OASIS::IMU::Mpu6050ImuProcessor> m_imuProcessor;

  // Conductor state
  MotorIntent m_motorIntent;
  std::string m_conductorStateTopic;
  std::string m_imuDebugTopic;
  std::string m_imuStatusTopic;
  std::string m_imuMappingDebugTopic;
};

} // namespace ROS
} // namespace OASIS
