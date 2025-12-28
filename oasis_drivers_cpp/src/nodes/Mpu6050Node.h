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
  MotorIntent m_motorIntent{m_ewmaTau, m_conductorStaleSeconds};

  // Parameters
  // Prefer ~0.03-0.06 rad/s for true still; vibration varies by build/setup,
  // so tune per train. Heuristics, not universal constants.
  double m_stationaryGyroThresh = 0.15;
  // Typical 0.3-1.5 m/s^2; vibration varies by build/setup, so tune per train.
  // Heuristic values only.
  double m_stationaryAccelMagThresh = 0.7;
  // Typical 0.5-3.0s; longer reduces false stationary at the cost of latency.
  double m_stationaryHoldSeconds = 1.0;
  // Trade-off: longer for robustness, shorter for faster leveling.
  double m_kpBase = 1.2;
  // Higher KP corrects quicker but can jitter under vibration.
  double m_kiBase = 0.05;
  // KI fights bias but can wind up; keep small if accel is noisy.
  double m_mahonyIntegralLimit = 0.5;
  double m_biasTau = 2.0;
  double m_biasQ = 1e-4;
  double m_biasVarMin = 1e-5;
  double m_biasVarMax = 0.5;
  double m_ewmaTau = 2.0;
  double m_relevelRate = 0.5;
  double m_accelConfidenceRange = 1.0;
  double m_gravityLpTau = 1.5;
  double m_accelScaleNoise = 0.02;
  double m_gyroScaleNoise = 0.02;
  double m_tempScale = 0.02;
  double m_conductorStaleSeconds = 0.5;
  double m_dvdtThresh = 0.4;
  double m_alinThresh = 0.6;
  // Forward inference: minimum IMU response required to update score, tuned
  // for LEGO train vibration levels (heuristic; gyro magnitude threshold).
  double m_forwardResponseAccelThresh = 0.2;
  double m_forwardResponseGyroThresh = 0.12;
  double m_forwardLockSeconds = 3.0;
  double m_forwardScoreThresh = 0.5;
  double m_forwardDeadband = 0.08;
  int m_forwardSignConsistencySamples = 10;
  double m_forwardGyroVetoZ = 0.8;
  double m_forwardGyroVetoXY = 0.8;
  double m_yawInflateFactor = 2.0;
  double m_dtMin = 0.001;
  double m_dtMax = 0.2;
  std::string m_conductorStateTopic;
  std::string m_imuDebugTopic;
  std::string m_imuStatusTopic;
  std::string m_imuMappingDebugTopic;
};

} // namespace ROS
} // namespace OASIS
