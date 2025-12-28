/*
 *  Copyright (C) 2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#pragma once

#include <array>
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
  using Vec3 = std::array<double, 3>;
  using Mapping = std::array<std::array<int, 3>, 3>;
  using Quaternion = std::array<double, 4>;

  Mpu6050Node();

  bool Initialize();
  void Deinitialize();

private:
  void PublishImu();
  void OnConductorState(const oasis_msgs::msg::ConductorState& msg);

  // ROS parameters
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr m_publisher;
  rclcpp::Subscription<oasis_msgs::msg::ConductorState>::SharedPtr m_conductorStateSub;
  rclcpp::TimerBase::SharedPtr m_timer;

  // IMU parameters
  std::unique_ptr<MPU6050> m_mpu6050;
  std::string m_i2cDevice;
  std::chrono::duration<double> m_publishPeriod;

  // IMU state
  double m_accelScale = 0.0;
  double m_gyroScale = 0.0;

  // Conductor state
  double m_motorVoltage = 0.0;
  double m_motorVoltageDvdt = 0.0;
  double m_motorVoltageFilt = 0.0;
  double m_motorVoltageFiltDvdt = 0.0;
  bool m_motorVoltageFiltInit = false;
  rclcpp::Time m_lastMotorStamp;

  // Filter state
  rclcpp::Time m_lastStamp;
  Vec3 m_gyroBias{0.0, 0.0, 0.0};
  Vec3 m_gyroBiasVar{0.01, 0.01, 0.01};
  Vec3 m_mahonyIntegral{0.0, 0.0, 0.0};
  Quaternion m_orientationQuat{1.0, 0.0, 0.0, 0.0};
  double m_yawVar = 0.5;
  bool m_covarianceInitialized = false;
  Vec3 m_gravityLpBody{0.0, 0.0, 0.0};
  bool m_gravityLpInit = false;

  // Stationary detection
  bool m_isStationary = false;
  double m_stationaryDuration = 0.0;
  double m_stationaryGoodDuration = 0.0;

  // Axis mapping
  bool m_zUpSolved = false;
  bool m_forwardSolved = false;
  Mapping m_zUpMapping{{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}};
  Mapping m_activeMapping{{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}};

  // Forward inference
  double m_forwardScoreX = 0.0;
  double m_forwardScoreY = 0.0;
  double m_forwardDominanceDuration = 0.0;
  int m_forwardDominantAxis = 0;

  // Temperature tracking
  bool m_hasTemperature = false;
  double m_lastTemperature = 0.0;
  double m_temperatureAtCal = 0.0;

  // Noise estimates
  Vec3 m_accelVar{0.2, 0.2, 0.2};
  Vec3 m_gyroVar{0.01, 0.01, 0.01};
  double m_rollVar = 0.1;
  double m_pitchVar = 0.1;

  // Parameters
  // Commanded intent threshold; motor voltage can change while a heavy train
  // stays still, so dv/dt is not motion truth.
  double m_stationaryVoltageThresh = 0.3;
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
  // Low-pass tau for commanded motor voltage intent filtering.
  double m_motorVoltageLpTau = 0.2;
  double m_dvdtThresh = 0.4;
  double m_alinThresh = 0.6;
  // Forward inference: minimum IMU response required to update score, tuned
  // for LEGO train vibration levels (heuristic; gyro magnitude threshold).
  double m_forwardResponseAccelThresh = 0.2;
  double m_forwardResponseGyroThresh = 0.12;
  double m_forwardLockSeconds = 3.0;
  double m_forwardScoreThresh = 0.5;
  double m_yawInflateFactor = 2.0;
  double m_dtMin = 0.001;
  double m_dtMax = 0.2;
  std::string m_conductorStateTopic;
};

} // namespace ROS
} // namespace OASIS
