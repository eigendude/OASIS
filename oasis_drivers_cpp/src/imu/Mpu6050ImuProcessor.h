/*
 *  Copyright (C) 2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#pragma once

#include "nodes/Mpu6050NodeUtils.h"

#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace OASIS::IMU
{
class Mpu6050ImuProcessor
{
public:
  using Vec3 = OASIS::ROS::Mpu6050NodeUtils::Vec3;
  using Mapping = OASIS::ROS::Mpu6050NodeUtils::Mapping;
  using Quaternion = OASIS::ROS::Mpu6050NodeUtils::Quaternion;

  struct Config
  {
    double stationary_gyro_thresh = 0.15;
    double stationary_accel_mag_thresh = 0.7;
    double stationary_hold_seconds = 1.0;
    double mahony_kp = 1.2;
    double mahony_ki = 0.05;
    double mahony_integral_limit = 0.5;
    double bias_tau = 2.0;
    double bias_q = 1e-4;
    double bias_var_min = 1e-5;
    double bias_var_max = 0.5;
    double ewma_tau = 2.0;
    double relevel_rate = 0.5;
    double accel_confidence_range = 1.0;
    double gravity_lp_tau = 1.5;
    double accel_scale_noise = 0.02;
    double gyro_scale_noise = 0.02;
    double temp_scale = 0.02;
    double dvdt_thresh = 0.4;
    double alin_thresh = 0.6;
    double forward_response_accel_thresh = 0.2;
    double forward_response_gyro_thresh = 0.12;
    double forward_lock_seconds = 3.0;
    double forward_score_thresh = 0.5;
    double forward_deadband = 0.08;
    int forward_sign_consistency_samples = 10;
    double forward_gyro_veto_z = 0.8;
    double forward_gyro_veto_xy = 0.8;
    double yaw_inflate_factor = 2.0;
    double dt_min = 0.001;
    double dt_max = 0.2;
  };

  struct MotorIntentInput
  {
    double duty_raw = 0.0;
    double duty_filt = 0.0;
    double duty_dvdt = 0.0;
    bool fresh = false;
    bool commanded = false;
    double sign = 1.0;
  };

  struct RawSample
  {
    int16_t ax = 0;
    int16_t ay = 0;
    int16_t az = 0;
    int16_t gx = 0;
    int16_t gy = 0;
    int16_t gz = 0;
    std::optional<int16_t> temp_raw;
    double timestamp = 0.0;
  };

  struct Output
  {
    Vec3 linear_acceleration{0.0, 0.0, 0.0};
    Vec3 angular_velocity{0.0, 0.0, 0.0};
    Quaternion orientation{1.0, 0.0, 0.0, 0.0};
    std::array<double, 9> orientation_covariance{};
    std::array<double, 9> angular_velocity_covariance{};
    std::array<double, 9> linear_acceleration_covariance{};
    bool forward_solved = false;
    Mapping active_mapping{{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}};
    std::vector<double> debug_values;
    std::optional<std::string> mapping_jump_reason;
    std::vector<std::string> info_messages;
  };

  explicit Mpu6050ImuProcessor(Config config);

  void SetAccelScale(double accel_scale);
  void SetGyroScale(double gyro_scale);

  std::optional<Output> Process(const RawSample& sample, const MotorIntentInput& motor);

private:
  void RegisterMappingJump(const char* reason, Output& output);

  Config m_config;
  double m_accelScale = 0.0;
  double m_gyroScale = 0.0;

  double m_lastTimestamp = 0.0;
  Vec3 m_gyroBias{0.0, 0.0, 0.0};
  Vec3 m_gyroBiasVar{0.01, 0.01, 0.01};
  Vec3 m_mahonyIntegral{0.0, 0.0, 0.0};
  Quaternion m_orientationQuat{1.0, 0.0, 0.0, 0.0};
  double m_yawVar = 0.5;
  bool m_covarianceInitialized = false;
  Vec3 m_gravityLpBody{0.0, 0.0, 0.0};
  bool m_gravityLpInit = false;

  bool m_isStationary = false;
  double m_stationaryDuration = 0.0;
  double m_stationaryGoodDuration = 0.0;

  bool m_zUpSolved = false;
  bool m_forwardSolved = false;
  Mapping m_zUpMapping{{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}};
  Mapping m_activeMapping{{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}};

  double m_forwardScoreX = 0.0;
  double m_forwardScoreY = 0.0;
  double m_forwardEnergyX = 0.0;
  double m_forwardEnergyY = 0.0;
  double m_forwardDominanceDuration = 0.0;
  int m_forwardDominantAxis = 0;
  int m_forwardSignConsistencyCount = 0;
  int m_forwardSignLast = 0;

  bool m_hasTemperature = false;
  double m_lastTemperature = 0.0;
  double m_temperatureAtCal = 0.0;

  Vec3 m_accelVar{0.2, 0.2, 0.2};
  Vec3 m_gyroVar{0.01, 0.01, 0.01};
  double m_rollVar = 0.1;
  double m_pitchVar = 0.1;

  std::vector<Mapping> m_mappings;
  Vec3 m_accelMean{0.0, 0.0, 0.0};
  bool m_accelMeanInit = false;

  std::array<bool, 3> m_gyroVarInit{{false, false, false}};
  std::array<bool, 3> m_accelVarInit{{false, false, false}};
  std::array<bool, 2> m_orientVarInit{{false, false}};
  bool m_accelMeanStationaryInit = false;
  Vec3 m_gyroMean{0.0, 0.0, 0.0};
  Vec3 m_accelMeanStationary{0.0, 0.0, 0.0};
  Vec3 m_accelResidualMean{0.0, 0.0, 0.0};
  Vec3 m_rollPitchMean{0.0, 0.0, 0.0};
  Vec3 m_gyroVar{0.0, 0.0, 0.0};
  Vec3 m_accelVar{0.0, 0.0, 0.0};
  Vec3 m_rollPitchVar{0.0, 0.0, 0.0};
};
} // namespace OASIS::IMU
