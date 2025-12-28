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
    // Gyro magnitude (rad/s) below which the IMU is considered stationary.
    double stationary_gyro_thresh = 0.15;
    // Accel magnitude deviation (m/s^2) threshold for stationary detection.
    double stationary_accel_mag_thresh = 0.7;
    // Time (s) that stationary detection must hold before latching.
    double stationary_hold_seconds = 1.0;
    // Mahony filter proportional gain for correcting attitude drift.
    double mahony_kp = 1.2;
    // Mahony filter integral gain for bias correction.
    double mahony_ki = 0.05;
    // Mahony integral windup limit.
    double mahony_integral_limit = 0.5;
    // Time constant (s) for gyro bias convergence.
    double bias_tau = 2.0;
    // Process noise for gyro bias random walk.
    double bias_q = 1e-4;
    // Minimum gyro bias variance.
    double bias_var_min = 1e-5;
    // Maximum gyro bias variance.
    double bias_var_max = 0.5;
    // EWMA time constant (s) for motor intent filtering.
    double ewma_tau = 2.0;
    // Rate for releveling Z-up orientation when stationary.
    double relevel_rate = 0.5;
    // Range (m/s^2) where accelerometer is considered reliable for leveling.
    double accel_confidence_range = 1.0;
    // Low-pass filter time constant (s) for gravity vector.
    double gravity_lp_tau = 1.5;
    // Accelerometer scale noise standard deviation.
    double accel_scale_noise = 0.02;
    // Gyroscope scale noise standard deviation.
    double gyro_scale_noise = 0.02;
    // Temperature scale noise standard deviation.
    double temp_scale = 0.02;
    // Motor intent derivative threshold for detecting acceleration events.
    double dvdt_thresh = 0.4;
    // Linear acceleration threshold (m/s^2) for motor intent detection.
    double alin_thresh = 0.6;
    // Minimum accelerometer response (m/s^2) to update forward inference score.
    double forward_response_accel_thresh = 0.2;
    // Minimum gyroscope response (rad/s) to update forward inference score.
    double forward_response_gyro_thresh = 0.12;
    // Time (s) to lock forward inference once a direction is chosen.
    double forward_lock_seconds = 3.0;
    // Score threshold to accept forward inference.
    double forward_score_thresh = 0.5;
    // Deadband for forward inference score updates.
    double forward_deadband = 0.08;
    // Samples required to confirm forward sign consistency.
    int forward_sign_consistency_samples = 10;
    // Veto threshold for forward inference based on Z-axis gyro magnitude.
    double forward_gyro_veto_z = 0.8;
    // Veto threshold for forward inference based on XY gyro magnitude.
    double forward_gyro_veto_xy = 0.8;
    // Factor to inflate yaw covariance relative to roll/pitch.
    double yaw_inflate_factor = 2.0;
    // Minimum delta time (s) allowed for filter updates.
    double dt_min = 0.001;
    // Maximum delta time (s) allowed for filter updates.
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
  Vec3 m_rollPitchVar{0.0, 0.0, 0.0};
};
} // namespace OASIS::IMU
