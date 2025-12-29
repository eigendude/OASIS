/*
 *  Copyright (C) 2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#pragma once

#include <array>
#include <cstddef>

namespace OASIS
{
namespace IMU
{

struct GyroBiasCalibratorConfig
{
  //! Expected gravity magnitude used for stillness detection (m/s^2).
  double gravity_mps2{9.80665};
  //! Allowed accel magnitude deviation from gravity during stillness (m/s^2).
  double accel_mag_tolerance_mps2{0.25};
  //! Max accel magnitude jerk during stillness (m/s^3).
  double accel_jerk_threshold_mps3{1.0};
  //! Duration to discard samples at startup (s).
  double warmup_duration_s{0.5};
  //! Duration of continuous stillness required before calibration (s).
  double stillness_duration_s{0.5};
  //! Duration of gyro sample collection for bias (s).
  double calibrate_duration_s{2.0};
  //! Max allowed gyro stddev per axis to accept calibration (rad/s).
  double gyro_stddev_threshold_rads{0.01};
};

struct GyroBiasEstimate
{
  //! Mean gyro bias per axis in rad/s.
  std::array<double, 3> bias_rads{};
  //! Standard deviation per axis in rad/s.
  std::array<double, 3> stddev_rads{};
};

class GyroBiasCalibrator
{
public:
  enum class State
  {
    WARMUP,
    WAIT_STILL,
    CALIBRATE,
    READY
  };

  explicit GyroBiasCalibrator(const GyroBiasCalibratorConfig& config);

  void Reset();

  State GetState() const;
  const GyroBiasEstimate& GetEstimate() const;

  void Update(const std::array<double, 3>& accel_mps2,
              const std::array<double, 3>& gyro_rads,
              double dt_seconds);

private:
  void StartWarmup();
  void StartWaitStill();
  void StartCalibrate();
  void StartReady();

  GyroBiasCalibratorConfig m_config;
  GyroBiasEstimate m_estimate;
  State m_state{State::WARMUP};
  double m_warmupElapsed{0.0};
  double m_stillElapsed{0.0};
  double m_calibrateElapsed{0.0};
  bool m_hasPrevAccel{false};
  double m_prevAccelMag{0.0};
  std::array<double, 3> m_mean{};
  std::array<double, 3> m_m2{};
  std::size_t m_samples{0};
};

} // namespace IMU
} // namespace OASIS
