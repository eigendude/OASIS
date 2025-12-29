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
#include <deque>

namespace OASIS::IMU
{
// Accelerometer calibrator: diagonal scale + bias learned online from gravity
// magnitude under stationary gating.
// Online accelerometer calibrator using diagonal scale and bias, learned from
// gravity magnitude under stationary gating.
class AccelCalibrator
{
public:
  struct Diagnostics
  {
    //! Estimated accelerometer bias (m/s^2) per axis.
    std::array<double, 3> bias_mps2{};
    //! Estimated per-axis scale (unitless).
    std::array<double, 3> scale{};
    //! Low-pass filtered raw accelerometer sample (m/s^2).
    std::array<double, 3> accel_lp_mps2{};
    //! Low-pass filtered calibrated accelerometer sample (m/s^2).
    std::array<double, 3> accel_cal_lp_mps2{};
    //! Norm of the calibrated acceleration (m/s^2).
    double accel_norm_mps2{0.0};
    //! Calibrated acceleration norm in units of g (unitless).
    double accel_g{0.0};
    //! Gravity magnitude error after bias/scale (m/s^2).
    double g_err_mps2{0.0};
    //! Confidence weight derived from motion statistics (0-1).
    double w_conf{1.0};
    //! Magnitude variance of the low-pass acceleration (m/s^4).
    double var_mag_mps4{0.0};
    //! Mean jerk magnitude over the window (m/s^3).
    double jerk_avg_mps3{0.0};
    //! Mean gyro magnitude over the window (rad/s).
    double gyro_mag_avg_rads{0.0};
    //! Whether positive coverage has been observed for each axis.
    std::array<bool, 3> axis_pos_seen{};
    //! Whether negative coverage has been observed for each axis.
    std::array<bool, 3> axis_neg_seen{};
    //! True when the estimator considers the sample quasi-stationary.
    bool stationary{false};
    //! True when bias learning is active for this update.
    bool learn_bias{false};
    //! True when scale learning is active for this update.
    bool learn_scale{false};
  };

  AccelCalibrator();

  const std::array<double, 3>& GetBias() const;
  const std::array<double, 3>& GetScale() const;
  Diagnostics Update(const std::array<double, 3>& accel_mps2,
                     const std::array<double, 3>& gyro_rads,
                     double dt_seconds);

private:
  struct WindowStats
  {
    void AddSample(double value, std::size_t target_size);
    double Mean() const;
    double Variance() const;
    bool Empty() const;
    std::size_t Size() const;

    std::deque<double> samples;
  };

  void UpdateCoverage(const std::array<double, 3>& direction);

  std::array<double, 3> m_bias_mps2{};
  std::array<double, 3> m_scale{1.0, 1.0, 1.0};
  std::array<double, 3> m_accel_lp_mps2{};
  std::array<double, 3> m_prev_accel_lp_mps2{};
  std::array<bool, 3> m_axis_pos_seen{};
  std::array<bool, 3> m_axis_neg_seen{};
  bool m_initialized{false};
  bool m_scale_initialized{false};

  WindowStats m_accel_mag_window;
  WindowStats m_jerk_window;
  WindowStats m_gyro_mag_window;
};
} // namespace OASIS::IMU
