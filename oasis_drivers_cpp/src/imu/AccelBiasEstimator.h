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
class AccelBiasEstimator
{
public:
  struct Diagnostics
  {
    std::array<double, 3> bias_mps2{};
    std::array<double, 3> accel_lp_mps2{};
    double accel_norm_mps2{0.0};
    double accel_g{0.0};
    double g_err_mps2{0.0};
    double w_conf{1.0};
    double var_mag_mps4{0.0};
    double jerk_avg_mps3{0.0};
    double gyro_mag_avg_rads{0.0};
    std::array<bool, 3> axis_pos_seen{};
    std::array<bool, 3> axis_neg_seen{};
    bool stationary{false};
  };

  AccelBiasEstimator();

  const std::array<double, 3>& GetBias() const;
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
  std::array<double, 3> m_accel_lp_mps2{};
  std::array<double, 3> m_prev_accel_lp_mps2{};
  std::array<bool, 3> m_axis_pos_seen{};
  std::array<bool, 3> m_axis_neg_seen{};
  bool m_initialized{false};

  WindowStats m_accel_mag_window;
  WindowStats m_jerk_window;
  WindowStats m_gyro_mag_window;
};
} // namespace OASIS::IMU
