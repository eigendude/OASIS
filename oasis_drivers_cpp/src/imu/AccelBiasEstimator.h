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
    //! Estimated accelerometer bias, meters per second^2 (typically a few
    //! m/s^2), inferred from stationary windows.
    std::array<double, 3> bias_mps2{};
    //! Low-pass filtered acceleration, meters per second^2 (near gravity at
    //! rest), used for bias estimation.
    std::array<double, 3> accel_lp_mps2{};
    //! Magnitude of filtered acceleration, meters per second^2 (near 9.81 when
    //! stationary).
    double accel_norm_mps2{0.0};
    //! Acceleration magnitude in units of g, expected ~1.0 at rest.
    double accel_g{0.0};
    //! Difference between accel_norm_mps2 and 1 g, meters per second^2 (near 0
    //! when stationary).
    double g_err_mps2{0.0};
    //! Confidence weight (0-1) based on stationarity checks (higher means more
    //! reliable).
    double w_conf{1.0};
    //! Variance of acceleration magnitude, (m/s^2)^2 (near 0 when stationary).
    double var_mag_mps4{0.0};
    //! Average jerk magnitude, meters per second^3 (near 0 when stationary),
    //! from filtered acceleration.
    double jerk_avg_mps3{0.0};
    //! Average gyro magnitude, radians per second (near 0 at rest).
    double gyro_mag_avg_rads{0.0};
    //! Flags indicating positive axis coverage during calibration (true once
    //! axis seen).
    std::array<bool, 3> axis_pos_seen{};
    //! Flags indicating negative axis coverage during calibration (true once
    //! axis seen).
    std::array<bool, 3> axis_neg_seen{};
    //! True when acceleration/gyro metrics indicate stationary conditions.
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
