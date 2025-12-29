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
#include <cstdint>

namespace OASIS::IMU
{

/**
 * @brief Online accelerometer bias/scale estimator using stationary magnitude residuals.
 *
 * The calibrator watches for stationary periods and nudges a diagonal
 * bias/scale model so the corrected acceleration magnitude matches gravity.
 */
class AccelCalibrator
{
public:
  /**
   * @brief Diagnostics snapshot emitted by Update().
   */
  struct Diagnostics
  {
    /// Estimated bias per axis (m/s^2). Applied additively to raw accel.
    std::array<double, 3> bias_mps2{0.0, 0.0, 0.0};
    /// Estimated scale per axis (unitless). Applied multiplicatively.
    std::array<double, 3> scale{1.0, 1.0, 1.0};
    /// True when gyro + accel jerk indicate the IMU is stationary.
    bool stationary{false};

    /// Coverage flags: true if gravity has pointed mostly in +axis (unit > +0.75).
    std::array<bool, 3> pos_seen{false, false, false};
    /// Coverage flags: true if gravity has pointed mostly in -axis (unit < -0.75).
    std::array<bool, 3> neg_seen{false, false, false};
  };

  AccelCalibrator();

  /**
   * @brief Update the estimator with a new IMU sample.
   *
   * @param accel_mps2 Linear acceleration (m/s^2) in sensor frame.
   * @param gyro_rads Angular velocity (rad/s) in sensor frame.
   * @param dt_seconds Sample interval (s). Non-positive values fall back to 0.02 s.
   * @return Diagnostic snapshot of the current estimator state.
   */
  Diagnostics Update(const std::array<double, 3>& accel_mps2,
                     const std::array<double, 3>& gyro_rads,
                     double dt_seconds);

  /// Latest bias estimate (m/s^2).
  const std::array<double, 3>& GetBias() const;
  /// Latest scale estimate (unitless).
  const std::array<double, 3>& GetScale() const;

  /**
   * @brief Reset uniform scale initialization state.
   *
   * Clears the stationary mean accumulator and allows the uniform scale
   * initialization to run again with new input scaling.
   */
  void ResetUniformScaleInitialization();

private:
  struct RunningMean
  {
    std::size_t n{0};
    double mean{0.0};

    void Add(double x);
    bool Ready(std::size_t min_n) const;
  };

  static double Norm3(const std::array<double, 3>& v);

  // State
  bool m_initialized{false};
  std::array<double, 3> m_accel_lp{0.0, 0.0, 0.0};
  std::array<double, 3> m_prev_accel_lp{0.0, 0.0, 0.0};

  std::array<double, 3> m_bias{0.0, 0.0, 0.0};
  std::array<double, 3> m_scale{1.0, 1.0, 1.0};

  bool m_uniform_scale_initialized{false};
  RunningMean m_stationary_norm_mean;

  std::array<bool, 3> m_pos_seen{false, false, false};
  std::array<bool, 3> m_neg_seen{false, false, false};
};

} // namespace OASIS::IMU
