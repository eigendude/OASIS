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

namespace OASIS::IMU
{
/**
 * Estimate a static gyroscope bias from stationary samples.
 *
 * The estimator gates samples using gyro magnitude and the difference between
 * acceleration norm and gravity. Once enough stationary samples are collected,
 * the bias is marked valid and no longer updated.
 */
class GyroBiasEstimator
{
public:
  /**
   * Clear bias state and sample counters.
   */
  void Reset();

  /**
   * Update the estimator with the latest motion sample.
   *
   * \param gyro_rads Angular velocity in rad/s in the body frame.
   * \param accel_mps2 Linear acceleration in m/s^2 in the body frame.
   * \param gravity_mps2 Gravity magnitude in m/s^2 used for stationarity.
   */
  void Update(const std::array<double, 3>& gyro_rads,
              const std::array<double, 3>& accel_mps2,
              double gravity_mps2);

  /**
   * Return the bias estimate in rad/s.
   */
  const std::array<double, 3>& GetBias() const { return m_bias_rads; }

  /**
   * True once the bias has converged with enough stationary samples.
   */
  bool IsValid() const { return m_valid; }

private:
  // Bias estimate in rad/s for each axis in the body frame.
  std::array<double, 3> m_bias_rads{0.0, 0.0, 0.0};

  // Count of stationary samples used for bias convergence.
  int m_stationary_samples{0};

  // True once the minimum number of stationary samples is reached.
  bool m_valid{false};
};
} // namespace OASIS::IMU
