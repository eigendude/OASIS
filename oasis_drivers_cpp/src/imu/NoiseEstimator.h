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
 * Jitter-aware 2nd-difference estimator for i.i.d. measurement noise.
 *
 * Given samples x0, x1, x2 with time steps h1, h2, the non-uniform
 * second-difference residual is:
 *
 *   d2 = x2 - x1 * (h1 + h2) / h1 + x0 * h2 / h1.
 *
 * Define a = h2 / h1 and b = 1 + a. For independent noise with variance
 * sigma^2, Var(d2) = (a^2 + b^2 + 1) * sigma^2, so K(h1, h2) is
 *
 *   K = a^2 + b^2 + 1
 *
 * and
 *
 *   sigma^2 = d2^2 / (a^2 + b^2 + 1).
 *
 * If h1 is tiny, fall back to the uniform-grid formula:
 *
 *   d2 = x2 - 2 * x1 + x0,  sigma^2 = d2^2 / 6.
 */
class NoiseEstimator
{
public:
  void Reset();
  std::array<double, 3> Update(int16_t x, int16_t y, int16_t z, double dt_s);

private:
  static constexpr size_t kWindowSize = 16;

  std::array<int32_t, 3> m_prev1_counts{0, 0, 0};
  std::array<int32_t, 3> m_prev2_counts{0, 0, 0};
  std::array<std::array<double, kWindowSize>, 3> m_sigma2_window{};
  std::array<double, 3> m_sigma2_sum{0.0, 0.0, 0.0};
  double m_prev_dt_s{0.0};
  size_t m_sigma2_index{0};
  size_t m_sigma2_count{0};
  int m_samples{0};
};
} // namespace OASIS::IMU
