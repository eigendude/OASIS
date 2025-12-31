/*
 *  Copyright (C) 2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "imu/NoiseEstimator.h"

#include <algorithm>

namespace OASIS::IMU
{
namespace
{
constexpr double kMinDtSeconds = 1e-4;
constexpr double kMaxDtSeconds = 1.0;

// For white measurement noise on a uniform grid, Var(d2) = 6 * sigma^2.
constexpr double kUniformSecondDiffVarianceDenominator = 6.0;
} // namespace

void NoiseEstimator::Reset()
{
  m_prev1_counts.fill(0);
  m_prev2_counts.fill(0);
  for (auto& axis_window : m_sigma2_window)
  {
    axis_window.fill(0.0);
  }
  m_sigma2_sum.fill(0.0);
  m_prev_dt_s = 0.0;
  m_sigma2_index = 0;
  m_sigma2_count = 0;
  m_samples = 0;
}

std::array<double, 3> NoiseEstimator::Update(int16_t x, int16_t y, int16_t z, double dt_s)
{
  const std::array<int32_t, 3> sample_counts{static_cast<int32_t>(x), static_cast<int32_t>(y),
                                             static_cast<int32_t>(z)};
  const double dt_clamped = std::clamp(dt_s, kMinDtSeconds, kMaxDtSeconds);
  const double prev_dt_clamped = std::clamp(m_prev_dt_s, kMinDtSeconds, kMaxDtSeconds);

  if (m_samples >= 2)
  {
    const bool use_uniform = m_prev_dt_s <= kMinDtSeconds;
    const double a = use_uniform ? 1.0 : (dt_clamped / prev_dt_clamped);
    const double b = use_uniform ? 2.0 : (1.0 + a);
    const double variance_denominator =
        use_uniform ? kUniformSecondDiffVarianceDenominator : (a * a + b * b + 1.0);
    std::array<double, 3> sigma2_counts{0.0, 0.0, 0.0};

    for (size_t axis = 0; axis < sample_counts.size(); ++axis)
    {
      const int32_t x2 = sample_counts[axis];
      const int32_t x1 = m_prev1_counts[axis];
      const int32_t x0 = m_prev2_counts[axis];
      const double d2 = use_uniform ? static_cast<double>(x2 - 2 * x1 + x0)
                                    : (static_cast<double>(x2) - static_cast<double>(x1) * b +
                                       static_cast<double>(x0) * a);
      sigma2_counts[axis] = (d2 * d2) / variance_denominator;
    }

    if (m_sigma2_count < kWindowSize)
    {
      for (size_t axis = 0; axis < sigma2_counts.size(); ++axis)
      {
        m_sigma2_window[axis][m_sigma2_index] = sigma2_counts[axis];
        m_sigma2_sum[axis] += sigma2_counts[axis];
      }
      ++m_sigma2_count;
    }
    else
    {
      for (size_t axis = 0; axis < sigma2_counts.size(); ++axis)
      {
        m_sigma2_sum[axis] += sigma2_counts[axis] - m_sigma2_window[axis][m_sigma2_index];
        m_sigma2_window[axis][m_sigma2_index] = sigma2_counts[axis];
      }
    }
    m_sigma2_index = (m_sigma2_index + 1) % kWindowSize;
  }

  m_prev2_counts = m_prev1_counts;
  m_prev1_counts = sample_counts;
  m_prev_dt_s = dt_s;
  ++m_samples;

  std::array<double, 3> sigma2_mean{0.0, 0.0, 0.0};
  if (m_sigma2_count > 0)
  {
    for (size_t axis = 0; axis < sigma2_mean.size(); ++axis)
    {
      sigma2_mean[axis] = m_sigma2_sum[axis] / static_cast<double>(m_sigma2_count);
    }
  }

  return sigma2_mean;
}
} // namespace OASIS::IMU
