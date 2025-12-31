/*
 *  Copyright (C) 2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "imu/Mpu6050ImuProcessor.h"

#include <algorithm>
#include <cstddef>

using namespace OASIS::IMU;

namespace
{
constexpr double kMinDtSeconds = 1e-4;
constexpr double kMaxDtSeconds = 1.0;

// For white measurement noise on a uniform grid, Var(d2) = 6 * sigma^2.
constexpr double kUniformSecondDiffVarianceDenominator = 6.0;
} // namespace

void Mpu6050ImuProcessor::Reset()
{
  m_accelNoise.Reset();
  m_gyroNoise.Reset();
}

void Mpu6050ImuProcessor::NoiseEstimator::Reset()
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

std::array<double, 3> Mpu6050ImuProcessor::NoiseEstimator::Update(
    int16_t x, int16_t y, int16_t z, double dt_s)
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
    const double variance_denominator = use_uniform
                                            ? kUniformSecondDiffVarianceDenominator
                                            : (a * a + b * b + 1.0);
    std::array<double, 3> sigma2_counts{0.0, 0.0, 0.0};

    for (size_t axis = 0; axis < sample_counts.size(); ++axis)
    {
      const int32_t x2 = sample_counts[axis];
      const int32_t x1 = m_prev1_counts[axis];
      const int32_t x0 = m_prev2_counts[axis];
      const double d2 = use_uniform ? static_cast<double>(x2 - 2 * x1 + x0)
                                    : (static_cast<double>(x2) -
                                       static_cast<double>(x1) * b +
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

Mpu6050ImuProcessor::ProcessedSample Mpu6050ImuProcessor::ProcessRaw(
    int16_t ax, int16_t ay, int16_t az, int16_t gx, int16_t gy, int16_t gz, double dt_s)
{
  ProcessedSample sample{};

  sample.accel_raw_mps2 = {static_cast<double>(ax) * m_accelScale,
                           static_cast<double>(ay) * m_accelScale,
                           static_cast<double>(az) * m_accelScale};
  sample.gyro_rads = {static_cast<double>(gx) * m_gyroScale, static_cast<double>(gy) * m_gyroScale,
                      static_cast<double>(gz) * m_gyroScale};
  const auto accel_sigma2_counts = m_accelNoise.Update(ax, ay, az, dt_s);
  const auto gyro_sigma2_counts = m_gyroNoise.Update(gx, gy, gz, dt_s);
  const double accel_floor = (m_accelScale * m_accelScale) / 12.0;
  const double gyro_floor = (m_gyroScale * m_gyroScale) / 12.0;

  for (size_t axis = 0; axis < accel_sigma2_counts.size(); ++axis)
  {
    const double accel_variance = accel_sigma2_counts[axis] * (m_accelScale * m_accelScale);
    const double gyro_variance = gyro_sigma2_counts[axis] * (m_gyroScale * m_gyroScale);

    sample.accel_var_mps2_2[axis] = std::max(accel_variance, accel_floor);
    sample.gyro_var_rads2_2[axis] = std::max(gyro_variance, gyro_floor);
  }

  return sample;
}
