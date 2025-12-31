/*
 *  Copyright (C) 2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "imu/Mpu6050ImuProcessor.h"

#include <algorithm>
#include <cmath>
#include <cstddef>

using namespace OASIS::IMU;

namespace
{
constexpr double kMinDtSeconds = 1e-4;
constexpr double kMaxDtSeconds = 1.0;

// Conservative thresholds for stationary gating (gyro rad/s, accel m/s^2).
constexpr double kGyroStationaryRadS = 0.1;
constexpr double kAccelStationaryBandMps2 = 0.5;

// Quantization variance for 1 LSB in counts^2.
constexpr double kQuantizationVarianceCounts = 1.0 / 12.0;
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
    int16_t x, int16_t y, int16_t z, double dt_s, bool is_stationary)
{
  const std::array<int32_t, 3> sample_counts{static_cast<int32_t>(x), static_cast<int32_t>(y),
                                             static_cast<int32_t>(z)};
  const double dt_clamped = std::clamp(dt_s, kMinDtSeconds, kMaxDtSeconds);
  const double prev_dt_clamped = std::clamp(m_prev_dt_s, kMinDtSeconds, kMaxDtSeconds);
  const bool has_valid_dt = dt_s > 0.0 && m_prev_dt_s > 0.0;

  if (m_samples >= 2 && has_valid_dt)
  {
    const double h1 = dt_clamped;
    const double h2 = prev_dt_clamped;
    const double variance_denominator = 2.0 * (1.0 / (h1 * h1) + 1.0 / (h2 * h2) + 1.0 / (h1 * h2));
    std::array<double, 3> sigma2_counts{0.0, 0.0, 0.0};

    for (size_t axis = 0; axis < sample_counts.size(); ++axis)
    {
      const int32_t x2 = sample_counts[axis];
      const int32_t x1 = m_prev1_counts[axis];
      const int32_t x0 = m_prev2_counts[axis];
      const double d = (static_cast<double>(x2 - x1) / h1) - (static_cast<double>(x1 - x0) / h2);
      sigma2_counts[axis] = (d * d) / variance_denominator;
    }

    if (is_stationary && m_sigma2_count < kWindowSize)
    {
      for (size_t axis = 0; axis < sigma2_counts.size(); ++axis)
      {
        m_sigma2_window[axis][m_sigma2_index] = sigma2_counts[axis];
        m_sigma2_sum[axis] += sigma2_counts[axis];
      }
      ++m_sigma2_count;
    }
    else if (is_stationary)
    {
      for (size_t axis = 0; axis < sigma2_counts.size(); ++axis)
      {
        m_sigma2_sum[axis] += sigma2_counts[axis] - m_sigma2_window[axis][m_sigma2_index];
        m_sigma2_window[axis][m_sigma2_index] = sigma2_counts[axis];
      }
    }
    if (is_stationary)
    {
      m_sigma2_index = (m_sigma2_index + 1) % kWindowSize;
    }
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
  const double accel_norm = std::sqrt(sample.accel_raw_mps2[0] * sample.accel_raw_mps2[0] +
                                      sample.accel_raw_mps2[1] * sample.accel_raw_mps2[1] +
                                      sample.accel_raw_mps2[2] * sample.accel_raw_mps2[2]);
  const double gyro_norm = std::sqrt(sample.gyro_rads[0] * sample.gyro_rads[0] +
                                     sample.gyro_rads[1] * sample.gyro_rads[1] +
                                     sample.gyro_rads[2] * sample.gyro_rads[2]);
  const bool is_stationary = gyro_norm < kGyroStationaryRadS &&
                             std::abs(accel_norm - m_gravity) < kAccelStationaryBandMps2;
  const auto accel_sigma2_counts = m_accelNoise.Update(ax, ay, az, dt_s, is_stationary);
  const auto gyro_sigma2_counts = m_gyroNoise.Update(gx, gy, gz, dt_s, is_stationary);

  for (size_t axis = 0; axis < accel_sigma2_counts.size(); ++axis)
  {
    const double accel_variance =
        (accel_sigma2_counts[axis] + kQuantizationVarianceCounts) * (m_accelScale * m_accelScale);
    const double gyro_variance =
        (gyro_sigma2_counts[axis] + kQuantizationVarianceCounts) * (m_gyroScale * m_gyroScale);

    sample.accel_var_mps2_2[axis] = accel_variance;
    sample.gyro_var_rads2_2[axis] = gyro_variance;
  }

  return sample;
}
