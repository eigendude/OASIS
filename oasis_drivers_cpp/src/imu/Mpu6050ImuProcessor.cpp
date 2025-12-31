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

// Conservative thresholds for stationary gating.
constexpr double kGyroStillRadS = 0.1;        // rad/s gyro norm threshold.
constexpr double kAccelStillBandMps2 = 0.5;   // m/s^2 band around |g|.

// Minimum stationary samples before accepting a gyro bias estimate.
constexpr int kGyroBiasMinSamples = 50;

// Quantization variance for 1 LSB in counts^2.
constexpr double kQuantizationVarianceCounts = 1.0 / 12.0;
} // namespace

void Mpu6050ImuProcessor::Reset()
{
  m_accelNoise.Reset();
  m_gyroNoise.Reset();
  m_gyroBias.Reset();
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

void Mpu6050ImuProcessor::GyroBiasEstimator::Reset()
{
  m_bias_sum.fill(0.0);
  m_bias_rads.fill(0.0);
  m_stationary_samples = 0;
  m_bias_valid = false;
}

void Mpu6050ImuProcessor::GyroBiasEstimator::Update(
    const std::array<double, 3>& gyro_rads, bool is_stationary)
{
  if (m_bias_valid || !is_stationary)
  {
    return;
  }

  for (size_t axis = 0; axis < gyro_rads.size(); ++axis)
  {
    m_bias_sum[axis] += gyro_rads[axis];
  }
  ++m_stationary_samples;

  if (m_stationary_samples >= kGyroBiasMinSamples)
  {
    for (size_t axis = 0; axis < gyro_rads.size(); ++axis)
    {
      m_bias_rads[axis] = m_bias_sum[axis] / static_cast<double>(m_stationary_samples);
    }
    m_bias_valid = true;
  }
}

std::array<double, 3> Mpu6050ImuProcessor::RemapToImuLink(
    const std::array<double, 3>& sample)
{
  // Identity remap for current hardware; keep helper for future axis flips.
  return sample;
}

Mpu6050ImuProcessor::ProcessedSamples Mpu6050ImuProcessor::ProcessRaw(
    int16_t ax, int16_t ay, int16_t az, int16_t gx, int16_t gy, int16_t gz, double dt_s)
{
  ProcessedSamples sample{};

  const std::array<double, 3> accel_mps2 = RemapToImuLink(
      {static_cast<double>(ax) * m_accelScale, static_cast<double>(ay) * m_accelScale,
       static_cast<double>(az) * m_accelScale});
  const std::array<double, 3> gyro_rads = RemapToImuLink(
      {static_cast<double>(gx) * m_gyroScale, static_cast<double>(gy) * m_gyroScale,
       static_cast<double>(gz) * m_gyroScale});
  const double accel_norm = std::sqrt(accel_mps2[0] * accel_mps2[0] +
                                      accel_mps2[1] * accel_mps2[1] +
                                      accel_mps2[2] * accel_mps2[2]);
  const double gyro_norm = std::sqrt(gyro_rads[0] * gyro_rads[0] + gyro_rads[1] * gyro_rads[1] +
                                     gyro_rads[2] * gyro_rads[2]);
  const bool is_stationary = gyro_norm < kGyroStillRadS &&
                             std::abs(accel_norm - m_gravity) < kAccelStillBandMps2;
  const auto accel_sigma2_counts = m_accelNoise.Update(ax, ay, az, dt_s, is_stationary);
  const auto gyro_sigma2_counts = m_gyroNoise.Update(gx, gy, gz, dt_s, is_stationary);
  m_gyroBias.Update(gyro_rads, is_stationary);

  for (size_t axis = 0; axis < accel_sigma2_counts.size(); ++axis)
  {
    const double accel_variance =
        (accel_sigma2_counts[axis] + kQuantizationVarianceCounts) * (m_accelScale * m_accelScale);
    const double gyro_variance =
        (gyro_sigma2_counts[axis] + kQuantizationVarianceCounts) * (m_gyroScale * m_gyroScale);

    sample.imu_raw.accel_var_mps2_2[axis] = accel_variance;
    sample.imu_raw.gyro_var_rads2_2[axis] = gyro_variance;
    sample.imu.accel_var_mps2_2[axis] = accel_variance;
    sample.imu.gyro_var_rads2_2[axis] = gyro_variance;
  }

  sample.imu_raw.accel_mps2 = accel_mps2;
  sample.imu_raw.gyro_rads = gyro_rads;
  sample.imu.accel_mps2 = accel_mps2;
  sample.imu.gyro_rads = gyro_rads;
  sample.gyro_bias_rads = m_gyroBias.GetBias();
  sample.gyro_bias_valid = m_gyroBias.IsValid();

  if (sample.gyro_bias_valid)
  {
    for (size_t axis = 0; axis < sample.imu.gyro_rads.size(); ++axis)
    {
      sample.imu.gyro_rads[axis] -= sample.gyro_bias_rads[axis];
    }
  }

  return sample;
}
