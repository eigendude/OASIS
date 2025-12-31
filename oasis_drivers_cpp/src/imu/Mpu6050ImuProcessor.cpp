/*
 *  Copyright (C) 2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "imu/Mpu6050ImuProcessor.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>

using namespace OASIS::IMU;

namespace
{
constexpr double kMinDtSeconds = 1e-4;
constexpr double kMaxDtSeconds = 1.0;

// For white measurement noise on a uniform grid, Var(d2) = 6 * sigma^2.
constexpr double kUniformSecondDiffVarianceDenominator = 6.0;

// Stationary gate for gyro bias estimation (rad/s).
constexpr double kGyroStillRadS = 0.10;

// Stationary gate for acceleration magnitude around gravity (m/s^2).
constexpr double kAccelStillBandMps2 = 0.50;

// Minimum stationary samples before the gyro bias is considered valid.
constexpr int kGyroBiasMinSamples = 50;

constexpr std::array<size_t, 3> kAxisMap{0, 1, 2};
constexpr std::array<double, 3> kAxisSign{1.0, 1.0, 1.0};

std::array<double, 3> RemapAxes(const std::array<double, 3>& sensor_axes)
{
  return {kAxisSign[0] * sensor_axes[kAxisMap[0]], kAxisSign[1] * sensor_axes[kAxisMap[1]],
          kAxisSign[2] * sensor_axes[kAxisMap[2]]};
}

std::array<double, 3> RemapVariances(const std::array<double, 3>& sensor_variances)
{
  return {sensor_variances[kAxisMap[0]], sensor_variances[kAxisMap[1]],
          sensor_variances[kAxisMap[2]]};
}

double VectorNorm(const std::array<double, 3>& values)
{
  return std::sqrt(values[0] * values[0] + values[1] * values[1] + values[2] * values[2]);
}
} // namespace

void Mpu6050ImuProcessor::Reset()
{
  m_accelNoise.Reset();
  m_gyroNoise.Reset();
  m_gyroBiasEstimator.Reset();
}

void Mpu6050ImuProcessor::GyroBiasEstimator::Reset()
{
  m_bias_rads.fill(0.0);
  m_stationary_samples = 0;
  m_valid = false;
}

void Mpu6050ImuProcessor::GyroBiasEstimator::Update(const std::array<double, 3>& gyro_rads,
                                                    const std::array<double, 3>& accel_mps2,
                                                    double gravity_mps2)
{
  if (m_valid)
  {
    return;
  }

  const double gyro_norm = VectorNorm(gyro_rads);
  const double accel_norm = VectorNorm(accel_mps2);
  const bool gyro_still = gyro_norm < kGyroStillRadS;
  const bool accel_still = std::abs(accel_norm - gravity_mps2) < kAccelStillBandMps2;

  if (!(gyro_still && accel_still))
  {
    return;
  }

  ++m_stationary_samples;
  const double sample_count = static_cast<double>(m_stationary_samples);
  for (size_t axis = 0; axis < m_bias_rads.size(); ++axis)
  {
    m_bias_rads[axis] += (gyro_rads[axis] - m_bias_rads[axis]) / sample_count;
  }

  if (m_stationary_samples >= kGyroBiasMinSamples)
  {
    m_valid = true;
  }
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

std::array<double, 3> Mpu6050ImuProcessor::NoiseEstimator::Update(int16_t x,
                                                                  int16_t y,
                                                                  int16_t z,
                                                                  double dt_s)
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

Mpu6050ImuProcessor::ProcessedOutputs Mpu6050ImuProcessor::ProcessRaw(
    int16_t ax, int16_t ay, int16_t az, int16_t gx, int16_t gy, int16_t gz, double dt_s)
{
  ProcessedOutputs outputs{};

  const std::array<double, 3> accel_sensor_mps2{static_cast<double>(ax) * m_accelScale,
                                                static_cast<double>(ay) * m_accelScale,
                                                static_cast<double>(az) * m_accelScale};
  const std::array<double, 3> gyro_sensor_rads{static_cast<double>(gx) * m_gyroScale,
                                               static_cast<double>(gy) * m_gyroScale,
                                               static_cast<double>(gz) * m_gyroScale};
  const std::array<double, 3> accel_body_mps2 = RemapAxes(accel_sensor_mps2);
  const std::array<double, 3> gyro_body_rads = RemapAxes(gyro_sensor_rads);

  const auto accel_sigma2_counts = m_accelNoise.Update(ax, ay, az, dt_s);
  const auto gyro_sigma2_counts = m_gyroNoise.Update(gx, gy, gz, dt_s);
  const double accel_floor = (m_accelScale * m_accelScale) / 12.0;
  const double gyro_floor = (m_gyroScale * m_gyroScale) / 12.0;
  std::array<double, 3> accel_var_sensor_mps2_2{0.0, 0.0, 0.0};
  std::array<double, 3> gyro_var_sensor_rads2_2{0.0, 0.0, 0.0};

  for (size_t axis = 0; axis < accel_sigma2_counts.size(); ++axis)
  {
    const double accel_variance = accel_sigma2_counts[axis] * (m_accelScale * m_accelScale);
    const double gyro_variance = gyro_sigma2_counts[axis] * (m_gyroScale * m_gyroScale);

    accel_var_sensor_mps2_2[axis] = std::max(accel_variance, accel_floor);
    gyro_var_sensor_rads2_2[axis] = std::max(gyro_variance, gyro_floor);
  }

  outputs.imu_raw.accel_mps2 = accel_body_mps2;
  outputs.imu_raw.gyro_rads = gyro_body_rads;
  outputs.imu_raw.accel_var_mps2_2 = RemapVariances(accel_var_sensor_mps2_2);
  outputs.imu_raw.gyro_var_rads2_2 = RemapVariances(gyro_var_sensor_rads2_2);

  m_gyroBiasEstimator.Update(gyro_body_rads, accel_body_mps2, m_gravity);
  outputs.gyro_bias_valid = m_gyroBiasEstimator.IsValid();

  std::array<double, 3> gyro_calibrated_rads = gyro_body_rads;
  if (outputs.gyro_bias_valid)
  {
    const auto& gyro_bias = m_gyroBiasEstimator.GetBias();
    for (size_t axis = 0; axis < gyro_calibrated_rads.size(); ++axis)
    {
      gyro_calibrated_rads[axis] -= gyro_bias[axis];
    }
  }

  outputs.imu.accel_mps2 = accel_body_mps2;
  outputs.imu.gyro_rads = gyro_calibrated_rads;
  outputs.imu.accel_var_mps2_2 = outputs.imu_raw.accel_var_mps2_2;
  outputs.imu.gyro_var_rads2_2 = outputs.imu_raw.gyro_var_rads2_2;

  return outputs;
}
