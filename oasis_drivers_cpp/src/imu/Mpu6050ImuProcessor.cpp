/*
 *  Copyright (C) 2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "imu/Mpu6050ImuProcessor.h"

#include <algorithm>

using namespace OASIS::IMU;

namespace
{
constexpr double kEwmaAlpha = 1.0 / 16.0;

// For white measurement noise, Var(d2) = 6 * sigma^2, so divide by 6 to
// recover per-sample variance from the second-difference residual.
constexpr double kSecondDiffVarianceDenominator = 6.0;

// Quantization error is uniform in [-0.5, 0.5] LSB, so the variance is 1/12
// in counts^2.
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
  m_sigma2_counts.fill(kQuantizationVarianceCounts);
  m_samples = 0;
}

std::array<double, 3> Mpu6050ImuProcessor::NoiseEstimator::Update(int16_t x, int16_t y, int16_t z)
{
  const std::array<int32_t, 3> sample_counts{static_cast<int32_t>(x), static_cast<int32_t>(y),
                                             static_cast<int32_t>(z)};

  if (m_samples >= 2)
  {
    for (size_t axis = 0; axis < sample_counts.size(); ++axis)
    {
      const int32_t d2 = sample_counts[axis] - 2 * m_prev1_counts[axis] + m_prev2_counts[axis];
      const double d2_value = static_cast<double>(d2);
      const double sigma2_instant = (d2_value * d2_value) / kSecondDiffVarianceDenominator;
      m_sigma2_counts[axis] =
          (1.0 - kEwmaAlpha) * m_sigma2_counts[axis] + kEwmaAlpha * sigma2_instant;
    }
  }

  m_prev2_counts = m_prev1_counts;
  m_prev1_counts = sample_counts;
  ++m_samples;

  return m_sigma2_counts;
}

Mpu6050ImuProcessor::ProcessedSample Mpu6050ImuProcessor::ProcessRaw(
    int16_t ax, int16_t ay, int16_t az, int16_t gx, int16_t gy, int16_t gz)
{
  ProcessedSample sample{};

  sample.accel_raw_mps2 = {static_cast<double>(ax) * m_accelScale,
                           static_cast<double>(ay) * m_accelScale,
                           static_cast<double>(az) * m_accelScale};
  sample.gyro_rads = {static_cast<double>(gx) * m_gyroScale, static_cast<double>(gy) * m_gyroScale,
                      static_cast<double>(gz) * m_gyroScale};
  const auto accel_sigma2_counts = m_accelNoise.Update(ax, ay, az);
  const auto gyro_sigma2_counts = m_gyroNoise.Update(gx, gy, gz);

  for (size_t axis = 0; axis < accel_sigma2_counts.size(); ++axis)
  {
    const double accel_analog_counts =
        std::max(accel_sigma2_counts[axis] - kQuantizationVarianceCounts, 0.0);
    const double gyro_analog_counts =
        std::max(gyro_sigma2_counts[axis] - kQuantizationVarianceCounts, 0.0);
    const double accel_total_counts = kQuantizationVarianceCounts + accel_analog_counts;
    const double gyro_total_counts = kQuantizationVarianceCounts + gyro_analog_counts;

    sample.accel_variance_mps2[axis] = accel_total_counts * (m_accelScale * m_accelScale);
    sample.gyro_variance_rads[axis] = gyro_total_counts * (m_gyroScale * m_gyroScale);
  }

  return sample;
}
