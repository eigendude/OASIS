/*
 *  Copyright (C) 2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "imu/GyroBiasEstimator.h"

#include <cmath>

namespace OASIS::IMU
{
namespace
{
// Stationary gate for gyro bias estimation (rad/s).
constexpr double kGyroStillRadS = 0.10;

// Stationary gate for acceleration magnitude around gravity (m/s^2).
constexpr double kAccelStillBandMps2 = 0.50;

// Minimum stationary samples before the gyro bias is considered valid.
constexpr int kGyroBiasMinSamples = 50;

double VectorNorm(const std::array<double, 3>& values)
{
  return std::sqrt(values[0] * values[0] + values[1] * values[1] + values[2] * values[2]);
}
} // namespace

void GyroBiasEstimator::Reset()
{
  m_bias_rads.fill(0.0);
  m_stationary_samples = 0;
  m_valid = false;
}

void GyroBiasEstimator::Update(const std::array<double, 3>& gyro_rads,
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
} // namespace OASIS::IMU
