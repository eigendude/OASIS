/*
 *  Copyright (C) 2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "imu/ImuTemperature.h"

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace OASIS::IMU
{
namespace
{
// MPU6050 datasheet scale factor: degrees Celsius per TEMP_OUT LSB.
constexpr double kTempScale = 1.0 / 340.0;
// MPU6050 datasheet offset: degrees Celsius added after scaling.
constexpr double kTempOffsetC = 36.53;
// Nominal sampling interval used to normalize variance across dt jitter.
constexpr double kNominalDtS = 1.0 / 50.0;
// Minimum supported dt for variance rate normalization (prevents blow-up).
constexpr double kMinDtS = 1e-4;
// Maximum supported dt for variance rate normalization (prevents underflow).
constexpr double kMaxDtS = 1.0;
} // namespace

ImuTemperature::Sample ImuTemperature::ProcessRaw(int16_t tempRaw, double dt_s)
{
  Sample sample{};

  sample.temperature_c = static_cast<double>(tempRaw) * kTempScale + kTempOffsetC;

  if (m_count < 2)
  {
    if (m_count == 0)
    {
      m_r2 = tempRaw;
    }
    else
    {
      m_r1 = m_r2;
      m_r2 = tempRaw;
    }
    ++m_count;
    sample.variance_c2 = m_minVarianceC2;
    return sample;
  }

  m_r0 = m_r1;
  m_r1 = m_r2;
  m_r2 = tempRaw;

  const int32_t d2 = static_cast<int32_t>(m_r2) - 2 * static_cast<int32_t>(m_r1) +
                     static_cast<int32_t>(m_r0);

  // Denominator 6.0 derives from Var(d2) = 6 * sigma^2 for white measurement noise.
  const double var_counts2_instant = (static_cast<double>(d2) * static_cast<double>(d2)) / 6.0;

  const double dt = std::clamp(dt_s, kMinDtS, kMaxDtS);
  const double var_rate_counts2_per_s = var_counts2_instant / dt;
  const double var_counts2 = var_rate_counts2_per_s * kNominalDtS;
  const double var_c2 = var_counts2 * (kTempScale * kTempScale);

  sample.variance_c2 = std::max(var_c2, m_minVarianceC2);

  return sample;
}

void ImuTemperature::SetMinStdDev(double min_stddev_c)
{
  const double clamped_stddev = std::max(min_stddev_c, 0.0);
  m_minVarianceC2 = clamped_stddev * clamped_stddev;
}

void ImuTemperature::Reset()
{
  m_r0 = 0;
  m_r1 = 0;
  m_r2 = 0;
  m_count = 0;
}
} // namespace OASIS::IMU
