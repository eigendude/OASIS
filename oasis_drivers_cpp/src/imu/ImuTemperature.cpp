/*
 *  Copyright (C) 2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "imu/ImuTemperature.h"

namespace OASIS::IMU
{
namespace
{
constexpr double kTempScale = 1.0 / 340.0;
constexpr double kTempOffsetC = 36.53;
} // namespace

ImuTemperature::Sample ImuTemperature::ProcessRaw(int16_t tempRaw, double dt_s)
{
  (void)dt_s;
  Sample sample{};

  // MPU6050 datasheet formula: Temp(°C) = (TEMP_OUT / 340) + 36.53
  sample.temperature_c = static_cast<double>(tempRaw) * kTempScale + kTempOffsetC;

  if (!m_hasX1)
  {
    m_x1 = sample.temperature_c;
    m_hasX1 = true;
    sample.variance_c2 = 0.0;
    return sample;
  }

  if (!m_hasX2)
  {
    m_x2 = m_x1;
    m_x1 = sample.temperature_c;
    m_hasX2 = true;
    sample.variance_c2 = 0.0;
    return sample;
  }

  const double d2 = sample.temperature_c - 2.0 * m_x1 + m_x2;
  // Denominator 6.0 derives from Var(d2) = 6*sigma^2 for white measurement noise.
  sample.variance_c2 = (d2 * d2) / 6.0;

  m_x2 = m_x1;
  m_x1 = sample.temperature_c;

  return sample;
}

void ImuTemperature::Reset()
{
  m_hasX1 = false;
  m_hasX2 = false;
  m_x1 = 0.0;
  m_x2 = 0.0;
}
} // namespace OASIS::IMU
