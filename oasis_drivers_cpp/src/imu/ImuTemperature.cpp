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

namespace OASIS::IMU
{
namespace
{
constexpr double kTempScale = 1.0 / 340.0;
constexpr double kTempOffsetC = 36.53;
} // namespace

ImuTemperature::Sample ImuTemperature::ProcessRaw(int16_t tempRaw, double dt_s)
{
  Sample sample{};

  // MPU6050 datasheet formula: Temp(°C) = (TEMP_OUT / 340) + 36.53
  sample.temperature_c = static_cast<double>(tempRaw) * kTempScale + kTempOffsetC;

  if (!m_hasSample)
  {
    m_meanC = sample.temperature_c;
    m_varianceC2 = 0.0;
    m_hasSample = true;
    sample.variance_c2 = m_varianceC2;
    return sample;
  }

  const double clampedDt = std::max(dt_s, 0.0);
  const double alpha = (m_timeConstantS > 0.0 && clampedDt > 0.0)
                           ? (1.0 - std::exp(-clampedDt / m_timeConstantS))
                           : 0.0;
  const double delta = sample.temperature_c - m_meanC;

  m_meanC += alpha * delta;
  m_varianceC2 = (1.0 - alpha) * (m_varianceC2 + alpha * delta * delta);
  sample.variance_c2 = m_varianceC2;

  return sample;
}

void ImuTemperature::Reset()
{
  m_meanC = 0.0;
  m_varianceC2 = 0.0;
  m_hasSample = false;
}

void ImuTemperature::SetTimeConstant(double time_constant_s)
{
  m_timeConstantS = std::max(time_constant_s, 0.0);
}
} // namespace OASIS::IMU
