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

ImuTemperature::Sample ImuTemperature::ProcessRaw(int16_t tempRaw)
{
  Sample sample{};

  // MPU6050 datasheet formula: Temp(°C) = (TEMP_OUT / 340) + 36.53
  sample.temperature_c = static_cast<double>(tempRaw) * kTempScale + kTempOffsetC;

  ++m_sampleCount;
  const double delta = sample.temperature_c - m_meanC;
  m_meanC += delta / static_cast<double>(m_sampleCount);
  const double delta2 = sample.temperature_c - m_meanC;
  m_m2 += delta * delta2;

  sample.variance_c2 = (m_sampleCount > 1) ? (m_m2 / static_cast<double>(m_sampleCount - 1)) : 0.0;

  return sample;
}

void ImuTemperature::Reset()
{
  m_sampleCount = 0;
  m_meanC = 0.0;
  m_m2 = 0.0;
}
} // namespace OASIS::IMU
