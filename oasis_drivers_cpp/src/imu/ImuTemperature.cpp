/*
 *  Copyright (C) 2025-2026 Garrett Brown
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

// Minimum supported dt for the second-derivative stencil (prevents blow-up).
constexpr double kMinDtS = 1e-4;

// Maximum supported dt for the second-derivative stencil (prevents underflow).
constexpr double kMaxDtS = 1.0;
} // namespace

ImuTemperature::Sample ImuTemperature::ProcessRaw(int16_t tempRaw, double dt_s)
{
  Sample sample{};

  sample.temperatureC = static_cast<double>(tempRaw) * kTempScale + kTempOffsetC;

  m_raw[m_rawIndex] = tempRaw;
  m_rawIndex = (m_rawIndex + 1) % kRawHistory;
  if (m_rawCount < kRawHistory)
  {
    ++m_rawCount;
  }

  const double dt = std::clamp(dt_s, kMinDtS, kMaxDtS);

  if (m_rawCount < 3 || !m_hasPrevDt)
  {
    sample.varianceC2 = m_minVarianceC2;
    m_prevDtS = dt;
    m_hasPrevDt = true;
    return sample;
  }

  const std::size_t idx2 = (m_rawIndex + kRawHistory - 1) % kRawHistory;
  const std::size_t idx1 = (m_rawIndex + kRawHistory - 2) % kRawHistory;
  const std::size_t idx0 = (m_rawIndex + kRawHistory - 3) % kRawHistory;

  const int32_t r2 = static_cast<int32_t>(m_raw[idx2]);
  const int32_t r1 = static_cast<int32_t>(m_raw[idx1]);
  const int32_t r0 = static_cast<int32_t>(m_raw[idx0]);

  const double h1 = m_prevDtS;
  const double h2 = dt;

  const double a = 2.0 / (h1 + h2);
  const double c0 = a * (1.0 / h1);
  const double c1 = -a * (1.0 / h1 + 1.0 / h2);
  const double c2 = a * (1.0 / h2);

  const double d2 =
      c0 * static_cast<double>(r0) + c1 * static_cast<double>(r1) + c2 * static_cast<double>(r2);

  const double denom = c0 * c0 + c1 * c1 + c2 * c2;

  // Denominator derives from Var(d2) for white measurement noise in counts and
  // reduces to 6.0 when h1 == h2.
  const double var_counts2_instant = (d2 * d2) / denom;

  if (m_varCount < kVarHistory)
  {
    m_varCounts2Instant[m_varIndex] = var_counts2_instant;
    m_varSumCounts2Instant += var_counts2_instant;
    ++m_varCount;
  }
  else
  {
    m_varSumCounts2Instant -= m_varCounts2Instant[m_varIndex];
    m_varCounts2Instant[m_varIndex] = var_counts2_instant;
    m_varSumCounts2Instant += var_counts2_instant;
  }
  m_varIndex = (m_varIndex + 1) % kVarHistory;

  const double mean_var_counts2_instant =
      (m_varCount > 0) ? (m_varSumCounts2Instant / static_cast<double>(m_varCount)) : 0.0;
  const double var_c2 = mean_var_counts2_instant * (kTempScale * kTempScale);

  sample.varianceC2 = std::max(var_c2, m_minVarianceC2);
  m_prevDtS = dt;
  m_hasPrevDt = true;

  return sample;
}

void ImuTemperature::SetMinStdDev(double min_stddev_c)
{
  const double clamped_stddev = std::max(min_stddev_c, 0.0);
  m_minVarianceC2 = clamped_stddev * clamped_stddev;
}

void ImuTemperature::Reset()
{
  m_raw.fill(0);
  m_varCounts2Instant.fill(0.0);
  m_rawIndex = 0;
  m_varIndex = 0;
  m_rawCount = 0;
  m_varCount = 0;
  m_varSumCounts2Instant = 0.0;
  m_hasPrevDt = false;
  m_prevDtS = 0.0;
}
} // namespace OASIS::IMU
