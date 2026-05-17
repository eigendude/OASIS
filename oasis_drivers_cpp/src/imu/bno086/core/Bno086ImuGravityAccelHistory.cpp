/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "imu/bno086/core/Bno086ImuGravityAccelHistory.hpp"

#include <algorithm>

namespace OASIS::IMU::BNO086
{
void Bno086ImuGravityAccelHistory::Push(const Bno086ImuGravityAccelSample& sample)
{
  m_samples[m_next] = sample;
  m_next = (m_next + 1) % m_samples.size();
  m_count = std::min(m_count + 1, m_samples.size());
}

std::optional<Bno086ImuGravityAccelSample> Bno086ImuGravityAccelHistory::SelectAtOrBefore(
    int64_t anchor_stamp_ns, int64_t future_tolerance_ns) const
{
  std::optional<Bno086ImuGravityAccelSample> nearestPast;
  std::optional<Bno086ImuGravityAccelSample> nearestFuture;

  for (std::size_t i = 0; i < m_count; ++i)
  {
    const Bno086ImuGravityAccelSample& sample = m_samples[i];
    if (!sample.has_sample)
      continue;

    if (sample.stamp_ns <= anchor_stamp_ns)
    {
      if (!nearestPast.has_value() || sample.stamp_ns > nearestPast->stamp_ns)
        nearestPast = sample;
    }
    else if (!nearestFuture.has_value() || sample.stamp_ns < nearestFuture->stamp_ns)
    {
      nearestFuture = sample;
    }
  }

  if (nearestPast.has_value())
    return nearestPast;

  if (nearestFuture.has_value() && nearestFuture->stamp_ns - anchor_stamp_ns <= future_tolerance_ns)
    return nearestFuture;

  return std::nullopt;
}

void Bno086ImuGravityAccelHistory::Reset()
{
  m_samples = {};
  m_next = 0;
  m_count = 0;
}
} // namespace OASIS::IMU::BNO086
