/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "imu/bno086/Bno086TimestampNormalizer.hpp"

#include <cstdint>

namespace OASIS::IMU::BNO086
{
namespace
{
// Maximum tolerated age of a reconstructed report stamp relative to the host
// packet arrival time. Older reconstructions are treated as a timestamp-domain
// reset and replaced with packet host time.
constexpr int64_t kMaxPastSkewNs = 100'000'000;

// Maximum tolerated future skew of a reconstructed report stamp relative to
// packet host time. Future-biased reconstructions are replaced with host time.
constexpr int64_t kMaxFutureSkewNs = 5'000'000;

bool HasSequenceGap(std::uint8_t last_sequence, std::uint8_t current_sequence)
{
  const std::uint8_t expected = static_cast<std::uint8_t>(last_sequence + 1U);
  return current_sequence != expected;
}
} // namespace

TimestampNormalizationResult Bno086TimestampNormalizer::Normalize(const TimestampSample& sample)
{
  TimestampNormalizationResult result;
  result.stamp_ns = sample.stamp_ns;

  const int64_t hostDeltaNs = sample.packet_host_stamp_ns - sample.stamp_ns;
  if (hostDeltaNs > kMaxPastSkewNs || hostDeltaNs < -kMaxFutureSkewNs)
  {
    result.stamp_ns = sample.packet_host_stamp_ns;
    result.reconstruction_reset = true;
  }

  if (m_hasLast)
  {
    if (sample.sequence == m_lastSequence && result.stamp_ns == m_lastStampNs)
    {
      result.duplicate = true;
      return result;
    }

    result.sequence_gap = HasSequenceGap(m_lastSequence, sample.sequence);

    if (result.stamp_ns <= m_lastStampNs)
    {
      result.stamp_ns = m_lastStampNs + 1;
      result.repaired_nonmonotonic = true;
    }
  }

  m_hasLast = true;
  m_lastSequence = sample.sequence;
  m_lastStampNs = result.stamp_ns;
  return result;
}

void Bno086TimestampNormalizer::Reset()
{
  m_hasLast = false;
  m_lastSequence = 0;
  m_lastStampNs = 0;
}
} // namespace OASIS::IMU::BNO086
