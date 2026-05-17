/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "imu/bno086/Bno086TimestampNormalizer.hpp"

#include <cstdint>
#include <optional>

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

std::uint8_t SequenceDelta(std::uint8_t last_sequence, std::uint8_t current_sequence)
{
  return static_cast<std::uint8_t>(current_sequence - last_sequence);
}
} // namespace

TimestampNormalizationResult Bno086TimestampNormalizer::Normalize(
    const TimestampSample& sample, std::optional<int64_t> expected_interval_ns)
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

    const std::uint8_t sequenceDelta = SequenceDelta(m_lastSequence, sample.sequence);
    result.sequence_gap = HasSequenceGap(m_lastSequence, sample.sequence);

    if (result.stamp_ns <= m_lastStampNs)
    {
      result.repaired_nonmonotonic = true;

      if (sequenceDelta > 0 && expected_interval_ns.has_value() && *expected_interval_ns > 0)
      {
        result.repaired_sequence_gap_to_interval = sequenceDelta > 1U;
        result.repaired_duplicate_to_interval =
            !result.repaired_sequence_gap_to_interval && result.stamp_ns == m_lastStampNs;
        result.repaired_nonmonotonic_to_interval =
            !result.repaired_sequence_gap_to_interval && result.stamp_ns < m_lastStampNs;

        const int64_t candidateStampNs = m_lastStampNs + (*expected_interval_ns * sequenceDelta);
        if (candidateStampNs <= sample.packet_host_stamp_ns)
        {
          result.stamp_ns = candidateStampNs;
        }
        else if (sample.packet_host_stamp_ns > m_lastStampNs)
        {
          result.stamp_ns = sample.packet_host_stamp_ns;
          result.interval_repair_clamped_to_host = true;
        }
        else
        {
          result.stamp_ns = m_lastStampNs + 1;
          result.interval_repair_bounded_to_legacy = true;
        }
      }
      else
      {
        result.stamp_ns = m_lastStampNs + 1;
      }
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
