/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "imu/bno086/Bno086TimestampNormalizer.hpp"

#include <algorithm>
#include <cstdint>

namespace OASIS::IMU::BNO086
{
TimestampNormalizationResult NormalizeReportTimestamp(ReportTimestampState& state,
                                                      std::uint8_t sequence,
                                                      std::int64_t raw_stamp_ns,
                                                      const TimestampNormalizationConfig& config)
{
  (void)config;

  TimestampNormalizationResult result;
  result.stamp_ns = raw_stamp_ns;

  if (!state.has_sample)
  {
    state.has_sample = true;
    state.last_sequence = sequence;
    state.last_stamp_ns = raw_stamp_ns;
    return result;
  }

  const std::int64_t lastStampNs = state.last_stamp_ns;
  const std::uint8_t sequenceDelta = static_cast<std::uint8_t>(sequence - state.last_sequence);

  result.sequence_delta = sequenceDelta;
  result.sequence_gap = sequenceDelta > 1;
  result.raw_stamp_delta_ns = raw_stamp_ns - lastStampNs;

  if (sequenceDelta == 0 && raw_stamp_ns <= lastStampNs)
  {
    ++state.true_duplicate;
    result.status = TimestampNormalizationStatus::TrueDuplicate;
    result.stamp_ns = lastStampNs;
    result.normalized_stamp_delta_ns = 0;
    return result;
  }

  if (result.sequence_gap)
  {
    ++state.sequence_gap_count;
    state.sequence_gap_max = std::max(state.sequence_gap_max, sequenceDelta);
  }

  result.status = TimestampNormalizationStatus::Accepted;
  result.normalized_stamp_delta_ns = result.raw_stamp_delta_ns;

  if (raw_stamp_ns <= lastStampNs)
  {
    ++state.repaired_nonmonotonic;
    result.status = TimestampNormalizationStatus::RepairedNonmonotonic;
    result.stamp_ns = lastStampNs + 1;
    result.normalized_stamp_delta_ns = result.stamp_ns - lastStampNs;
  }

  state.last_sequence = sequence;
  state.last_stamp_ns = result.stamp_ns;
  return result;
}
} // namespace OASIS::IMU::BNO086
