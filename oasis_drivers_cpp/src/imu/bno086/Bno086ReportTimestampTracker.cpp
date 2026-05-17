/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "imu/bno086/Bno086ReportTimestampTracker.hpp"

#include <algorithm>
#include <cstdint>

namespace OASIS::IMU::BNO086
{
namespace
{
// Units: ns. Allows cadence-derived samples in one batched packet to land
// modestly ahead of the host drain anchor
constexpr int64_t kBatchedTimestampFutureSlopNs = 50'000'000;

// Units: sequence ticks. Larger deltas are treated as stream gaps instead of
// expanding one missing payload into a long synthetic cadence run
constexpr std::uint8_t kMaxCadenceSequenceDelta = 16;

std::uint8_t SequenceDelta(std::uint8_t last_sequence, std::uint8_t current_sequence)
{
  return static_cast<std::uint8_t>(current_sequence - last_sequence);
}
} // namespace

ReportTimestampTrackerResult Bno086ReportTimestampTracker::Update(
    const ReportTimestampTrackerInput& input)
{
  ReportTimestampTrackerResult result;

  if (!m_initialized)
  {
    result.stamp_ns = HostAnchorNs(input);
    result.initialized = true;
    result.reanchored = true;
    result.used_host_anchor = true;

    m_initialized = true;
    m_lastSequence = input.sequence;
    m_lastStampNs = result.stamp_ns;
    return result;
  }

  result.sequence_delta = SequenceDelta(m_lastSequence, input.sequence);
  result.duplicate_sequence = result.sequence_delta == 0;

  if (input.expected_interval_ns.has_value() && *input.expected_interval_ns > 0)
  {
    if (result.sequence_delta == 0)
    {
      result.stamp_ns = m_lastStampNs;
    }
    else
    {
      result.gap_detected = result.sequence_delta > 1U;
      const int64_t intervalAdvanceNs =
          *input.expected_interval_ns * static_cast<int64_t>(result.sequence_delta);
      const int64_t cadenceStampNs = m_lastStampNs + intervalAdvanceNs;
      const bool cadenceIsPlausible =
          result.sequence_delta <= kMaxCadenceSequenceDelta &&
          cadenceStampNs <= input.packet_host_stamp_ns + kBatchedTimestampFutureSlopNs;

      if (cadenceIsPlausible)
      {
        result.stamp_ns = cadenceStampNs;
        result.used_interval_cadence = true;
      }
      else
      {
        result.stamp_ns = std::max(m_lastStampNs + *input.expected_interval_ns,
                                   input.packet_host_stamp_ns - kBatchedTimestampFutureSlopNs);
        result.reanchored = true;
        result.gap_detected = true;
        result.used_host_anchor = true;
      }
    }
  }
  else
  {
    result.stamp_ns = HostAnchorNs(input);
    result.used_host_anchor = true;
  }

  m_lastSequence = input.sequence;
  m_lastStampNs = result.stamp_ns;
  return result;
}

void Bno086ReportTimestampTracker::Reset()
{
  m_initialized = false;
  m_lastSequence = 0;
  m_lastStampNs = 0;
}

int64_t Bno086ReportTimestampTracker::HostAnchorNs(const ReportTimestampTrackerInput& input) const
{
  if (!input.has_delay)
    return input.packet_host_stamp_ns;

  const int64_t delayNs = static_cast<int64_t>(input.delay_us) * 1'000;
  if (delayNs < 0 || delayNs > kBatchedTimestampFutureSlopNs)
    return input.packet_host_stamp_ns;

  return input.packet_host_stamp_ns - delayNs;
}
} // namespace OASIS::IMU::BNO086
