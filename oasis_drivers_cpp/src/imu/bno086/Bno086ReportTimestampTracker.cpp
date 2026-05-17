/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "imu/bno086/Bno086ReportTimestampTracker.hpp"

#include <cstdint>

namespace OASIS::IMU::BNO086
{
namespace
{
// Units: ns. Allows cadence-derived samples in one batched packet to land
// modestly ahead of the host drain anchor
constexpr int64_t kBatchedTimestampFutureSlopNs = 50'000'000;

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
    result.stamp_ns = SharedEpochAnchorNs(input);
    result.initialized = true;
    result.reanchored = true;
    result.used_shared_epoch_anchor = input.shared_epoch_stamp_ns.has_value() &&
                                      input.expected_interval_ns.has_value() &&
                                      *input.expected_interval_ns > 0;
    result.used_host_anchor = !result.used_shared_epoch_anchor;

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
      result.stamp_ns = m_lastStampNs + intervalAdvanceNs;
      result.used_interval_cadence = true;
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

int64_t Bno086ReportTimestampTracker::SharedEpochAnchorNs(
    const ReportTimestampTrackerInput& input) const
{
  if (!input.shared_epoch_stamp_ns.has_value() || !input.expected_interval_ns.has_value() ||
      *input.expected_interval_ns <= 0)
  {
    return HostAnchorNs(input);
  }

  const int64_t hostAnchorNs = HostAnchorNs(input);
  const int64_t epochNs = *input.shared_epoch_stamp_ns;
  if (hostAnchorNs <= epochNs)
    return epochNs;

  const int64_t elapsedNs = hostAnchorNs - epochNs;
  const int64_t intervalNs = *input.expected_interval_ns;
  const int64_t intervalCount = elapsedNs / intervalNs;
  return epochNs + intervalCount * intervalNs;
}
} // namespace OASIS::IMU::BNO086
