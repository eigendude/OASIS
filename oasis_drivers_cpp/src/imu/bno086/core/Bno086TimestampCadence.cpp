/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "imu/bno086/core/Bno086TimestampCadence.hpp"

#include <cstddef>

namespace OASIS::IMU::BNO086
{
namespace
{
// Units: ns. SH-2 delay values above this are not plausible sample latency
constexpr int64_t kMaxPlausibleSampleDelayNs = 50'000'000;

bool IsKnownReportId(ReportId report_id)
{
  switch (report_id)
  {
    case ReportId::Accelerometer:
    case ReportId::GyroscopeCalibrated:
    case ReportId::LinearAcceleration:
    case ReportId::RotationVector:
    case ReportId::Gravity:
      return true;
    default:
      break;
  }

  return false;
}
} // namespace

void Bno086TimestampCadence::Reset()
{
  for (Bno086ReportTimestampTracker& tracker : m_timestampTrackers)
    tracker.Reset();
  m_bnoCadenceEpochNs.reset();
  m_lastEmittedTimestampNs.fill(std::nullopt);
}

Bno086TimestampCadenceResult Bno086TimestampCadence::Finalize(
    const SensorEvent& event,
    int64_t packet_host_stamp_ns,
    std::optional<int64_t> expected_interval_ns)
{
  Bno086TimestampCadenceResult result;
  if (!IsKnownReportId(event.report_id))
    return result;

  const std::size_t timestampIndex = static_cast<std::size_t>(event.report_id);
  if (timestampIndex >= m_timestampTrackers.size())
    return result;

  if (!m_bnoCadenceEpochNs.has_value())
    m_bnoCadenceEpochNs = HostAnchorNs(event, packet_host_stamp_ns);

  const ReportTimestampTrackerResult trackedStamp = m_timestampTrackers[timestampIndex].Update(
      ReportTimestampTrackerInput{event.sequence, expected_interval_ns, packet_host_stamp_ns,
                                  event.has_delay, event.delay_us, m_bnoCadenceEpochNs});

  result.candidate_stamp_ns = trackedStamp.stamp_ns;
  result.duplicate_sequence = trackedStamp.duplicate_sequence;
  result.sequence_delta = trackedStamp.sequence_delta;

  int64_t stampNs = trackedStamp.stamp_ns;
  bool duplicate = trackedStamp.duplicate_sequence;

  const std::optional<int64_t>& lastStampNs = m_lastEmittedTimestampNs[timestampIndex];
  result.last_stamp_ns = lastStampNs;
  if (duplicate && lastStampNs.has_value())
    return result;

  if (lastStampNs.has_value() && stampNs <= *lastStampNs)
  {
    if (trackedStamp.duplicate_sequence)
    {
      duplicate = true;
      stampNs = *lastStampNs;
    }
    else if (expected_interval_ns.has_value() && *expected_interval_ns > 0)
    {
      stampNs = *lastStampNs + *expected_interval_ns;
    }
    else
    {
      stampNs = *lastStampNs + 1;
    }

    result.monotonic_guard_adjusted = true;
  }

  m_lastEmittedTimestampNs[timestampIndex] = stampNs;
  if (!duplicate)
    result.stamp_ns = stampNs;
  return result;
}

int64_t Bno086TimestampCadence::HostAnchorNs(const SensorEvent& event,
                                             int64_t packet_host_stamp_ns) const
{
  if (!event.has_delay)
    return packet_host_stamp_ns;

  const int64_t delayNs = static_cast<int64_t>(event.delay_us) * 1'000;
  if (delayNs < 0 || delayNs > kMaxPlausibleSampleDelayNs)
    return packet_host_stamp_ns;

  return packet_host_stamp_ns - delayNs;
}
} // namespace OASIS::IMU::BNO086
