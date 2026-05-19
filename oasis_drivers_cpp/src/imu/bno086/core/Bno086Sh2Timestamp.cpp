/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "imu/bno086/core/Bno086Sh2Timestamp.hpp"

namespace OASIS::IMU::BNO086
{
Bno086Sh2TimestampResult ComputeBno086Sh2Timestamp(const Bno086Sh2TimestampInput& input)
{
  Bno086Sh2TimestampResult result;
  result.used_timebase_reference = input.has_timebase_reference;
  result.used_report_delay = input.has_report_delay;
  result.used_missing_timebase_fallback = !input.has_timebase_reference;

  const int64_t delayNs = input.has_report_delay ? static_cast<int64_t>(input.report_delay_ticks) *
                                                       SH2_TIMESTAMP_TICK_NS
                                                 : 0;

  if (input.has_timebase_reference)
  {
    const int64_t baseDeltaNs =
        static_cast<int64_t>(input.timebase_delta_ticks) * SH2_TIMESTAMP_TICK_NS;
    result.stamp_ns = input.interrupt_stamp_ns - baseDeltaNs + delayNs;
  }
  else
  {
    result.stamp_ns = input.interrupt_stamp_ns + delayNs;
  }

  return result;
}

Bno086OutputStampGateResult Bno086OutputStampGate::Check(int64_t stamp_ns)
{
  Bno086OutputStampGateResult result;

  if (m_lastStampNs.has_value())
  {
    result.duplicate_stamp = stamp_ns == *m_lastStampNs;
    result.nonmonotonic_stamp = stamp_ns < *m_lastStampNs;
    result.should_publish = stamp_ns > *m_lastStampNs;
  }

  if (result.should_publish)
    m_lastStampNs = stamp_ns;

  return result;
}

void Bno086OutputStampGate::Reset()
{
  m_lastStampNs.reset();
}

Bno086SequenceUpdate Bno086ReportSequenceDiagnostics::Update(ReportId report_id,
                                                             std::uint8_t sequence)
{
  Bno086SequenceUpdate result;
  const std::size_t index = static_cast<std::size_t>(report_id);
  std::optional<std::uint8_t>& lastSequence = m_lastSequences[index];

  if (lastSequence.has_value())
  {
    result.sequence_delta = static_cast<std::uint8_t>(sequence - *lastSequence);
    result.duplicate_sequence = result.sequence_delta == 0;
    result.sequence_gap = result.sequence_delta > 1;
  }

  lastSequence = sequence;
  return result;
}

void Bno086ReportSequenceDiagnostics::Reset()
{
  for (std::optional<std::uint8_t>& sequence : m_lastSequences)
    sequence.reset();
}
} // namespace OASIS::IMU::BNO086
