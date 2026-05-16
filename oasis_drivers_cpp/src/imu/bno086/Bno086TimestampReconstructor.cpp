/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "imu/bno086/Bno086TimestampReconstructor.hpp"

#include <cstdlib>
#include <limits>

namespace OASIS::IMU::BNO086
{
namespace
{
// Largest plausible SH-2 timebase delta accepted from one interrupt packet
// Units: microseconds
constexpr std::int64_t kMaxBaseTimestampStepUs = 1'000'000;
} // namespace

std::int64_t Bno086TimestampReconstructor::Reconstruct(const SensorEvent& event,
                                                       std::int64_t packet_host_stamp_ns)
{
  std::int64_t baseHostStampNs = packet_host_stamp_ns;

  if (event.has_base_timestamp)
  {
    baseHostStampNs =
        packet_host_stamp_ns - (static_cast<std::int64_t>(event.base_timestamp_us) * 1000);

    m_diagnostics.latest_base_delta_us = event.base_timestamp_us;
    m_diagnostics.latest_host_delta_us =
        m_lastBaseHostStampNs.has_value() ? (baseHostStampNs - *m_lastBaseHostStampNs) / 1000 : 0;
    m_diagnostics.latest_base_host_error_us = 0;

    if (event.base_timestamp_us > kMaxBaseTimestampStepUs)
    {
      ++m_diagnostics.base_resets;
      ++m_diagnostics.base_resets_large_delta;
      baseHostStampNs = packet_host_stamp_ns;
    }

    m_lastBaseTimestampUs = event.base_timestamp_us;
    m_lastBaseHostStampNs = baseHostStampNs;
  }
  else
  {
    ++m_diagnostics.missing_base;
  }

  std::int64_t stampNs = baseHostStampNs;
  if (event.has_delay)
  {
    stampNs += static_cast<std::int64_t>(event.delay_us) * 1000;
    ++m_diagnostics.delay_applied;
  }

  return stampNs;
}

const TimestampReconstructionDiagnostics& Bno086TimestampReconstructor::GetDiagnostics() const
{
  return m_diagnostics;
}
} // namespace OASIS::IMU::BNO086
