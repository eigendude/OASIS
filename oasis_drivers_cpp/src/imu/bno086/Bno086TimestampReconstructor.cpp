/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "imu/bno086/Bno086TimestampReconstructor.hpp"

namespace OASIS::IMU::BNO086
{
namespace
{
// Largest plausible SH-2 base timestamp step accepted between packet anchors
// Units: microseconds
constexpr std::uint32_t kMaxBaseTimestampStepUs = 1'000'000;
} // namespace

std::int64_t Bno086TimestampReconstructor::Reconstruct(const SensorEvent& event,
                                                       std::int64_t packet_host_stamp_ns)
{
  std::int64_t baseHostStampNs = packet_host_stamp_ns;

  if (event.has_base_timestamp)
  {
    if (m_lastBaseTimestampUs.has_value() && m_lastBaseHostStampNs.has_value())
    {
      const std::uint32_t deltaUs = event.base_timestamp_us - *m_lastBaseTimestampUs;

      if (deltaUs <= kMaxBaseTimestampStepUs)
      {
        baseHostStampNs = *m_lastBaseHostStampNs + static_cast<std::int64_t>(deltaUs) * 1000;
      }
      else
      {
        ++m_diagnostics.base_resets;
      }
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
    stampNs -= static_cast<std::int64_t>(event.delay_us) * 1000;
    ++m_diagnostics.delay_applied;
  }

  return stampNs;
}

const TimestampReconstructionDiagnostics& Bno086TimestampReconstructor::GetDiagnostics() const
{
  return m_diagnostics;
}
} // namespace OASIS::IMU::BNO086
