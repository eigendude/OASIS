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
// Largest plausible SH-2 base timestamp step accepted between packet anchors
// Units: microseconds
constexpr std::int64_t kMaxBaseTimestampStepUs = 1'000'000;

// Upper uint32 range where a backward base timestamp may be real wraparound
// Units: microseconds
constexpr std::uint32_t kWrapHighWaterUs = 0xFFFF0000U;

// Lower uint32 range where a backward base timestamp may be real wraparound
// Units: microseconds
constexpr std::uint32_t kWrapLowWaterUs = 0x00010000U;

// Largest allowed disagreement between SH-2 base delta and host elapsed time
// Units: microseconds
constexpr std::int64_t kMaxBaseHostDeltaErrorUs = 100'000;

bool IsPlausibleWrapWindow(std::uint32_t current_base_us, std::uint32_t last_base_us)
{
  return last_base_us >= kWrapHighWaterUs && current_base_us <= kWrapLowWaterUs;
}

std::optional<std::uint64_t> ComputeWrappedDeltaUs(std::uint32_t current_base_us,
                                                   std::uint32_t last_base_us)
{
  if (!IsPlausibleWrapWindow(current_base_us, last_base_us))
    return std::nullopt;

  return (static_cast<std::uint64_t>(std::numeric_limits<std::uint32_t>::max()) - last_base_us) +
         1ULL + current_base_us;
}

std::optional<std::int64_t> ComputeBaseDeltaUs(std::uint32_t current_base_us,
                                               std::uint32_t last_base_us)
{
  if (current_base_us >= last_base_us)
    return static_cast<std::int64_t>(current_base_us - last_base_us);

  const std::optional<std::uint64_t> wrappedDeltaUs =
      ComputeWrappedDeltaUs(current_base_us, last_base_us);
  if (!wrappedDeltaUs.has_value())
    return std::nullopt;

  if (*wrappedDeltaUs > static_cast<std::uint64_t>(kMaxBaseTimestampStepUs))
    return std::nullopt;

  return static_cast<std::int64_t>(*wrappedDeltaUs);
}
} // namespace

std::int64_t Bno086TimestampReconstructor::Reconstruct(const SensorEvent& event,
                                                       std::int64_t packet_host_stamp_ns)
{
  std::int64_t baseHostStampNs = packet_host_stamp_ns;

  if (event.has_base_timestamp)
  {
    if (m_lastBaseTimestampUs.has_value() && m_lastBaseHostStampNs.has_value())
    {
      const std::optional<std::int64_t> baseDeltaUs =
          ComputeBaseDeltaUs(event.base_timestamp_us, *m_lastBaseTimestampUs);
      const std::int64_t hostDeltaUs = (packet_host_stamp_ns - *m_lastBaseHostStampNs) / 1000;

      m_diagnostics.latest_host_delta_us = hostDeltaUs;

      bool acceptDelta = false;
      if (baseDeltaUs.has_value())
      {
        const std::int64_t errorUs = *baseDeltaUs - hostDeltaUs;

        m_diagnostics.latest_base_delta_us = *baseDeltaUs;
        m_diagnostics.latest_base_host_error_us = errorUs;

        acceptDelta = *baseDeltaUs >= 0 && *baseDeltaUs <= kMaxBaseTimestampStepUs &&
                      hostDeltaUs >= 0 && std::llabs(errorUs) <= kMaxBaseHostDeltaErrorUs;
      }
      else
      {
        m_diagnostics.latest_base_delta_us = 0;
        m_diagnostics.latest_base_host_error_us = 0;
      }

      if (acceptDelta)
      {
        baseHostStampNs = *m_lastBaseHostStampNs + (*baseDeltaUs * 1000);

        if (event.base_timestamp_us < *m_lastBaseTimestampUs)
          ++m_diagnostics.base_wraps_accepted;
      }
      else
      {
        const std::optional<std::uint64_t> wrappedDeltaUs =
            ComputeWrappedDeltaUs(event.base_timestamp_us, *m_lastBaseTimestampUs);

        ++m_diagnostics.base_resets;
        if (!baseDeltaUs.has_value())
        {
          if (wrappedDeltaUs.has_value())
            ++m_diagnostics.base_resets_large_delta;
          else
            ++m_diagnostics.base_resets_negative_delta;
        }
        else if (*baseDeltaUs > kMaxBaseTimestampStepUs)
        {
          ++m_diagnostics.base_resets_large_delta;
        }
        else
        {
          ++m_diagnostics.base_resets_host_mismatch;
        }
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
