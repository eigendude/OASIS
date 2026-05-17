/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "imu/bno086/Bno086TimestampMapper.hpp"

#include <algorithm>
#include <cstdint>
#include <limits>

namespace OASIS::IMU::BNO086
{
namespace
{
// Units: us. Maximum normal step between adjacent base timestamps; larger
// forward jumps indicate a device timeline discontinuity or unobserved pause
constexpr std::uint32_t kMaxNormalBaseStepUs = 5'000'000;

// Units: us. Minimum backwards base jump considered a reset candidate
constexpr std::uint32_t kMinResetBackwardJumpUs = 1'000'000;

// Units: ns. Minimum host-time gap required before a backwards base jump can
// be treated as a true reset instead of local batch timestamp context
constexpr int64_t kMinResetHostGapNs = 500'000'000;

// Units: ns. Long host-time gap required before implausible mapped drift can
// safely reanchor without confusing normal batched payload timing for reset
constexpr int64_t kMinImplausibleDriftReanchorHostGapNs = 500'000'000;

// Units: ns. Maximum accepted age of a mapped sample relative to host time
constexpr int64_t kMaxMappedPastSkewNs = 1'000'000'000;

// Units: ns. Maximum accepted future bias relative to packet host time
constexpr int64_t kMaxMappedFutureSkewNs = 50'000'000;

// Units: us. Range of a uint32_t microsecond timestamp before wrap
constexpr std::uint64_t kUint32Range =
    static_cast<std::uint64_t>(std::numeric_limits<std::uint32_t>::max()) + 1ULL;

// Units: us. A wrap is only claimed when the previous timestamp is near the
// maximum counter value and the current timestamp is near zero
constexpr std::uint32_t kWrapBoundaryWindowUs = kMaxNormalBaseStepUs;
} // namespace

TimestampMappingResult Bno086TimestampMapper::Map(const TimestampMappingInput& input)
{
  TimestampMappingResult result;

  if (!input.has_base_timestamp)
  {
    result.stamp_ns = input.packet_host_stamp_ns;
    if (input.has_delay)
      result.stamp_ns -= static_cast<int64_t>(input.delay_us) * 1'000;

    result.reanchor_reason = TimestampReanchorReason::MissingBase;
    result.packet_host_stamp_ns = input.packet_host_stamp_ns;
    result.mapped_stamp_after_reanchor_ns = result.stamp_ns;
    return result;
  }

  if (!m_initialized)
  {
    m_baseWrapCount = 0;
    ReanchorOffset(input.base_timestamp_us, input, result, TimestampReanchorReason::Startup,
                   static_cast<int64_t>(input.base_timestamp_us) * 1'000);
    result.initialized_offset = true;
  }

  const int64_t extendedBaseTimestampUs =
      ExtendBaseTimestampUs(input.base_timestamp_us, input, result);
  const int64_t sampleDelayUs = input.has_delay ? static_cast<int64_t>(input.delay_us) : 0;
  const int64_t deviceSampleTimeUs = extendedBaseTimestampUs - sampleDelayUs;
  result.stamp_ns = (deviceSampleTimeUs * 1'000) + m_deviceToRosOffsetNs;
  result.used_device_time = true;

  const int64_t hostDeltaNs = input.packet_host_stamp_ns - result.stamp_ns;
  if (hostDeltaNs > kMaxMappedPastSkewNs || hostDeltaNs < -kMaxMappedFutureSkewNs)
  {
    result.rejected_implausible_mapping = true;
    const int64_t hostGapNs = input.packet_host_stamp_ns - m_lastPacketHostStampNs;
    if (hostGapNs >= kMinImplausibleDriftReanchorHostGapNs)
    {
      ReanchorOffset(extendedBaseTimestampUs, input, result,
                     TimestampReanchorReason::ImplausibleDrift, result.stamp_ns);
      result.stamp_ns = (deviceSampleTimeUs * 1'000) + m_deviceToRosOffsetNs;
    }
  }

  m_lastBaseTimestampUs = input.base_timestamp_us;
  m_lastExtendedBaseTimestampUs = extendedBaseTimestampUs;
  m_lastPacketHostStampNs = input.packet_host_stamp_ns;
  return result;
}

void Bno086TimestampMapper::Reset()
{
  m_initialized = false;
  m_lastBaseTimestampUs = 0;
  m_baseWrapCount = 0;
  m_lastExtendedBaseTimestampUs = 0;
  m_lastPacketHostStampNs = 0;
  m_deviceToRosOffsetNs = 0;
}

int64_t Bno086TimestampMapper::ExtendBaseTimestampUs(std::uint32_t base_timestamp_us,
                                                     const TimestampMappingInput& input,
                                                     TimestampMappingResult& result)
{
  if (!m_initialized)
    return static_cast<int64_t>(base_timestamp_us);

  if (base_timestamp_us >= m_lastBaseTimestampUs)
  {
    return static_cast<int64_t>((m_baseWrapCount * kUint32Range) + base_timestamp_us);
  }

  const std::uint32_t backwardDeltaUs = m_lastBaseTimestampUs - base_timestamp_us;
  const bool isNearWrapBoundary =
      m_lastBaseTimestampUs >= std::numeric_limits<std::uint32_t>::max() - kWrapBoundaryWindowUs &&
      base_timestamp_us <= kWrapBoundaryWindowUs;
  if (isNearWrapBoundary)
  {
    ++m_baseWrapCount;
    result.detected_wrap_or_reset = true;
    const int64_t extendedBaseTimestampUs =
        static_cast<int64_t>((m_baseWrapCount * kUint32Range) + base_timestamp_us);
    RecordDecision(extendedBaseTimestampUs, input, result, TimestampReanchorReason::Wrap,
                   (extendedBaseTimestampUs * 1'000) + m_deviceToRosOffsetNs, m_deviceToRosOffsetNs,
                   false);
    return extendedBaseTimestampUs;
  }

  const int64_t hostGapNs = input.packet_host_stamp_ns - m_lastPacketHostStampNs;
  if (backwardDeltaUs >= kMinResetBackwardJumpUs && hostGapNs >= kMinResetHostGapNs)
  {
    m_baseWrapCount = 0;
    result.detected_wrap_or_reset = true;
    ReanchorOffset(base_timestamp_us, input, result, TimestampReanchorReason::Reset,
                   (static_cast<int64_t>(base_timestamp_us) * 1'000) + m_deviceToRosOffsetNs);
  }

  return static_cast<int64_t>(base_timestamp_us);
}

void Bno086TimestampMapper::ReanchorOffset(int64_t extended_base_timestamp_us,
                                           const TimestampMappingInput& input,
                                           TimestampMappingResult& result,
                                           TimestampReanchorReason reason,
                                           int64_t mapped_stamp_before_reanchor_ns)
{
  const int64_t oldOffsetNs = m_deviceToRosOffsetNs;
  m_deviceToRosOffsetNs = input.packet_host_stamp_ns - (extended_base_timestamp_us * 1'000);
  m_initialized = true;
  result.reanchored_offset = true;
  RecordDecision(extended_base_timestamp_us, input, result, reason, mapped_stamp_before_reanchor_ns,
                 oldOffsetNs, true);
}

void Bno086TimestampMapper::RecordDecision(int64_t extended_base_timestamp_us,
                                           const TimestampMappingInput& input,
                                           TimestampMappingResult& result,
                                           TimestampReanchorReason reason,
                                           int64_t mapped_stamp_before_reanchor_ns,
                                           int64_t old_offset_ns,
                                           bool reanchor)
{
  result.reanchor_reason = reason;
  result.previous_base_timestamp_us = m_lastBaseTimestampUs;
  result.current_base_timestamp_us = input.base_timestamp_us;
  result.raw_base_delta_us =
      static_cast<int64_t>(input.base_timestamp_us) - static_cast<int64_t>(m_lastBaseTimestampUs);
  result.previous_extended_device_time_us = m_lastExtendedBaseTimestampUs;
  result.current_extended_device_time_us = extended_base_timestamp_us;
  result.packet_host_stamp_ns = input.packet_host_stamp_ns;
  result.old_offset_ns = reanchor ? old_offset_ns : m_deviceToRosOffsetNs;
  result.new_offset_ns = m_deviceToRosOffsetNs;
  result.offset_delta_ms = static_cast<double>(result.new_offset_ns - result.old_offset_ns) / 1e6;
  result.mapped_stamp_before_reanchor_ns = mapped_stamp_before_reanchor_ns;
  result.mapped_stamp_after_reanchor_ns =
      (extended_base_timestamp_us * 1'000) + m_deviceToRosOffsetNs;
}
} // namespace OASIS::IMU::BNO086
