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

// Units: ns. Maximum accepted offset drift between the stable device clock
// mapping and a fresh host receive anchor; much larger than batch latency
constexpr int64_t kMaxOffsetDriftNs = 250'000'000;

// Units: ns. Maximum accepted age of a mapped sample relative to host time
constexpr int64_t kMaxMappedPastSkewNs = 1'000'000'000;

// Units: ns. Maximum accepted future bias relative to packet host time
constexpr int64_t kMaxMappedFutureSkewNs = 50'000'000;

// Units: us. Range of a uint32_t microsecond timestamp before wrap
constexpr std::uint64_t kUint32Range =
    static_cast<std::uint64_t>(std::numeric_limits<std::uint32_t>::max()) + 1ULL;

int64_t Abs64(int64_t value)
{
  return value < 0 ? -value : value;
}
} // namespace

TimestampMappingResult Bno086TimestampMapper::Map(const TimestampMappingInput& input)
{
  TimestampMappingResult result;

  if (!input.has_base_timestamp)
  {
    result.stamp_ns = input.packet_host_stamp_ns;
    if (input.has_delay)
      result.stamp_ns -= static_cast<int64_t>(input.delay_us) * 1'000;

    return result;
  }

  if (!m_initialized)
  {
    m_baseWrapCount = 0;
    m_lastBaseTimestampUs = input.base_timestamp_us;
    ReanchorOffset(input.base_timestamp_us, input, result);
    result.initialized_offset = true;
  }

  const int64_t extendedBaseTimestampUs =
      ExtendBaseTimestampUs(input.base_timestamp_us, input, result);
  const int64_t freshOffsetNs = input.packet_host_stamp_ns - (extendedBaseTimestampUs * 1'000);
  if (Abs64(freshOffsetNs - m_deviceToRosOffsetNs) > kMaxOffsetDriftNs)
    ReanchorOffset(extendedBaseTimestampUs, input, result);

  const int64_t sampleDelayUs = input.has_delay ? static_cast<int64_t>(input.delay_us) : 0;
  const int64_t deviceSampleTimeUs = extendedBaseTimestampUs - sampleDelayUs;
  result.stamp_ns = (deviceSampleTimeUs * 1'000) + m_deviceToRosOffsetNs;
  result.used_device_time = true;

  const int64_t hostDeltaNs = input.packet_host_stamp_ns - result.stamp_ns;
  if (hostDeltaNs > kMaxMappedPastSkewNs || hostDeltaNs < -kMaxMappedFutureSkewNs)
  {
    result.rejected_implausible_mapping = true;
    ReanchorOffset(extendedBaseTimestampUs, input, result);
    result.stamp_ns = (deviceSampleTimeUs * 1'000) + m_deviceToRosOffsetNs;
  }

  m_lastBaseTimestampUs = input.base_timestamp_us;
  return result;
}

void Bno086TimestampMapper::Reset()
{
  m_initialized = false;
  m_lastBaseTimestampUs = 0;
  m_baseWrapCount = 0;
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
    const std::uint32_t forwardDeltaUs = base_timestamp_us - m_lastBaseTimestampUs;
    if (forwardDeltaUs > kMaxNormalBaseStepUs)
    {
      m_baseWrapCount = 0;
      result.detected_wrap_or_reset = true;
      ReanchorOffset(base_timestamp_us, input, result);
    }

    return static_cast<int64_t>((m_baseWrapCount * kUint32Range) + base_timestamp_us);
  }

  const std::uint64_t wrapForwardDeltaUs = static_cast<std::uint64_t>(base_timestamp_us) +
                                           kUint32Range -
                                           static_cast<std::uint64_t>(m_lastBaseTimestampUs);
  if (wrapForwardDeltaUs <= kMaxNormalBaseStepUs)
  {
    ++m_baseWrapCount;
    result.detected_wrap_or_reset = true;
    return static_cast<int64_t>((m_baseWrapCount * kUint32Range) + base_timestamp_us);
  }

  m_baseWrapCount = 0;
  result.detected_wrap_or_reset = true;
  ReanchorOffset(base_timestamp_us, input, result);
  return static_cast<int64_t>(base_timestamp_us);
}

void Bno086TimestampMapper::ReanchorOffset(int64_t extended_base_timestamp_us,
                                           const TimestampMappingInput& input,
                                           TimestampMappingResult& result)
{
  m_deviceToRosOffsetNs = input.packet_host_stamp_ns - (extended_base_timestamp_us * 1'000);
  m_initialized = true;
  result.reanchored_offset = true;
}
} // namespace OASIS::IMU::BNO086
