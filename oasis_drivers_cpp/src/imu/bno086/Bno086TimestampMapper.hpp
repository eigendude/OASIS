/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#pragma once

#include <cstdint>

namespace OASIS::IMU::BNO086
{
/*!
 * \brief Input for mapping an SH-2 device timestamp into ROS time
 */
struct TimestampMappingInput
{
  /*!
   * \brief True when the report has an SH-2 base timestamp
   */
  bool has_base_timestamp{false};

  /*!
   * \brief SH-2 base timestamp associated with this report
   *
   * Units: microseconds, converted from the SH-2 100 us tick field
   */
  std::uint32_t base_timestamp_us{0};

  /*!
   * \brief True when the report has an SH-2 sample delay field
   */
  bool has_delay{false};

  /*!
   * \brief Delay from sample time to base timestamp
   *
   * Units: microseconds, converted from the SH-2 100 us tick field
   */
  std::uint32_t delay_us{0};

  /*!
   * \brief Report sequence value associated with this sample
   *
   * Units: unsigned counter ticks from the report payload
   */
  std::uint8_t sequence{0};

  /*!
   * \brief Host timestamp for the packet carrying this sample
   *
   * Units: nanoseconds on the caller's monotonic timeline
   */
  int64_t packet_host_stamp_ns{0};
};

/*!
 * \brief Result from mapping an SH-2 timestamp into ROS time
 */
struct TimestampMappingResult
{
  /*!
   * \brief Timestamp mapped onto the caller's monotonic timeline
   *
   * Units: nanoseconds
   */
  int64_t stamp_ns{0};

  /*!
   * \brief True when the output used SH-2 device timestamp fields
   */
  bool used_device_time{false};

  /*!
   * \brief True when the device-to-ROS offset was initialized
   */
  bool initialized_offset{false};

  /*!
   * \brief True when the device-to-ROS offset was re-anchored
   */
  bool reanchored_offset{false};

  /*!
   * \brief True when the base timestamp appeared to wrap or reset
   */
  bool detected_wrap_or_reset{false};

  /*!
   * \brief True when the stable mapping was implausible
   */
  bool rejected_implausible_mapping{false};
};

/*!
 * \brief Stable SH-2 device-time to ROS-time mapper with no ROS dependencies
 */
class Bno086TimestampMapper
{
public:
  TimestampMappingResult Map(const TimestampMappingInput& input);
  void Reset();

private:
  int64_t ExtendBaseTimestampUs(std::uint32_t base_timestamp_us,
                                const TimestampMappingInput& input,
                                TimestampMappingResult& result);
  void ReanchorOffset(int64_t extended_base_timestamp_us,
                      const TimestampMappingInput& input,
                      TimestampMappingResult& result);

  bool m_initialized{false};
  std::uint32_t m_lastBaseTimestampUs{0};
  std::uint64_t m_baseWrapCount{0};
  int64_t m_deviceToRosOffsetNs{0};
};
} // namespace OASIS::IMU::BNO086
