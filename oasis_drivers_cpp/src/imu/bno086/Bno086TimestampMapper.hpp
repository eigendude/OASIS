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
 * \brief Reason the timestamp mapper changed or flagged its time anchor
 */
enum class TimestampReanchorReason
{
  /*!
   * \brief No reanchor or reset decision was made
   */
  None,

  /*!
   * \brief First valid base timestamp initialized the mapping offset
   */
  Startup,

  /*!
   * \brief Large forward drift forced a fresh mapping anchor
   */
  Forward,

  /*!
   * \brief Base timestamp wrapped at the device counter boundary
   */
  Wrap,

  /*!
   * \brief Large discontinuity after a host-time gap indicated reset
   */
  Reset,

  /*!
   * \brief Stable mapping drifted implausibly far from host time
   */
  ImplausibleDrift,

  /*!
   * \brief Report lacked a base timestamp and used packet host time
   */
  MissingBase,
};

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

  /*!
   * \brief Reason for the latest reanchor or reset decision
   */
  TimestampReanchorReason reanchor_reason{TimestampReanchorReason::None};

  /*!
   * \brief Previous base timestamp before the decision
   *
   * Units: microseconds from the SH-2 base timestamp field
   */
  std::uint32_t previous_base_timestamp_us{0};

  /*!
   * \brief Current base timestamp used for the decision
   *
   * Units: microseconds from the SH-2 base timestamp field
   */
  std::uint32_t current_base_timestamp_us{0};

  /*!
   * \brief Signed delta from previous to current base timestamp
   *
   * Units: microseconds, before any wrap extension
   */
  int64_t raw_base_delta_us{0};

  /*!
   * \brief Previous extended device base time before the decision
   *
   * Units: microseconds on the mapper's extended device timeline
   */
  int64_t previous_extended_device_time_us{0};

  /*!
   * \brief Current extended device base time used for the decision
   *
   * Units: microseconds on the mapper's extended device timeline
   */
  int64_t current_extended_device_time_us{0};

  /*!
   * \brief Host timestamp for the packet carrying the decision sample
   *
   * Units: nanoseconds on the caller's monotonic timeline
   */
  int64_t packet_host_stamp_ns{0};

  /*!
   * \brief Device-to-ROS offset before the decision
   *
   * Units: nanoseconds
   */
  int64_t old_offset_ns{0};

  /*!
   * \brief Device-to-ROS offset after the decision
   *
   * Units: nanoseconds
   */
  int64_t new_offset_ns{0};

  /*!
   * \brief Offset change caused by the decision
   *
   * Units: milliseconds, positive means the offset moved later
   */
  double offset_delta_ms{0.0};

  /*!
   * \brief Mapped stamp computed before reanchoring
   *
   * Units: nanoseconds on the caller's monotonic timeline
   */
  int64_t mapped_stamp_before_reanchor_ns{0};

  /*!
   * \brief Mapped stamp computed after reanchoring
   *
   * Units: nanoseconds on the caller's monotonic timeline
   */
  int64_t mapped_stamp_after_reanchor_ns{0};
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
                      TimestampMappingResult& result,
                      TimestampReanchorReason reason,
                      int64_t mapped_stamp_before_reanchor_ns);
  void RecordDecision(int64_t extended_base_timestamp_us,
                      const TimestampMappingInput& input,
                      TimestampMappingResult& result,
                      TimestampReanchorReason reason,
                      int64_t mapped_stamp_before_reanchor_ns,
                      int64_t old_offset_ns,
                      bool reanchor);

  bool m_initialized{false};
  std::uint32_t m_lastBaseTimestampUs{0};
  std::uint64_t m_baseWrapCount{0};
  int64_t m_lastExtendedBaseTimestampUs{0};
  int64_t m_lastPacketHostStampNs{0};
  int64_t m_deviceToRosOffsetNs{0};
};
} // namespace OASIS::IMU::BNO086
