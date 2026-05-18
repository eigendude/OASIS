/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#pragma once

#include <cstdint>
#include <optional>

namespace OASIS::IMU::BNO086
{
/*!
 * \brief Input for per-report cadence timestamp tracking
 */
struct ReportTimestampTrackerInput
{
  /*!
   * \brief Report sequence value associated with this sample
   *
   * Units: unsigned 8-bit counter ticks from the report payload
   */
  std::uint8_t sequence{0};

  /*!
   * \brief Accepted report interval for this stream
   *
   * Units: nanoseconds, unset when the stream cadence is unknown
   */
  std::optional<int64_t> expected_interval_ns;

  /*!
   * \brief Host timestamp for the packet carrying this sample
   *
   * Units: nanoseconds on the caller's monotonic timeline
   */
  int64_t packet_host_stamp_ns{0};

  /*!
   * \brief True when the report has an SH-2 sample delay field
   */
  bool has_delay{false};

  /*!
   * \brief Delay from sample time to packet context
   *
   * Units: microseconds, used only when it is small enough to be plausible
   */
  std::uint32_t delay_us{0};

  /*!
   * \brief Shared timestamp epoch for cadence-aligned report streams
   *
   * Units: nanoseconds on the caller's monotonic timeline. When set with a
   * known interval, the first sample is placed on this epoch's cadence grid
   * near the packet host anchor so streams share one phase reference.
   */
  std::optional<int64_t> shared_epoch_stamp_ns;
};

/*!
 * \brief Result from per-report cadence timestamp tracking
 */
struct ReportTimestampTrackerResult
{
  /*!
   * \brief Timestamp selected for this report sample
   *
   * Units: nanoseconds on the caller's monotonic timeline
   */
  int64_t stamp_ns{0};

  /*!
   * \brief Cadence candidate before any host reanchor guard
   *
   * Units: nanoseconds on the caller's monotonic timeline
   */
  int64_t candidate_stamp_ns{0};

  /*!
   * \brief Packet host anchor used to validate the cadence candidate
   *
   * Units: nanoseconds on the caller's monotonic timeline
   */
  int64_t host_anchor_stamp_ns{0};

  /*!
   * \brief Candidate minus host anchor when the stream reanchored to host
   *
   * Units: nanoseconds
   */
  int64_t reanchor_delta_ns{0};

  /*!
   * \brief True when this sample initialized the stream state
   */
  bool initialized{false};

  /*!
   * \brief Unsigned sequence delta from previous to current sample
   *
   * Units: report sequence counter ticks
   */
  std::uint8_t sequence_delta{0};

  /*!
   * \brief True when this sample reset the stream timestamp anchor
   */
  bool reanchored{false};

  /*!
   * \brief True when stale or future cadence reanchored to packet host time
   */
  bool reanchored_to_host{false};

  /*!
   * \brief True when sequence or cadence implied a stream gap
   */
  bool gap_detected{false};

  /*!
   * \brief True when the sample repeated the previous sequence value
   */
  bool duplicate_sequence{false};

  /*!
   * \brief True when timestamp came from sequence delta and interval
   */
  bool used_interval_cadence{false};

  /*!
   * \brief True when timestamp came from packet host time
   */
  bool used_host_anchor{false};

  /*!
   * \brief True when the first timestamp used a shared epoch cadence grid
   */
  bool used_shared_epoch_anchor{false};
};

/*!
 * \brief Per-report timestamp tracker with no ROS dependencies
 */
class Bno086ReportTimestampTracker
{
public:
  ReportTimestampTrackerResult Update(const ReportTimestampTrackerInput& input);
  void Reset();

private:
  int64_t HostAnchorNs(const ReportTimestampTrackerInput& input) const;
  int64_t SharedEpochAnchorNs(const ReportTimestampTrackerInput& input) const;

  bool m_initialized{false};
  std::uint8_t m_lastSequence{0};
  int64_t m_lastStampNs{0};
};
} // namespace OASIS::IMU::BNO086
