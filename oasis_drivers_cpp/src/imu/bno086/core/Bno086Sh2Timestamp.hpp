/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#pragma once

#include "imu/bno086/sh2/Bno086Reports.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>

namespace OASIS::IMU::BNO086
{
/*!
 * \brief SH-2 timestamp tick duration
 *
 * Units: nanoseconds per tick. The BNO08X Timebase Reference and per-report
 * delay fields use 100 us ticks, so 100 us * 1000 ns/us = 100000 ns.
 */
constexpr int64_t SH2_TIMESTAMP_TICK_NS = 100'000;

/*!
 * \brief Inputs for projecting one SH-2 report onto the host clock
 */
struct Bno086Sh2TimestampInput
{
  /*!
   * \brief Host timestamp paired with this packet's Timebase Reference
   *
   * Units: nanoseconds on the ROS time domain. This is ideally the H_INTN
   * falling-edge timestamp for the packet, or the packet read-start time when
   * the edge cannot be paired reliably to individual SHTP packets.
   */
  int64_t packet_host_anchor_ns{0};

  /*!
   * \brief True when \ref timebase_delta_ticks came from report 0xFB
   */
  bool has_timebase_reference{false};

  /*!
   * \brief Signed Timebase Reference delta from packet anchor to report base
   *
   * Units: 100 us ticks. The base time is packet anchor time minus this delta.
   */
  std::int32_t timebase_delta_ticks{0};

  /*!
   * \brief True when \ref report_delay_ticks came from a sensor report header
   */
  bool has_report_delay{false};

  /*!
   * \brief Per-report delay from the timebase to the sensor sample
   *
   * Units: 100 us ticks, expected range [0, 16383]
   */
  std::uint16_t report_delay_ticks{0};
};

/*!
 * \brief Result of projecting one SH-2 report onto the host clock
 */
struct Bno086Sh2TimestampResult
{
  /*!
   * \brief Computed event timestamp
   *
   * Units: nanoseconds on the ROS time domain
   */
  int64_t stamp_ns{0};

  /*!
   * \brief True when the Timebase Reference path was used
   */
  bool used_timebase_reference{false};

  /*!
   * \brief True when a per-report delay was applied
   */
  bool used_report_delay{false};

  /*!
   * \brief True when the timestamp fell back because report 0xFB was absent
   */
  bool used_missing_timebase_fallback{false};
};

/*!
 * \brief Result of an output monotonicity check
 */
struct Bno086OutputStampGateResult
{
  /*!
   * \brief True when the sample should be published
   */
  bool should_publish{true};

  /*!
   * \brief True when a previous published output stamp exists
   */
  bool has_previous_stamp{false};

  /*!
   * \brief Previous published output stamp
   *
   * Units: nanoseconds on the ROS time domain
   */
  int64_t previous_stamp_ns{0};

  /*!
   * \brief Candidate output stamp being checked
   *
   * Units: nanoseconds on the ROS time domain
   */
  int64_t current_stamp_ns{0};

  /*!
   * \brief Difference from previous published stamp to candidate stamp
   *
   * Units: nanoseconds, computed as current - previous
   */
  int64_t delta_ns{0};

  /*!
   * \brief True when the sample stamp matched the previous published stamp
   */
  bool duplicate_stamp{false};

  /*!
   * \brief True when the sample stamp went backward
   */
  bool backward_stamp{false};

  /*!
   * \brief True when the sample stamp went backward
   */
  bool nonmonotonic_stamp{false};
};

/*!
 * \brief Tracks one output stream's published timestamp boundary
 */
class Bno086OutputStampGate
{
public:
  Bno086OutputStampGateResult Preview(int64_t stamp_ns) const;

  Bno086OutputStampGateResult Check(int64_t stamp_ns);

  void Reset();

private:
  std::optional<int64_t> m_lastStampNs;
};

/*!
 * \brief Sequence diagnostics for one report stream
 */
struct Bno086SequenceUpdate
{
  /*!
   * \brief True when this sequence equals the previous sequence
   */
  bool duplicate_sequence{false};

  /*!
   * \brief True when this sequence skipped one or more expected values
   */
  bool sequence_gap{false};

  /*!
   * \brief Unsigned delta from previous to current sequence
   *
   * Units: 8-bit sequence ticks modulo 256
   */
  std::uint8_t sequence_delta{0};
};

/*!
 * \brief Counts duplicate and skipped SH-2 sensor report sequences
 */
class Bno086ReportSequenceDiagnostics
{
public:
  Bno086SequenceUpdate Update(ReportId report_id, std::uint8_t sequence);

  void Reset();

private:
  std::array<std::optional<std::uint8_t>, 256> m_lastSequences{};
};

Bno086Sh2TimestampResult ComputeBno086Sh2Timestamp(const Bno086Sh2TimestampInput& input);
} // namespace OASIS::IMU::BNO086
