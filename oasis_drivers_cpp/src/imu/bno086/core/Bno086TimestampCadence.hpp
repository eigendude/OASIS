/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#pragma once

#include "imu/bno086/core/Bno086ReportTimestampTracker.hpp"
#include "imu/bno086/sh2/Bno086Reports.hpp"

#include <array>
#include <cstdint>
#include <optional>

namespace OASIS::IMU::BNO086
{
/*!
 * \brief Finalized timestamp decision for one BNO086 sensor event
 */
struct Bno086TimestampCadenceResult
{
  /*!
   * \brief Final timestamp to emit, unset for duplicates or invalid reports
   *
   * Units: ns
   */
  std::optional<int64_t> stamp_ns;

  /*!
   * \brief Tracker candidate timestamp before monotonic guarding
   *
   * Units: ns
   */
  int64_t candidate_stamp_ns{0};

  /*!
   * \brief Packet host anchor used to validate cadence timestamp
   *
   * Units: ns
   */
  int64_t host_anchor_stamp_ns{0};

  /*!
   * \brief Candidate minus host anchor when reanchored to host
   *
   * Units: ns
   */
  int64_t reanchor_delta_ns{0};

  /*!
   * \brief Previous emitted timestamp for this report stream
   *
   * Units: ns
   */
  std::optional<int64_t> last_stamp_ns;

  /*!
   * \brief True when this event repeats the previous report sequence
   */
  bool duplicate_sequence{false};

  /*!
   * \brief Unsigned sequence delta from previous to current report
   *
   * Units: report sequence counter ticks
   */
  std::uint8_t sequence_delta{0};

  /*!
   * \brief True when the helper adjusted a non-monotonic candidate
   */
  bool monotonic_guard_adjusted{false};

  /*!
   * \brief True when the tracker reanchored stale or future cadence to host
   */
  bool reanchored_to_host{false};
};

/*!
 * \brief Per-report timestamp cadence finalizer with monotonic guarding
 */
class Bno086TimestampCadence
{
public:
  void Reset();

  Bno086TimestampCadenceResult Finalize(const SensorEvent& event,
                                        int64_t packet_host_stamp_ns,
                                        std::optional<int64_t> expected_interval_ns);

private:
  int64_t HostAnchorNs(const SensorEvent& event, int64_t packet_host_stamp_ns) const;

  std::array<Bno086ReportTimestampTracker, 256> m_timestampTrackers{};
  std::optional<int64_t> m_bnoCadenceEpochNs;
  std::array<std::optional<int64_t>, 256> m_lastEmittedTimestampNs{};
};
} // namespace OASIS::IMU::BNO086
