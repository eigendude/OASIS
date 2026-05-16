/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#pragma once

#include "imu/bno086/Bno086Reports.hpp"

#include <cstdint>
#include <optional>

namespace OASIS::IMU::BNO086
{
/*!
 * \brief Runtime counters for SH-2 timestamp reconstruction
 */
struct TimestampReconstructionDiagnostics
{
  /*!
   * \brief Count of base timestamp jumps that reset the host anchor
   *
   * Units: events
   */
  std::uint64_t base_resets{0};

  /*!
   * \brief Count of resets from backward base movement without plausible wrap
   *
   * Units: events
   */
  std::uint64_t base_resets_negative_delta{0};

  /*!
   * \brief Count of resets from base deltas larger than the sane bound
   *
   * Units: events
   */
  std::uint64_t base_resets_large_delta{0};

  /*!
   * \brief Count of resets from disagreement between base and host deltas
   *
   * Units: events
   */
  std::uint64_t base_resets_host_mismatch{0};

  /*!
   * \brief Count of accepted uint32 base timestamp wraparounds
   *
   * Units: events
   */
  std::uint64_t base_wraps_accepted{0};

  /*!
   * \brief Count of events reconstructed from packet host time without a base
   *
   * Units: events
   */
  std::uint64_t missing_base{0};

  /*!
   * \brief Count of events where per-report delay was subtracted
   *
   * Units: events
   */
  std::uint64_t delay_applied{0};

  /*!
   * \brief Latest SH-2 base timestamp delta considered
   *
   * Units: microseconds
   */
  std::int64_t latest_base_delta_us{0};

  /*!
   * \brief Latest host timestamp delta between base anchors
   *
   * Units: microseconds
   */
  std::int64_t latest_host_delta_us{0};

  /*!
   * \brief Latest difference between SH-2 base delta and host delta
   *
   * Units: microseconds
   */
  std::int64_t latest_base_host_error_us{0};
};

/*!
 * \brief Reconstructs host-domain timestamps from SH-2 report timing fields
 */
class Bno086TimestampReconstructor
{
public:
  std::int64_t Reconstruct(const SensorEvent& event, std::int64_t packet_host_stamp_ns);

  const TimestampReconstructionDiagnostics& GetDiagnostics() const;

private:
  std::optional<std::uint32_t> m_lastBaseTimestampUs;
  std::optional<std::int64_t> m_lastBaseHostStampNs;
  TimestampReconstructionDiagnostics m_diagnostics{};
};
} // namespace OASIS::IMU::BNO086
