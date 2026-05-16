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
 * \brief Per-report timestamp repair state for one SH-2 report stream
 */
struct ReportTimestampState
{
  /*!
   * \brief True after the first sample for this report stream is accepted
   *
   * Units: boolean
   */
  bool has_sample{false};

  /*!
   * \brief Last accepted SH-2 report sequence number
   *
   * Units: uint8 counter ticks, wraps at 255
   */
  std::uint8_t last_sequence{0};

  /*!
   * \brief Last accepted or repaired host-domain timestamp
   *
   * Units: nanoseconds in the caller's timestamp domain
   */
  std::int64_t last_stamp_ns{0};

  /*!
   * \brief Count of advanced-sequence samples with non-increasing raw stamps
   *
   * Units: samples
   */
  std::uint64_t repaired_nonmonotonic{0};

  /*!
   * \brief Count of same-sequence samples with non-increasing raw stamps
   *
   * Units: samples
   */
  std::uint64_t true_duplicate{0};

  /*!
   * \brief Count of accepted samples where the sequence advanced by > 1
   *
   * Units: samples
   */
  std::uint64_t sequence_gap_count{0};

  /*!
   * \brief Largest observed uint8 sequence delta for this report stream
   *
   * Units: counter ticks in range [0, 255]
   */
  std::uint8_t sequence_gap_max{0};
};

/*!
 * \brief Timestamp repair behavior for one normalization operation
 */
struct TimestampNormalizationConfig
{
  /*!
   * \brief Expected time between consecutive reports
   *
   * Units: microseconds, diagnostic context only
   */
  std::uint32_t expected_interval_us{10'000};
};

/*!
 * \brief Classification of one report timestamp normalization
 */
enum class TimestampNormalizationStatus : std::uint8_t
{
  FirstSample,
  Accepted,
  RepairedNonmonotonic,
  TrueDuplicate,
};

/*!
 * \brief Result of normalizing one report timestamp
 */
struct TimestampNormalizationResult
{
  /*!
   * \brief Classification chosen for this sample
   *
   * Units: enum code
   */
  TimestampNormalizationStatus status{TimestampNormalizationStatus::FirstSample};

  /*!
   * \brief Raw or repaired monotonic timestamp
   *
   * Units: nanoseconds in the caller's timestamp domain
   */
  std::int64_t stamp_ns{0};

  /*!
   * \brief Difference between the raw stamp and prior accepted stamp
   *
   * Units: nanoseconds
   */
  std::int64_t raw_stamp_delta_ns{0};

  /*!
   * \brief Difference between the returned stamp and prior accepted stamp
   *
   * Units: nanoseconds
   */
  std::int64_t normalized_stamp_delta_ns{0};

  /*!
   * \brief Wrapped uint8 delta from prior sequence to current sequence
   *
   * Units: counter ticks in range [0, 255]
   */
  std::uint8_t sequence_delta{0};

  /*!
   * \brief True when \ref sequence_delta shows one or more missing samples
   *
   * Units: boolean
   */
  bool sequence_gap{false};
};

TimestampNormalizationResult NormalizeReportTimestamp(ReportTimestampState& state,
                                                      std::uint8_t sequence,
                                                      std::int64_t raw_stamp_ns,
                                                      const TimestampNormalizationConfig& config);
} // namespace OASIS::IMU::BNO086
