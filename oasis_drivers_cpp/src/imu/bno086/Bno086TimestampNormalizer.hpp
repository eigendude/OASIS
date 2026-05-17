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
 * \brief Input sample metadata for monotonic timestamp normalization
 */
struct TimestampSample
{
  TimestampSample() = default;

  TimestampSample(std::uint8_t sequence_value,
                  int64_t stamp_value_ns,
                  int64_t packet_host_stamp_value_ns,
                  std::optional<int64_t> max_future_skew_value_ns = std::nullopt)
    : sequence(sequence_value),
      stamp_ns(stamp_value_ns),
      packet_host_stamp_ns(packet_host_stamp_value_ns),
      max_future_skew_ns(max_future_skew_value_ns)
  {
  }

  /*!
   * \brief Report sequence value associated with the sample
   *
   * Units: unsigned counter ticks from the report payload
   */
  std::uint8_t sequence{0};

  /*!
   * \brief Reconstructed sample timestamp
   *
   * Units: nanoseconds on the caller's monotonic timeline
   */
  int64_t stamp_ns{0};

  /*!
   * \brief Host timestamp for the packet carrying this sample
   *
   * Units: nanoseconds on the caller's monotonic timeline
   */
  int64_t packet_host_stamp_ns{0};

  /*!
   * \brief Optional future skew accepted before host-time reset
   *
   * Units: nanoseconds, unset uses the normalizer default
   */
  std::optional<int64_t> max_future_skew_ns;
};

/*!
 * \brief Result of normalizing one timestamp sample
 */
struct TimestampNormalizationResult
{
  /*!
   * \brief Timestamp to use after sanity clamping and monotonic repair
   *
   * Units: nanoseconds on the caller's monotonic timeline
   */
  int64_t stamp_ns{0};

  /*!
   * \brief True when the sample repeated both sequence and timestamp
   */
  bool duplicate{false};

  /*!
   * \brief True when a new sequence needed a minimal monotonic repair
   */
  bool repaired_nonmonotonic{false};

  /*!
   * \brief True when an equal stamp was repaired using expected cadence
   */
  bool repaired_duplicate_to_interval{false};

  /*!
   * \brief True when a regressing stamp was repaired using expected cadence
   */
  bool repaired_nonmonotonic_to_interval{false};

  /*!
   * \brief True when a sequence gap was repaired using expected cadence
   */
  bool repaired_sequence_gap_to_interval{false};

  /*!
   * \brief True when interval repair was bounded to packet host time
   */
  bool interval_repair_clamped_to_host{false};

  /*!
   * \brief True when interval repair could only use legacy monotonic repair
   */
  bool interval_repair_bounded_to_legacy{false};

  /*!
   * \brief True when reconstruction was implausible and host time was used
   */
  bool reconstruction_reset{false};

  /*!
   * \brief True when the report sequence skipped one or more values
   */
  bool sequence_gap{false};
};

/*!
 * \brief Per-report timestamp normalizer with no ROS dependencies
 */
class Bno086TimestampNormalizer
{
public:
  TimestampNormalizationResult Normalize(
      const TimestampSample& sample, std::optional<int64_t> expected_interval_ns = std::nullopt);
  void Reset();

private:
  bool m_hasLast{false};
  std::uint8_t m_lastSequence{0};
  int64_t m_lastStampNs{0};
};
} // namespace OASIS::IMU::BNO086
