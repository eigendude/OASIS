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
 * \brief Freshness result for pairing two timestamped samples
 */
enum class SampleFreshnessStatus
{
  /*!
   * \brief Sample is close enough to the reference timestamp
   */
  Accepted,

  /*!
   * \brief Sample is before the reference by more than the age limit
   */
  TooOld,

  /*!
   * \brief Sample is after the reference by more than the future tolerance
   */
  TooFuture,
};

/*!
 * \brief Result from comparing a sample stamp against a reference stamp
 */
struct SampleFreshnessResult
{
  /*!
   * \brief Freshness classification
   *
   * Expected range: one of Accepted, TooOld, or TooFuture. Used by the node to
   * decide whether a cached sample can compose with the current cadence anchor.
   */
  SampleFreshnessStatus status{SampleFreshnessStatus::Accepted};

  /*!
   * \brief Signed age of sample relative to reference
   *
   * Units: nanoseconds. Positive means the sample is older than the reference;
   * negative means the sample is slightly in the future.
   */
  int64_t age_ns{0};
};

/*!
 * \brief Evaluate whether a timestamped sample can pair with a reference
 *
 * \param reference_stamp_ns Reference sample timestamp in nanoseconds
 * \param sample_stamp_ns Candidate sample timestamp in nanoseconds
 * \param max_past_age_ns Maximum accepted positive age in nanoseconds
 * \param future_tolerance_ns Maximum accepted negative age magnitude in
 * nanoseconds
 */
SampleFreshnessResult EvaluateSampleFreshness(int64_t reference_stamp_ns,
                                              int64_t sample_stamp_ns,
                                              int64_t max_past_age_ns,
                                              int64_t future_tolerance_ns);

/*!
 * \brief Check whether a multi-sample timestamp span is coherent
 *
 * \param oldest_stamp_ns Oldest timestamp in the frame, nanoseconds
 * \param newest_stamp_ns Newest timestamp in the frame, nanoseconds
 * \param max_span_ns Maximum accepted newest-minus-oldest span, nanoseconds
 */
bool IsTimestampSpanCoherent(int64_t oldest_stamp_ns, int64_t newest_stamp_ns, int64_t max_span_ns);
} // namespace OASIS::IMU::BNO086
