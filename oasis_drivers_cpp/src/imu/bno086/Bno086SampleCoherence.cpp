/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "imu/bno086/Bno086SampleCoherence.hpp"

#include <algorithm>

namespace OASIS::IMU::BNO086
{
SampleFreshnessResult EvaluateSampleFreshness(int64_t reference_stamp_ns,
                                              int64_t sample_stamp_ns,
                                              int64_t max_past_age_ns,
                                              int64_t future_tolerance_ns)
{
  SampleFreshnessResult result;
  result.age_ns = reference_stamp_ns - sample_stamp_ns;

  if (result.age_ns > max_past_age_ns)
    result.status = SampleFreshnessStatus::TooOld;
  else if (-result.age_ns > future_tolerance_ns)
    result.status = SampleFreshnessStatus::TooFuture;
  else
    result.status = SampleFreshnessStatus::Accepted;

  return result;
}

int64_t EffectiveMaxPastAgeNs(int64_t configured_max_age_ns,
                              int64_t report_interval_ns,
                              int64_t minimum_max_age_ns)
{
  return std::max({configured_max_age_ns, 3 * report_interval_ns, minimum_max_age_ns});
}

int64_t EffectiveCoreSpanToleranceNs(int64_t linear_interval_ns,
                                     int64_t gyro_interval_ns,
                                     int64_t orientation_interval_ns,
                                     int64_t minimum_span_ns)
{
  return std::max(linear_interval_ns + gyro_interval_ns + orientation_interval_ns, minimum_span_ns);
}

bool IsTimestampSpanCoherent(int64_t oldest_stamp_ns, int64_t newest_stamp_ns, int64_t max_span_ns)
{
  return newest_stamp_ns >= oldest_stamp_ns && newest_stamp_ns - oldest_stamp_ns <= max_span_ns;
}
} // namespace OASIS::IMU::BNO086
