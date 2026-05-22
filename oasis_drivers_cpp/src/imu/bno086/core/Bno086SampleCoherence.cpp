/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "imu/bno086/core/Bno086SampleCoherence.hpp"

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

ImuCorePublishDecision EvaluateLinearAccelAnchoredImuPublish(const ImuCoreSampleStamps& stamps,
                                                             int64_t max_past_age_ns,
                                                             int64_t future_tolerance_ns)
{
  ImuCorePublishDecision decision;
  decision.publish_stamp_ns = stamps.linear_accel_stamp_ns;

  if (!(stamps.has_orientation && stamps.has_gyro && stamps.has_linear_accel))
  {
    decision.status = ImuCorePublishStatus::MissingCoreState;
    return decision;
  }

  const SampleFreshnessResult orientationFreshness =
      EvaluateSampleFreshness(stamps.linear_accel_stamp_ns, stamps.orientation_stamp_ns,
                              max_past_age_ns, future_tolerance_ns);
  decision.orientation_age_ns = orientationFreshness.age_ns;
  if (orientationFreshness.status != SampleFreshnessStatus::Accepted)
  {
    decision.status = ImuCorePublishStatus::StaleOrientation;
    return decision;
  }

  const SampleFreshnessResult gyroFreshness = EvaluateSampleFreshness(
      stamps.linear_accel_stamp_ns, stamps.gyro_stamp_ns, max_past_age_ns, future_tolerance_ns);
  decision.gyro_age_ns = gyroFreshness.age_ns;
  if (gyroFreshness.status != SampleFreshnessStatus::Accepted)
  {
    decision.status = ImuCorePublishStatus::StaleGyro;
    return decision;
  }

  decision.status = ImuCorePublishStatus::Accepted;
  decision.should_publish = true;
  return decision;
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
