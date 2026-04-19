/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "imu/bno086/Bno086OrientationCovariancePolicy.hpp"

#include <algorithm>
#include <cmath>

namespace OASIS::IMU::BNO086
{
namespace
{
constexpr double kRotationVectorAccuracyQ12Scale = 1.0 / 4096.0;

// SH-2 exposes only a coarse orientation quality bucket here rather than a
// native full 3x3 orientation covariance estimate.
//
// These fallback 1-sigma buckets are provisional engineering heuristics used
// to publish a conservative ROS covariance contract when the Rotation Vector
// estimated-accuracy field is unusable.
//
// They are not strongly justified as a physical covariance model from the
// datasheet alone, and may be calibrated later from real robot resting data.
constexpr double kFallbackSigmaUnreliableRad = 0.20;
constexpr double kFallbackSigmaLowRad = 0.08;
constexpr double kFallbackSigmaMediumRad = 0.03;
constexpr double kFallbackSigmaHighRad = 0.012;

constexpr double kMinimumPublishedSigmaRad = 0.012;
constexpr double kMaximumReasonableAccuracyEstimateRad = 3.14159265358979323846;

double RotationVectorAccuracyEstimateRad(std::int16_t raw_accuracy_estimate_q12)
{
  return static_cast<double>(raw_accuracy_estimate_q12) * kRotationVectorAccuracyQ12Scale;
}

Mat3 DiagonalCovarianceFromSigma(double sigma_rad)
{
  const double variance_rad2 = sigma_rad * sigma_rad;

  Mat3 covariance{};
  covariance[0][0] = variance_rad2;
  covariance[0][1] = 0.0;
  covariance[0][2] = 0.0;

  covariance[1][0] = 0.0;
  covariance[1][1] = variance_rad2;
  covariance[1][2] = 0.0;

  covariance[2][0] = 0.0;
  covariance[2][1] = 0.0;
  covariance[2][2] = variance_rad2;
  return covariance;
}
} // namespace

OrientationCovariancePolicyResult ResolveOrientationCovariancePolicy(
    std::uint8_t accuracy_bucket, std::int16_t raw_accuracy_estimate_q12)
{
  OrientationCovariancePolicyResult result;
  result.accuracy_bucket = accuracy_bucket;
  result.raw_accuracy_estimate_q12 = raw_accuracy_estimate_q12;
  result.sigma_rad = OrientationAccuracyBucketToSigmaRad(accuracy_bucket);
  result.source = OrientationCovarianceSource::AccuracyBucketFallback;

  // The Rotation Vector report carries a fifth 16-bit field after the
  // quaternion terms. BNO08X metadata marks that auxiliary term as Q12, and
  // the SH-2 Rotation Vector input report defines it as the accuracy estimate
  // in radians.
  const double decoded_accuracy_estimate_rad =
      RotationVectorAccuracyEstimateRad(raw_accuracy_estimate_q12);
  result.accuracy_estimate_rad = decoded_accuracy_estimate_rad;

  if (std::isfinite(decoded_accuracy_estimate_rad) && decoded_accuracy_estimate_rad > 0.0 &&
      decoded_accuracy_estimate_rad <= kMaximumReasonableAccuracyEstimateRad)
  {
    result.has_accuracy_estimate = true;

    // This is still a driver-boundary heuristic rather than a native SH-2 3x3
    // covariance. Prefer the Rotation Vector estimated accuracy whenever SH-2
    // provides a sane value, then clamp to a small resting floor so the
    // published covariance stays useful without implying a perfect attitude
    // solution.
    result.sigma_rad = std::max(decoded_accuracy_estimate_rad, kMinimumPublishedSigmaRad);
    result.source = OrientationCovarianceSource::RotationVectorAccuracyEstimate;
  }
  else if (!(decoded_accuracy_estimate_rad > 0.0))
  {
    // Fall back to the explicit bucket heuristic table when SH-2 does not
    // provide a usable estimated-accuracy value.
    result.rejection_reason = OrientationCovarianceEstimateRejectionReason::RawEstimateNonPositive;
  }
  else
  {
    // Fall back to the explicit bucket heuristic table when SH-2 does not
    // provide a usable estimated-accuracy value.
    result.rejection_reason = OrientationCovarianceEstimateRejectionReason::RawEstimateOutOfRange;
  }

  result.covariance_rad2 = DiagonalCovarianceFromSigma(result.sigma_rad);
  return result;
}

double OrientationAccuracyBucketToSigmaRad(std::uint8_t accuracy_bucket)
{
  switch (accuracy_bucket)
  {
    case 3:
      return kFallbackSigmaHighRad;

    case 2:
      return kFallbackSigmaMediumRad;

    case 1:
      return kFallbackSigmaLowRad;

    case 0:
    default:
      return kFallbackSigmaUnreliableRad;
  }
}

const char* OrientationCovarianceSourceName(OrientationCovarianceSource source)
{
  switch (source)
  {
    case OrientationCovarianceSource::RotationVectorAccuracyEstimate:
      return "rotation_vector_accuracy_estimate";

    case OrientationCovarianceSource::AccuracyBucketFallback:
    default:
      return "accuracy_bucket_fallback";
  }
}

const char* OrientationCovarianceEstimateRejectionReasonName(
    OrientationCovarianceEstimateRejectionReason reason)
{
  switch (reason)
  {
    case OrientationCovarianceEstimateRejectionReason::None:
      return "none";

    case OrientationCovarianceEstimateRejectionReason::RawEstimateNonPositive:
      return "raw_estimate_non_positive";

    case OrientationCovarianceEstimateRejectionReason::RawEstimateOutOfRange:
      return "raw_estimate_out_of_range";

    default:
      return "unknown";
  }
}
} // namespace OASIS::IMU::BNO086
