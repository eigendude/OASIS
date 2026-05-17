/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "imu/bno086/utils/Bno086CovarianceUtils.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>

namespace OASIS::IMU::BNO086
{
std::array<double, 9> PredictedCovarianceFromPresent(
    const std::array<double, 9>& present_orientation_covariance,
    double prediction_horizon_sec,
    double& sigma_noise_rad,
    double& sigma_rms_rad,
    double& sigma_bound_rad)
{
  // Present orientation covariance comes from the driver-owned SH-2 policy:
  // Rotation Vector estimated accuracy when available, otherwise the fallback
  // accuracy bucket table. Host prediction should never be more confident than
  // that estimate, so start from the present-time diagonal and add only
  // nonnegative growth.
  //
  // The growth term models additional small-angle variance that accumulates
  // while integrating gyro forward in time:
  //
  //   sigma_growth = k_prediction_sigma_rate_rad_per_sec * horizon
  //   variance_growth = sigma_growth^2
  //
  // This keeps Sigma_pred(h=0) == Sigma_present and makes the diagonal grow
  // monotonically for h > 0.
  constexpr double kPredictionSigmaRateRadPerSec = 0.05;

  std::array<double, 9> predictedCovariance{};
  predictedCovariance.fill(0.0);

  const double clampedHorizonSec = std::max(prediction_horizon_sec, 0.0);
  sigma_noise_rad = kPredictionSigmaRateRadPerSec * clampedHorizonSec;

  const double growthVarianceRad2 = sigma_noise_rad * sigma_noise_rad;
  double maxPredictedVarianceRad2 = 0.0;

  for (std::size_t axis = 0; axis < 3; ++axis)
  {
    const std::size_t diagonalIndex = (axis * 3) + axis;
    const double presentVarianceRad2 = std::max(present_orientation_covariance[diagonalIndex], 0.0);
    const double predictedVarianceRad2 = presentVarianceRad2 + growthVarianceRad2;
    predictedCovariance[diagonalIndex] = predictedVarianceRad2;
    maxPredictedVarianceRad2 = std::max(maxPredictedVarianceRad2, predictedVarianceRad2);
  }

  sigma_rms_rad = std::sqrt(maxPredictedVarianceRad2);
  sigma_bound_rad = sigma_rms_rad;
  return predictedCovariance;
}

OASIS::IMU::Mat3 CovarianceFromAccuracyBucket(std::uint8_t accuracy,
                                              double sigma_unreliable,
                                              double sigma_low,
                                              double sigma_medium,
                                              double sigma_high)
{
  const double sigma = (accuracy >= 3)   ? sigma_high
                       : (accuracy == 2) ? sigma_medium
                       : (accuracy == 1) ? sigma_low
                                         : sigma_unreliable;

  const double variance = sigma * sigma;

  OASIS::IMU::Mat3 covariance{};
  covariance[0][0] = variance;
  covariance[0][1] = 0.0;
  covariance[0][2] = 0.0;

  covariance[1][0] = 0.0;
  covariance[1][1] = variance;
  covariance[1][2] = 0.0;

  covariance[2][0] = 0.0;
  covariance[2][1] = 0.0;
  covariance[2][2] = variance;
  return covariance;
}

void SetCovariance(std::array<double, 9>& dst, const OASIS::IMU::Mat3& src)
{
  dst[0] = src[0][0];
  dst[1] = src[0][1];
  dst[2] = src[0][2];

  dst[3] = src[1][0];
  dst[4] = src[1][1];
  dst[5] = src[1][2];

  dst[6] = src[2][0];
  dst[7] = src[2][1];
  dst[8] = src[2][2];
}
} // namespace OASIS::IMU::BNO086
