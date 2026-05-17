/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "imu/bno086/utils/Bno086CovarianceUtils.hpp"

namespace OASIS::IMU::BNO086
{
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

} // namespace OASIS::IMU::BNO086
