/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "imu/bno086/Bno086GravityUtils.hpp"

namespace OASIS::IMU::BNO086
{
Vec3 CanonicalizeGravityVector(const Vec3& raw_gravity_mps2)
{
  return {
      -raw_gravity_mps2[0],
      -raw_gravity_mps2[1],
      -raw_gravity_mps2[2],
  };
}

PublishedGravityMeasurement MakePublishedGravityMeasurement(
    const Vec3& gravity_mps2, const std::optional<Mat3>& gravity_cov_mps2_2)
{
  PublishedGravityMeasurement measurement;
  measurement.gravity_mps2 = gravity_mps2;
  measurement.covariance.fill(0.0);

  if (gravity_cov_mps2_2.has_value())
  {
    measurement.covariance[0] = (*gravity_cov_mps2_2)[0][0];
    measurement.covariance[1] = (*gravity_cov_mps2_2)[0][1];
    measurement.covariance[2] = (*gravity_cov_mps2_2)[0][2];

    measurement.covariance[6] = (*gravity_cov_mps2_2)[1][0];
    measurement.covariance[7] = (*gravity_cov_mps2_2)[1][1];
    measurement.covariance[8] = (*gravity_cov_mps2_2)[1][2];

    measurement.covariance[12] = (*gravity_cov_mps2_2)[2][0];
    measurement.covariance[13] = (*gravity_cov_mps2_2)[2][1];
    measurement.covariance[14] = (*gravity_cov_mps2_2)[2][2];
  }

  // geometry_msgs/AccelWithCovariance uses the rotational block for angular
  // acceleration. The BNO086 gravity topic does not estimate that quantity.
  measurement.covariance[21] = -1.0;
  return measurement;
}
} // namespace OASIS::IMU::BNO086
