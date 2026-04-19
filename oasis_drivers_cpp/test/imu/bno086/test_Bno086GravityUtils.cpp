/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "imu/bno086/Bno086GravityUtils.hpp"

#include <gtest/gtest.h>

namespace OASIS::IMU::BNO086
{
TEST(Bno086GravityUtils, canonicalizeGravityVectorFlipsRawSh2Sign)
{
  const Vec3 rawGravityMps2{0.0, 0.0, 9.8125};

  const Vec3 canonicalGravityMps2 = CanonicalizeGravityVector(rawGravityMps2);

  EXPECT_DOUBLE_EQ(canonicalGravityMps2[0], -0.0);
  EXPECT_DOUBLE_EQ(canonicalGravityMps2[1], -0.0);
  EXPECT_DOUBLE_EQ(canonicalGravityMps2[2], -9.8125);
}

TEST(Bno086GravityUtils, publishedGravityMeasurementPreservesCanonicalDownVectorAndCovariance)
{
  const Vec3 canonicalGravityMps2{0.0, 0.0, -9.81};
  const Mat3 gravityCovarianceMps22{
      std::array<double, 3>{0.04, 0.01, 0.02},
      std::array<double, 3>{0.01, 0.05, 0.03},
      std::array<double, 3>{0.02, 0.03, 0.06},
  };

  const PublishedGravityMeasurement measurement =
      MakePublishedGravityMeasurement(canonicalGravityMps2, gravityCovarianceMps22);

  EXPECT_DOUBLE_EQ(measurement.gravity_mps2[0], 0.0);
  EXPECT_DOUBLE_EQ(measurement.gravity_mps2[1], 0.0);
  EXPECT_DOUBLE_EQ(measurement.gravity_mps2[2], -9.81);

  EXPECT_DOUBLE_EQ(measurement.covariance[0], 0.04);
  EXPECT_DOUBLE_EQ(measurement.covariance[1], 0.01);
  EXPECT_DOUBLE_EQ(measurement.covariance[2], 0.02);
  EXPECT_DOUBLE_EQ(measurement.covariance[6], 0.01);
  EXPECT_DOUBLE_EQ(measurement.covariance[7], 0.05);
  EXPECT_DOUBLE_EQ(measurement.covariance[8], 0.03);
  EXPECT_DOUBLE_EQ(measurement.covariance[12], 0.02);
  EXPECT_DOUBLE_EQ(measurement.covariance[13], 0.03);
  EXPECT_DOUBLE_EQ(measurement.covariance[14], 0.06);
  EXPECT_DOUBLE_EQ(measurement.covariance[21], -1.0);
}

TEST(Bno086GravityUtils,
     publishedGravityMeasurementMarksAngularAccelerationUnknownWithoutCovariance)
{
  const PublishedGravityMeasurement measurement =
      MakePublishedGravityMeasurement(Vec3{1.0, -2.0, -9.81}, std::nullopt);

  EXPECT_DOUBLE_EQ(measurement.gravity_mps2[0], 1.0);
  EXPECT_DOUBLE_EQ(measurement.gravity_mps2[1], -2.0);
  EXPECT_DOUBLE_EQ(measurement.gravity_mps2[2], -9.81);
  EXPECT_DOUBLE_EQ(measurement.covariance[0], 0.0);
  EXPECT_DOUBLE_EQ(measurement.covariance[7], 0.0);
  EXPECT_DOUBLE_EQ(measurement.covariance[14], 0.0);
  EXPECT_DOUBLE_EQ(measurement.covariance[21], -1.0);
}
} // namespace OASIS::IMU::BNO086
