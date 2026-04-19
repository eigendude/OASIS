/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "imu/bno086/Bno086OrientationCovariancePolicy.hpp"

#include <gtest/gtest.h>

namespace OASIS::IMU::BNO086
{
TEST(Bno086OrientationCovariancePolicy, mapsEachAccuracyBucketToExpectedFallbackSigma)
{
  EXPECT_DOUBLE_EQ(OrientationAccuracyBucketToSigmaRad(0), 0.20);
  EXPECT_DOUBLE_EQ(OrientationAccuracyBucketToSigmaRad(1), 0.08);
  EXPECT_DOUBLE_EQ(OrientationAccuracyBucketToSigmaRad(2), 0.03);
  EXPECT_DOUBLE_EQ(OrientationAccuracyBucketToSigmaRad(3), 0.012);
}

TEST(Bno086OrientationCovariancePolicy, invalidAccuracyBucketFallsBackToUnreliableSigma)
{
  EXPECT_DOUBLE_EQ(OrientationAccuracyBucketToSigmaRad(4), 0.20);
  EXPECT_DOUBLE_EQ(OrientationAccuracyBucketToSigmaRad(255), 0.20);
}

TEST(Bno086OrientationCovariancePolicy, usesRotationVectorAccuracyEstimateWhenPresent)
{
  const OrientationCovariancePolicyResult result = ResolveOrientationCovariancePolicy(1, 96);

  EXPECT_EQ(result.accuracy_bucket, 1);
  EXPECT_EQ(result.raw_accuracy_estimate_q12, 96);
  EXPECT_TRUE(result.has_accuracy_estimate);
  EXPECT_NEAR(result.accuracy_estimate_rad, 96.0 / 4096.0, 1e-12);
  EXPECT_EQ(result.source, OrientationCovarianceSource::RotationVectorAccuracyEstimate);
  EXPECT_NEAR(result.sigma_rad, 96.0 / 4096.0, 1e-12);
  EXPECT_NEAR(result.covariance_rad2[0][0], result.sigma_rad * result.sigma_rad, 1e-12);
  EXPECT_DOUBLE_EQ(result.covariance_rad2[0][1], 0.0);
  EXPECT_DOUBLE_EQ(result.covariance_rad2[1][2], 0.0);
  EXPECT_NEAR(result.covariance_rad2[2][2], result.sigma_rad * result.sigma_rad, 1e-12);
}

TEST(Bno086OrientationCovariancePolicy, enforcesMinimumSigmaWhenEstimateIsTooSmall)
{
  const OrientationCovariancePolicyResult result = ResolveOrientationCovariancePolicy(3, 1);

  EXPECT_TRUE(result.has_accuracy_estimate);
  EXPECT_NEAR(result.accuracy_estimate_rad, 1.0 / 4096.0, 1e-12);
  EXPECT_EQ(result.source, OrientationCovarianceSource::RotationVectorAccuracyEstimate);
  EXPECT_DOUBLE_EQ(result.sigma_rad, 0.012);
  EXPECT_DOUBLE_EQ(result.covariance_rad2[0][0], 0.000144);
  EXPECT_DOUBLE_EQ(result.covariance_rad2[1][1], 0.000144);
  EXPECT_DOUBLE_EQ(result.covariance_rad2[2][2], 0.000144);
}

TEST(Bno086OrientationCovariancePolicy, fallsBackToBucketSigmaWhenAccuracyEstimateIsInvalid)
{
  const OrientationCovariancePolicyResult result = ResolveOrientationCovariancePolicy(2, 0);

  EXPECT_FALSE(result.has_accuracy_estimate);
  EXPECT_DOUBLE_EQ(result.accuracy_estimate_rad, 0.0);
  EXPECT_EQ(result.source, OrientationCovarianceSource::AccuracyBucketFallback);
  EXPECT_DOUBLE_EQ(result.sigma_rad, 0.03);
  EXPECT_DOUBLE_EQ(result.covariance_rad2[0][0], 0.0009);
  EXPECT_DOUBLE_EQ(result.covariance_rad2[1][1], 0.0009);
  EXPECT_DOUBLE_EQ(result.covariance_rad2[2][2], 0.0009);
}

TEST(Bno086OrientationCovariancePolicy,
     fallsBackToBucketSigmaWhenAccuracyEstimateExceedsReasonableRange)
{
  const OrientationCovariancePolicyResult result = ResolveOrientationCovariancePolicy(3, 13000);

  EXPECT_FALSE(result.has_accuracy_estimate);
  EXPECT_DOUBLE_EQ(result.accuracy_estimate_rad, 0.0);
  EXPECT_EQ(result.source, OrientationCovarianceSource::AccuracyBucketFallback);
  EXPECT_DOUBLE_EQ(result.sigma_rad, 0.012);
  EXPECT_DOUBLE_EQ(result.covariance_rad2[0][0], 0.000144);
}

TEST(Bno086OrientationCovariancePolicy, sourceNamesAreStable)
{
  EXPECT_STREQ(
      OrientationCovarianceSourceName(OrientationCovarianceSource::RotationVectorAccuracyEstimate),
      "rotation_vector_accuracy_estimate");
  EXPECT_STREQ(OrientationCovarianceSourceName(OrientationCovarianceSource::AccuracyBucketFallback),
               "accuracy_bucket_fallback");
}
} // namespace OASIS::IMU::BNO086
