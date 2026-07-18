/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "ahrs/AhrsMath.hpp"

#include <cmath>
#include <limits>

#include <gtest/gtest.h>

using namespace OASIS::AHRS;

namespace
{
void ExpectVectorNear(const Eigen::Vector3d& actual,
                      const Eigen::Vector3d& expected,
                      double tolerance = 1.0e-12)
{
  EXPECT_NEAR(actual.x(), expected.x(), tolerance);
  EXPECT_NEAR(actual.y(), expected.y(), tolerance);
  EXPECT_NEAR(actual.z(), expected.z(), tolerance);
}
} // namespace

TEST(AhrsMath, MountingSolveMapsTiltedGravityToBaseNegativeZ)
{
  const Eigen::Quaterniond q_tilt = QuaternionFromRollPitchYaw(0.20, -0.35, 0.0);
  const Eigen::Vector3d gravity_imu =
      q_tilt.inverse().toRotationMatrix() * Eigen::Vector3d(0.0, 0.0, -1.0);

  const std::optional<AhrsMountingSolution> solution =
      SolveMountingFromGravity(gravity_imu, 12, 2.0);

  ASSERT_TRUE(solution.has_value());
  const Eigen::Vector3d gravity_base = solution->R_BI * gravity_imu;
  EXPECT_NEAR(gravity_base.x(), 0.0, 1.0e-9);
  EXPECT_NEAR(gravity_base.y(), 0.0, 1.0e-9);
  EXPECT_NEAR(gravity_base.z(), -1.0, 1.0e-9);
}

TEST(AhrsMath, MountingYawRemainsZero)
{
  const std::optional<AhrsMountingSolution> solution =
      SolveMountingFromGravity(Eigen::Vector3d(0.1, 0.2, -0.97).normalized(), 10, 2.0);

  ASSERT_TRUE(solution.has_value());
  EXPECT_NEAR(YawFromQuaternion(solution->q_BI), 0.0, 1.0e-12);
}

TEST(AhrsMath, RotatesCovarianceThroughMounting)
{
  const Eigen::Matrix3d rotation =
      QuaternionFromRollPitchYaw(0.0, 0.0, M_PI / 2.0).toRotationMatrix();
  Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
  covariance(0, 0) = 1.0;
  covariance(1, 1) = 4.0;
  covariance(2, 2) = 9.0;

  const Eigen::Matrix3d rotated = RotateCovariance(rotation, covariance);

  EXPECT_NEAR(rotated(0, 0), 4.0, 1.0e-12);
  EXPECT_NEAR(rotated(1, 1), 1.0, 1.0e-12);
  EXPECT_NEAR(rotated(2, 2), 9.0, 1.0e-12);
}

TEST(AhrsMath, CovarianceParsingRequiresFiniteValuesAndNonnegativeDiagonal)
{
  const std::array<double, 9> valid{1.0, 0.2, -0.3, 0.4, 2.0, 0.5, -0.6, 0.7, 3.0};
  const std::optional<Eigen::Matrix3d> parsed = ParseMatrix3(valid);

  ASSERT_TRUE(parsed.has_value());
  EXPECT_DOUBLE_EQ((*parsed)(0, 1), 0.2);
  EXPECT_DOUBLE_EQ((*parsed)(2, 1), 0.7);

  for (const std::size_t diagonal_index : {0U, 4U, 8U})
  {
    std::array<double, 9> negative_diagonal = valid;
    negative_diagonal[diagonal_index] = -1.0;
    EXPECT_FALSE(ParseMatrix3(negative_diagonal).has_value());
  }

  std::array<double, 9> nonfinite = valid;
  nonfinite[1] = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(ParseMatrix3(nonfinite).has_value());
}

TEST(AhrsMath, LinearCovarianceParsingRetainsCrossAxisTerms)
{
  std::array<double, 36> covariance{};
  covariance[0] = 1.0;
  covariance[1] = 0.25;
  covariance[2] = -0.5;
  covariance[6] = 0.75;
  covariance[7] = 2.0;
  covariance[8] = 1.25;
  covariance[12] = -1.5;
  covariance[13] = 1.75;
  covariance[14] = 3.0;

  const std::optional<Eigen::Matrix3d> parsed = ParseLinearCovariance3(covariance);

  ASSERT_TRUE(parsed.has_value());
  EXPECT_DOUBLE_EQ((*parsed)(0, 1), 0.25);
  EXPECT_DOUBLE_EQ((*parsed)(1, 2), 1.25);
  EXPECT_DOUBLE_EQ((*parsed)(2, 0), -1.5);
}

TEST(AhrsMath, GravityResidualMatchesPredictedDirection)
{
  const Eigen::Quaterniond q_WB = QuaternionFromRollPitchYaw(0.31, -0.27, 0.73);
  const Eigen::Vector3d gravity_world(0.0, 0.0, -1.0);
  const Eigen::Vector3d gravity_base = q_WB.conjugate() * gravity_world;
  const std::optional<AhrsGravityResidual> residual =
      ComputeGravityResidual(9.81 * gravity_base, std::nullopt, q_WB);

  ASSERT_TRUE(residual.has_value());
  ExpectVectorNear(residual->predicted_direction, gravity_base);
  EXPECT_NEAR(residual->residual_norm, 0.0, 1.0e-12);
  EXPECT_TRUE(std::isnan(residual->mahalanobis_distance));
}

TEST(AhrsMath, ZeroGravityResidualWithUsableCovarianceHasZeroMahalanobisDistance)
{
  const Eigen::Matrix3d covariance = Eigen::Matrix3d::Identity();
  const std::optional<AhrsGravityResidual> residual = ComputeGravityResidual(
      Eigen::Vector3d(0.0, 0.0, -9.81), covariance, Eigen::Quaterniond::Identity());

  ASSERT_TRUE(residual.has_value());
  EXPECT_DOUBLE_EQ(residual->mahalanobis_distance, 0.0);
}

TEST(AhrsMath, NormalizedGravityCovarianceUsesExactSpecifiedFloor)
{
  const Eigen::Quaterniond q_WB = QuaternionFromRollPitchYaw(0.01, 0.0, 0.0);
  const std::optional<AhrsGravityResidual> residual =
      ComputeGravityResidual(Eigen::Vector3d(0.0, 0.0, -9.81), Eigen::Matrix3d::Zero(), q_WB);

  ASSERT_TRUE(residual.has_value());
  const double expected = residual->residual_norm / std::sqrt(1.0e-9);
  EXPECT_NEAR(residual->mahalanobis_distance, expected, 1.0e-9 * expected);
}

TEST(AhrsMath, FailedNormalizedCovarianceInversionLeavesMahalanobisUnavailable)
{
  const Eigen::Matrix3d covariance =
      std::numeric_limits<double>::max() * Eigen::Matrix3d::Identity();
  const std::optional<AhrsGravityResidual> residual = ComputeGravityResidual(
      Eigen::Vector3d(0.0, 0.0, -1.0e-8), covariance, QuaternionFromRollPitchYaw(0.1, 0.0, 0.0));

  ASSERT_TRUE(residual.has_value());
  EXPECT_TRUE(std::isnan(residual->mahalanobis_distance));
}

TEST(AhrsMath, AcceptedSmallGravityContributesToMounting)
{
  AhrsMountingConfig config;
  config.calibration_duration_sec = 0.0;
  config.min_sample_count = 1;
  BootMountingCalibrator calibrator(config);

  const std::optional<AhrsMountingSolution> solution =
      calibrator.AddGravitySample(0, Eigen::Vector3d(0.0, 0.0, -5.0e-4), std::nullopt);

  ASSERT_TRUE(solution.has_value());
  EXPECT_EQ(solution->sample_count, 1);
}

TEST(AhrsMath, AngularRateMountingRejectionIsUnchanged)
{
  AhrsMountingConfig config;
  config.calibration_duration_sec = 0.0;
  config.stationary_angular_speed_threshold_rads = 0.35;
  config.min_sample_count = 1;
  BootMountingCalibrator calibrator(config);

  EXPECT_FALSE(
      calibrator
          .AddGravitySample(0, Eigen::Vector3d(0.0, 0.0, -9.81), Eigen::Vector3d(0.0, 0.0, 0.36))
          .has_value());
  const std::optional<AhrsMountingSolution> solution = calibrator.AddGravitySample(
      1, Eigen::Vector3d(0.0, 0.0, -9.81), Eigen::Vector3d(0.0, 0.0, 0.35));
  ASSERT_TRUE(solution.has_value());
  EXPECT_EQ(solution->sample_count, 1);
}

TEST(AhrsMath, GravityResidualRejectsInverseAttitudeDirection)
{
  const Eigen::Quaterniond q_WB = QuaternionFromRollPitchYaw(0.42, -0.33, 0.61);
  const Eigen::Vector3d gravity_base = q_WB.conjugate() * Eigen::Vector3d(0.0, 0.0, -9.81);

  const std::optional<AhrsGravityResidual> matching =
      ComputeGravityResidual(gravity_base, std::nullopt, q_WB);
  const std::optional<AhrsGravityResidual> inconsistent =
      ComputeGravityResidual(gravity_base, std::nullopt, q_WB.conjugate());

  ASSERT_TRUE(matching.has_value());
  ASSERT_TRUE(inconsistent.has_value());
  EXPECT_NEAR(matching->residual_norm, 0.0, 1.0e-12);
  EXPECT_GT(inconsistent->residual_norm, 0.1);
}

TEST(AhrsMath, MountingCompositionFollowsBaseToImuToWorldChain)
{
  const Eigen::Quaterniond q_WI = QuaternionFromRollPitchYaw(-0.31, 0.22, M_PI / 2.0);
  const Eigen::Quaterniond q_BI = QuaternionFromRollPitchYaw(0.19, -0.28, 0.0);
  const Eigen::Quaterniond q_IB = q_BI.conjugate();

  const Eigen::Quaterniond q_WB = ComposeWorldFromBase(q_WI, q_BI);
  const Eigen::Vector3d vector_base(0.37, -0.41, 0.83);
  const Eigen::Vector3d chained = q_WI * (q_IB * vector_base);

  ExpectVectorNear(q_WB * vector_base, chained);
  EXPECT_NEAR(q_WB.norm(), 1.0, 1.0e-12);

  const Eigen::Vector3d legacy = q_BI * (q_WI * vector_base);
  EXPECT_GT((legacy - chained).norm(), 0.1);
  EXPECT_GT(((q_WI * q_BI) * vector_base - chained).norm(), 0.1);
  EXPECT_GT(((q_IB * q_WI) * vector_base - chained).norm(), 0.1);
}

TEST(AhrsMath, QuaternionInputNormalizationPreservesImuToWorldDirectionAndRejectsInvalidValues)
{
  const Eigen::Quaterniond q_WI = QuaternionFromRollPitchYaw(0.37, -0.29, 0.81);
  const Eigen::Quaterniond scaled(2.7 * q_WI.w(), 2.7 * q_WI.x(), 2.7 * q_WI.y(), 2.7 * q_WI.z());

  const std::optional<Eigen::Quaterniond> normalized = NormalizeQuaternion(scaled);
  ASSERT_TRUE(normalized.has_value());
  ExpectVectorNear(*normalized * Eigen::Vector3d(0.2, -0.6, 0.7),
                   q_WI * Eigen::Vector3d(0.2, -0.6, 0.7));
  EXPECT_FALSE(NormalizeQuaternion(Eigen::Quaterniond(0.0, 0.0, 0.0, 0.0)).has_value());
  EXPECT_FALSE(NormalizeQuaternion(
                   Eigen::Quaterniond(1.0, std::numeric_limits<double>::infinity(), 0.0, 0.0))
                   .has_value());
}

TEST(AhrsMath, MountingQuaternionInverseHasOppositeVisibleRotation)
{
  const Eigen::Quaterniond q_BI = QuaternionFromRollPitchYaw(0.35, -0.24, 0.0);
  const Eigen::Quaterniond q_IB = q_BI.conjugate();
  const Eigen::Vector3d vector_imu(0.6, -0.2, 0.7);

  ExpectVectorNear(q_IB * (q_BI * vector_imu), vector_imu);
  EXPECT_NEAR(q_IB.x(), -q_BI.x(), 1.0e-12);
  EXPECT_NEAR(q_IB.y(), -q_BI.y(), 1.0e-12);
  EXPECT_NEAR(q_IB.z(), -q_BI.z(), 1.0e-12);
  EXPECT_NEAR(q_IB.w(), q_BI.w(), 1.0e-12);
}

TEST(AhrsMath, MountedAttitudePreservesNegativeNinetyDegreeYawDirection)
{
  const Eigen::Quaterniond q_BI = QuaternionFromRollPitchYaw(-0.21, 0.34, 0.0);
  const Eigen::Quaterniond expected_q_WB = QuaternionFromRollPitchYaw(0.18, -0.29, -M_PI / 2.0);
  const Eigen::Quaterniond q_WI = expected_q_WB * q_BI;

  const Eigen::Quaterniond q_WB = ComposeWorldFromBase(q_WI, q_BI);

  ExpectVectorNear(q_WB * Eigen::Vector3d::UnitX(), expected_q_WB * Eigen::Vector3d::UnitX());
  EXPECT_NEAR(YawFromQuaternion(q_WB), -M_PI / 2.0, 1.0e-12);
}

TEST(AhrsMath, SessionYawInitializesToZeroAndPreservesTilt)
{
  const Eigen::Quaterniond q_WB = QuaternionFromRollPitchYaw(0.29, -0.37, M_PI / 2.0);
  const double initial_yaw = YawFromQuaternion(q_WB);
  const Eigen::Quaterniond q_OW = QuaternionFromYaw(-initial_yaw);

  const Eigen::Quaterniond q_OB = ComposeOdomFromBase(q_OW, q_WB);

  EXPECT_NEAR(YawFromQuaternion(q_OB), 0.0, 1.0e-12);
  const Eigen::Vector3d base_z(0.0, 0.0, 1.0);
  ExpectVectorNear(q_OB * base_z, q_OW * (q_WB * base_z));
  EXPECT_NEAR(q_OB.norm(), 1.0, 1.0e-12);
}

TEST(AhrsMath, SessionYawRetainsRelativeYawSign)
{
  const Eigen::Quaterniond q_WB_initial = QuaternionFromRollPitchYaw(0.0, 0.0, M_PI / 2.0);
  const Eigen::Quaterniond q_OW = QuaternionFromYaw(-YawFromQuaternion(q_WB_initial));
  const Eigen::Quaterniond q_WB_later = QuaternionFromRollPitchYaw(0.0, 0.0, M_PI / 4.0);

  const Eigen::Quaterniond q_OB = ComposeOdomFromBase(q_OW, q_WB_later);

  EXPECT_NEAR(YawFromQuaternion(q_OB), -M_PI / 4.0, 1.0e-12);
  const Eigen::Vector3d odom_x = q_OB * Eigen::Vector3d::UnitX();
  ExpectVectorNear(odom_x, Eigen::Vector3d(std::sqrt(0.5), -std::sqrt(0.5), 0.0));
}
