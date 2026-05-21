/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "ahrs/AhrsMath.hpp"

#include <cmath>

#include <gtest/gtest.h>

using namespace OASIS::AHRS;

TEST(AhrsMath, MountingSolveMapsTiltedGravityToBaseNegativeZ)
{
  AhrsMountingConfig config;
  const Eigen::Quaterniond q_tilt = QuaternionFromRollPitchYaw(0.20, -0.35, 0.0);
  const Eigen::Vector3d gravity_imu =
      q_tilt.inverse().toRotationMatrix() * Eigen::Vector3d(0.0, 0.0, -1.0);

  const std::optional<AhrsMountingSolution> solution =
      SolveMountingFromGravity(gravity_imu, config, 12, 2.0);

  ASSERT_TRUE(solution.has_value());
  const Eigen::Vector3d gravity_base = solution->R_BI * gravity_imu;
  EXPECT_NEAR(gravity_base.x(), 0.0, 1.0e-9);
  EXPECT_NEAR(gravity_base.y(), 0.0, 1.0e-9);
  EXPECT_NEAR(gravity_base.z(), -1.0, 1.0e-9);
}

TEST(AhrsMath, MountingYawRemainsZero)
{
  AhrsMountingConfig config;
  const std::optional<AhrsMountingSolution> solution =
      SolveMountingFromGravity(Eigen::Vector3d(0.1, 0.2, -0.97).normalized(), config, 10, 2.0);

  ASSERT_TRUE(solution.has_value());
  EXPECT_NEAR(solution->yaw_rad, 0.0, 1.0e-12);
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

TEST(AhrsMath, GravityResidualMatchesPredictedDirection)
{
  const Eigen::Quaterniond q_WB = Eigen::Quaterniond::Identity();
  const std::optional<AhrsGravityResidual> residual =
      ComputeGravityResidual(Eigen::Vector3d(0.0, 0.0, -9.81), std::nullopt, q_WB);

  ASSERT_TRUE(residual.has_value());
  EXPECT_NEAR(residual->residual_norm, 0.0, 1.0e-12);
}
