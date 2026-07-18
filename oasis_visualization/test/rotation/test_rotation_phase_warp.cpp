/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "rotation/rotation_phase_warp.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>

#include <gtest/gtest.h>

namespace OASIS::Visualization
{
namespace
{
constexpr double PI = 3.14159265358979323846;
constexpr double TWO_PI = 2.0 * PI;
constexpr double DEFAULT_NONLINEARITY = 0.75;

TEST(RotationPhaseWarp, ZeroNonlinearityReproducesLinearAngleExactly)
{
  for (int step = 0; step <= 100; ++step)
  {
    const double phase = static_cast<double>(step) / 100.0;
    EXPECT_DOUBLE_EQ(CalculateRotationAngle(phase, 0.0), TWO_PI * phase);
  }
}

TEST(RotationPhaseWarp, QuarterRevolutionAnglesAreFixed)
{
  EXPECT_DOUBLE_EQ(CalculateRotationAngle(0.0, DEFAULT_NONLINEARITY), 0.0);
  EXPECT_NEAR(CalculateRotationAngle(0.25, DEFAULT_NONLINEARITY), PI / 2.0,
              std::numeric_limits<double>::epsilon());
  EXPECT_NEAR(CalculateRotationAngle(0.5, DEFAULT_NONLINEARITY), PI,
              std::numeric_limits<double>::epsilon());
  EXPECT_NEAR(CalculateRotationAngle(0.75, DEFAULT_NONLINEARITY), 3.0 * PI / 2.0,
              std::numeric_limits<double>::epsilon());
}

TEST(RotationPhaseWarp, CompletesExactlyOneRevolution)
{
  constexpr double revolutionSeconds = 8.0;
  const double phaseAtOneRevolution = revolutionSeconds / revolutionSeconds;
  EXPECT_DOUBLE_EQ(CalculateRotationAngle(phaseAtOneRevolution, DEFAULT_NONLINEARITY), TWO_PI);

  constexpr double finalPhase = 1.0 - 1.0e-9;
  EXPECT_LT(CalculateRotationAngle(finalPhase, DEFAULT_NONLINEARITY), TWO_PI);
  EXPECT_NEAR(CalculateRotationAngle(finalPhase, DEFAULT_NONLINEARITY), TWO_PI, 2.0e-9);
}

TEST(RotationPhaseWarp, IsStrictlyMonotonicAtDefaultNonlinearity)
{
  double previousAngle = CalculateRotationAngle(0.0, DEFAULT_NONLINEARITY);
  for (int step = 1; step <= 10000; ++step)
  {
    const double phase = static_cast<double>(step) / 10000.0;
    const double angle = CalculateRotationAngle(phase, DEFAULT_NONLINEARITY);
    EXPECT_GT(angle, previousAngle);
    previousAngle = angle;
  }
}

TEST(RotationPhaseWarp, VelocityExtremaMatchImageOrientations)
{
  const double minimumVelocity = 1.0 - DEFAULT_NONLINEARITY;
  const double maximumVelocity = 1.0 + DEFAULT_NONLINEARITY;

  EXPECT_DOUBLE_EQ(CalculateRelativeAngularVelocity(0.0, DEFAULT_NONLINEARITY), minimumVelocity);
  EXPECT_DOUBLE_EQ(CalculateRelativeAngularVelocity(0.5, DEFAULT_NONLINEARITY), minimumVelocity);
  EXPECT_DOUBLE_EQ(CalculateRelativeAngularVelocity(0.25, DEFAULT_NONLINEARITY), maximumVelocity);
  EXPECT_DOUBLE_EQ(CalculateRelativeAngularVelocity(0.75, DEFAULT_NONLINEARITY), maximumVelocity);
}

TEST(RotationPhaseWarp, RejectsInvalidNonlinearity)
{
  const double invalidValues[] = {
      -std::numeric_limits<double>::epsilon(),      1.0,
      1.0 + std::numeric_limits<double>::epsilon(), std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::infinity(),
  };

  for (const double value : invalidValues)
  {
    EXPECT_FALSE(IsValidRotationNonlinearity(value));
    EXPECT_THROW(CalculateRotationAngle(0.0, value), std::invalid_argument);
  }
}
} // namespace
} // namespace OASIS::Visualization
