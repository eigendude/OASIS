/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "rotation/rotation_phase_warp.hpp"

#include <cmath>
#include <stdexcept>

namespace OASIS::Visualization
{
namespace
{
constexpr double TWO_PI = 6.28318530717958647692;

void ValidatePhase(double phase)
{
  if (!std::isfinite(phase) || phase < 0.0 || phase > 1.0)
    throw std::invalid_argument("rotation phase must be finite and in the range [0, 1]");
}

void ValidateNonlinearity(double nonlinearity)
{
  if (!IsValidRotationNonlinearity(nonlinearity))
  {
    throw std::invalid_argument("rotation nonlinearity must be finite and in the range [0, 1)");
  }
}
} // namespace

bool IsValidRotationNonlinearity(double nonlinearity)
{
  return std::isfinite(nonlinearity) && nonlinearity >= 0.0 && nonlinearity < 1.0;
}

double CalculateRotationAngle(double phase, double nonlinearity)
{
  ValidatePhase(phase);
  ValidateNonlinearity(nonlinearity);

  const double linearAngle = TWO_PI * phase;
  if (phase == 0.0 || phase == 1.0)
    return linearAngle;

  // The second harmonic has zero displacement at each quarter revolution.
  // Its derivative varies speed smoothly between 1-k and 1+k while its zero
  // net integral preserves the exact revolution period
  return linearAngle - 0.5 * nonlinearity * std::sin(2.0 * linearAngle);
}

double CalculateRelativeAngularVelocity(double phase, double nonlinearity)
{
  ValidatePhase(phase);
  ValidateNonlinearity(nonlinearity);

  const double linearAngle = TWO_PI * phase;

  return 1.0 - nonlinearity * std::cos(2.0 * linearAngle);
}
} // namespace OASIS::Visualization
