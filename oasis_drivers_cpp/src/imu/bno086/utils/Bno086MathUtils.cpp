/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "imu/bno086/utils/Bno086MathUtils.hpp"

#include <cmath>

namespace OASIS::IMU::BNO086
{
double QToDouble(std::int16_t value, unsigned q_point)
{
  return static_cast<double>(value) / static_cast<double>(1U << q_point);
}

void NormalizeQuaternion(std::array<double, 4>& q)
{
  const double norm = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);

  if (norm <= 0.0)
  {
    q = {0.0, 0.0, 0.0, 1.0};
    return;
  }

  q[0] /= norm;
  q[1] /= norm;
  q[2] /= norm;
  q[3] /= norm;
}

std::array<double, 4> MultiplyQuaternion(const std::array<double, 4>& lhs,
                                         const std::array<double, 4>& rhs)
{
  return {
      lhs[3] * rhs[0] + lhs[0] * rhs[3] + lhs[1] * rhs[2] - lhs[2] * rhs[1],
      lhs[3] * rhs[1] - lhs[0] * rhs[2] + lhs[1] * rhs[3] + lhs[2] * rhs[0],
      lhs[3] * rhs[2] + lhs[0] * rhs[1] - lhs[1] * rhs[0] + lhs[2] * rhs[3],
      lhs[3] * rhs[3] - lhs[0] * rhs[0] - lhs[1] * rhs[1] - lhs[2] * rhs[2],
  };
}

std::array<double, 4> PredictQuaternion(const std::array<double, 4>& orientation_xyzw,
                                        const OASIS::IMU::Vec3& gyro_rads,
                                        double prediction_horizon_sec)
{
  if (prediction_horizon_sec <= 0.0)
    return orientation_xyzw;

  const double angularSpeed = std::sqrt(gyro_rads[0] * gyro_rads[0] + gyro_rads[1] * gyro_rads[1] +
                                        gyro_rads[2] * gyro_rads[2]);
  if (angularSpeed <= 1e-9)
    return orientation_xyzw;

  const double halfAngle = 0.5 * angularSpeed * prediction_horizon_sec;
  const double sinHalfAngle = std::sin(halfAngle);
  const double invAngularSpeed = 1.0 / angularSpeed;
  std::array<double, 4> deltaQuaternion{
      gyro_rads[0] * invAngularSpeed * sinHalfAngle,
      gyro_rads[1] * invAngularSpeed * sinHalfAngle,
      gyro_rads[2] * invAngularSpeed * sinHalfAngle,
      std::cos(halfAngle),
  };

  std::array<double, 4> predictedOrientation =
      MultiplyQuaternion(orientation_xyzw, deltaQuaternion);
  NormalizeQuaternion(predictedOrientation);
  return predictedOrientation;
}
} // namespace OASIS::IMU::BNO086
