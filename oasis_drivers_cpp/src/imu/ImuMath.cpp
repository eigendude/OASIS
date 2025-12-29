/*
 *  Copyright (C) 2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "imu/ImuMath.h"

#include <algorithm>
#include <cmath>

namespace OASIS::IMU::Math
{
namespace
{
constexpr double kEps = 1e-9;
} // namespace

double Dot(const std::array<double, 3>& a, const std::array<double, 3>& b)
{
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

double Norm(const std::array<double, 3>& v)
{
  return std::sqrt(Dot(v, v));
}

std::array<double, 3> Add(const std::array<double, 3>& a, const std::array<double, 3>& b)
{
  return {a[0] + b[0], a[1] + b[1], a[2] + b[2]};
}

std::array<double, 3> Subtract(const std::array<double, 3>& a, const std::array<double, 3>& b)
{
  return {a[0] - b[0], a[1] - b[1], a[2] - b[2]};
}

std::array<double, 3> Scale(const std::array<double, 3>& v, double s)
{
  return {v[0] * s, v[1] * s, v[2] * s};
}

std::array<double, 3> Cross(const std::array<double, 3>& a, const std::array<double, 3>& b)
{
  return {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2],
          a[0] * b[1] - a[1] * b[0]};
}

std::array<double, 3> Normalize(const std::array<double, 3>& v, double eps)
{
  const double norm = Norm(v);
  if (norm <= std::max(eps, kEps))
    return {0.0, 0.0, 0.0};

  return {v[0] / norm, v[1] / norm, v[2] / norm};
}

Quaternion Normalize(const Quaternion& q, double eps)
{
  const double norm = std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
  if (norm <= std::max(eps, kEps))
    return Quaternion{};

  return Quaternion{q.w / norm, q.x / norm, q.y / norm, q.z / norm};
}

Quaternion Multiply(const Quaternion& lhs, const Quaternion& rhs)
{
  return Quaternion{
      lhs.w * rhs.w - lhs.x * rhs.x - lhs.y * rhs.y - lhs.z * rhs.z,
      lhs.w * rhs.x + lhs.x * rhs.w + lhs.y * rhs.z - lhs.z * rhs.y,
      lhs.w * rhs.y - lhs.x * rhs.z + lhs.y * rhs.w + lhs.z * rhs.x,
      lhs.w * rhs.z + lhs.x * rhs.y - lhs.y * rhs.x + lhs.z * rhs.w};
}

Quaternion FromAxisAngle(const std::array<double, 3>& axis_unit, double angle_rad)
{
  const double half = 0.5 * angle_rad;
  const double s = std::sin(half);
  return Normalize(Quaternion{std::cos(half), axis_unit[0] * s, axis_unit[1] * s, axis_unit[2] * s});
}

Quaternion FromWorldAxesInSensor(const std::array<double, 3>& x_world_in_sensor,
                                 const std::array<double, 3>& y_world_in_sensor,
                                 const std::array<double, 3>& z_world_in_sensor)
{
  const double r00 = x_world_in_sensor[0];
  const double r01 = x_world_in_sensor[1];
  const double r02 = x_world_in_sensor[2];
  const double r10 = y_world_in_sensor[0];
  const double r11 = y_world_in_sensor[1];
  const double r12 = y_world_in_sensor[2];
  const double r20 = z_world_in_sensor[0];
  const double r21 = z_world_in_sensor[1];
  const double r22 = z_world_in_sensor[2];

  const double trace = r00 + r11 + r22;
  Quaternion q;

  if (trace > 0.0)
  {
    const double s = std::sqrt(trace + 1.0) * 2.0;
    q.w = 0.25 * s;
    q.x = (r21 - r12) / s;
    q.y = (r02 - r20) / s;
    q.z = (r10 - r01) / s;
  }
  else if (r00 > r11 && r00 > r22)
  {
    const double s = std::sqrt(1.0 + r00 - r11 - r22) * 2.0;
    q.w = (r21 - r12) / s;
    q.x = 0.25 * s;
    q.y = (r01 + r10) / s;
    q.z = (r02 + r20) / s;
  }
  else if (r11 > r22)
  {
    const double s = std::sqrt(1.0 + r11 - r00 - r22) * 2.0;
    q.w = (r02 - r20) / s;
    q.x = (r01 + r10) / s;
    q.y = 0.25 * s;
    q.z = (r12 + r21) / s;
  }
  else
  {
    const double s = std::sqrt(1.0 + r22 - r00 - r11) * 2.0;
    q.w = (r10 - r01) / s;
    q.x = (r02 + r20) / s;
    q.y = (r12 + r21) / s;
    q.z = 0.25 * s;
  }

  return Normalize(q);
}
} // namespace OASIS::IMU::Math
