/*
 *  Copyright (C) 2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#pragma once

#include <array>

namespace OASIS::IMU::Math
{
/**
 * @brief Quaternion representing a rotation (sensor -> world).
 */
struct Quaternion
{
  /// Scalar component.
  double w{1.0};
  /// X component of the vector part.
  double x{0.0};
  /// Y component of the vector part.
  double y{0.0};
  /// Z component of the vector part.
  double z{0.0};
};

double Dot(const std::array<double, 3>& a, const std::array<double, 3>& b);
double Norm(const std::array<double, 3>& v);
std::array<double, 3> Add(const std::array<double, 3>& a, const std::array<double, 3>& b);
std::array<double, 3> Subtract(const std::array<double, 3>& a, const std::array<double, 3>& b);
std::array<double, 3> Scale(const std::array<double, 3>& v, double s);
std::array<double, 3> Cross(const std::array<double, 3>& a, const std::array<double, 3>& b);
std::array<double, 3> Normalize(const std::array<double, 3>& v, double eps = 1e-9);

Quaternion Normalize(const Quaternion& q, double eps = 1e-12);
Quaternion Multiply(const Quaternion& lhs, const Quaternion& rhs);
Quaternion FromAxisAngle(const std::array<double, 3>& axis_unit, double angle_rad);
Quaternion FromWorldAxesInSensor(const std::array<double, 3>& x_world_in_sensor,
                                 const std::array<double, 3>& y_world_in_sensor,
                                 const std::array<double, 3>& z_world_in_sensor);
} // namespace OASIS::IMU::Math
