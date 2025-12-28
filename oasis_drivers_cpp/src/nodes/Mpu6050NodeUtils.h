/*
 *  Copyright (C) 2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#pragma once

#include "Mpu6050Node.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <vector>

namespace OASIS::ROS::Mpu6050NodeUtils
{
using Vec3 = Mpu6050Node::Vec3;
using Mapping = Mpu6050Node::Mapping;
using Quaternion = Mpu6050Node::Quaternion;

// IMU parameters
inline constexpr double GRAVITY = 9.80665; // m/s^2
inline constexpr double ACCEL_SCALE = GRAVITY / 16384.0; // +/-2g full scale
inline constexpr double GYRO_SCALE = (M_PI / 180.0) / 131.0; // +/-250 deg/s full scale
inline constexpr double Z_UP_SCORE_GRAVITY_PENALTY = 1.0;

// Utility helpers

double AccelScaleFromRange(uint8_t range);

double GyroScaleFromRange(uint8_t range);

Vec3 Add(const Vec3& a, const Vec3& b);

Vec3 Sub(const Vec3& a, const Vec3& b);

Vec3 Scale(const Vec3& a, double s);

double Dot(const Vec3& a, const Vec3& b);

Vec3 Cross(const Vec3& a, const Vec3& b);

double Norm(const Vec3& a);

Vec3 Normalize(const Vec3& a);

Vec3 ApplyMapping(const Mapping& mapping, const Vec3& v);

Mapping MultiplyMapping(const Mapping& a, const Mapping& b);

Mapping TransposeMapping(const Mapping& m);

Quaternion NormalizeQuat(const Quaternion& q);

Quaternion MultiplyQuat(const Quaternion& a, const Quaternion& b);

Quaternion ConjugateQuat(const Quaternion& q);

Vec3 RotateVec(const Quaternion& q, const Vec3& v);

Quaternion QuatFromAxisAngle(const Vec3& axis, double angle);

Quaternion QuatFromEuler(double roll, double pitch, double yaw);

Quaternion Slerp(const Quaternion& a, const Quaternion& b, double t);

Vec3 EulerFromQuat(const Quaternion& q);

Quaternion QuatFromRotationMatrix(const Mapping& m);

std::vector<Mapping> GenerateMappings();

double EwmaAlpha(double dt, double tau);

void EwmaUpdate(double x, double alpha, double& mean, double& var, bool& initialized);

} // namespace OASIS::ROS::Mpu6050NodeUtils
