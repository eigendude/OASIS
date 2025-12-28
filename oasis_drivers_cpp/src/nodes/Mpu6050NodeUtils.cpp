/*
 *  Copyright (C) 2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "Mpu6050NodeUtils.h"

#include <algorithm>
#include <cmath>

namespace OASIS::ROS::Mpu6050NodeUtils
{

double AccelScaleFromRange(uint8_t range)
{
  switch (range)
  {
    case MPU6050_ACCEL_FS_2:
      return GRAVITY / 16384.0;
    case MPU6050_ACCEL_FS_4:
      return GRAVITY / 8192.0;
    case MPU6050_ACCEL_FS_8:
      return GRAVITY / 4096.0;
    case MPU6050_ACCEL_FS_16:
      return GRAVITY / 2048.0;
    default:
      return ACCEL_SCALE;
  }
}

double GyroScaleFromRange(uint8_t range)
{
  switch (range)
  {
    case MPU6050_GYRO_FS_250:
      return (M_PI / 180.0) / 131.0;
    case MPU6050_GYRO_FS_500:
      return (M_PI / 180.0) / 65.5;
    case MPU6050_GYRO_FS_1000:
      return (M_PI / 180.0) / 32.8;
    case MPU6050_GYRO_FS_2000:
      return (M_PI / 180.0) / 16.4;
    default:
      return GYRO_SCALE;
  }
}

Vec3 Add(const Vec3& a, const Vec3& b)
{
  return {a[0] + b[0], a[1] + b[1], a[2] + b[2]};
}

Vec3 Sub(const Vec3& a, const Vec3& b)
{
  return {a[0] - b[0], a[1] - b[1], a[2] - b[2]};
}

Vec3 Scale(const Vec3& a, double s)
{
  return {a[0] * s, a[1] * s, a[2] * s};
}

double Dot(const Vec3& a, const Vec3& b)
{
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

Vec3 Cross(const Vec3& a, const Vec3& b)
{
  return {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2],
          a[0] * b[1] - a[1] * b[0]};
}

double Norm(const Vec3& a)
{
  return std::sqrt(Dot(a, a));
}

Vec3 Normalize(const Vec3& a)
{
  const double n = Norm(a);
  if (n <= 1e-9)
    return {0.0, 0.0, 0.0};
  return Scale(a, 1.0 / n);
}

Vec3 ApplyMapping(const Mapping& mapping, const Vec3& v)
{
  Vec3 out{0.0, 0.0, 0.0};
  for (size_t i = 0; i < 3; ++i)
  {
    double sum = 0.0;
    for (size_t j = 0; j < 3; ++j)
      sum += static_cast<double>(mapping[i][j]) * v[j];
    out[i] = sum;
  }
  return out;
}

Mapping MultiplyMapping(const Mapping& a, const Mapping& b)
{
  Mapping out{{{0, 0, 0}, {0, 0, 0}, {0, 0, 0}}};
  for (size_t i = 0; i < 3; ++i)
  {
    for (size_t j = 0; j < 3; ++j)
    {
      int sum = 0;
      for (size_t k = 0; k < 3; ++k)
        sum += a[i][k] * b[k][j];
      out[i][j] = sum;
    }
  }
  return out;
}

Mapping TransposeMapping(const Mapping& m)
{
  Mapping out{{{0, 0, 0}, {0, 0, 0}, {0, 0, 0}}};
  for (size_t i = 0; i < 3; ++i)
  {
    for (size_t j = 0; j < 3; ++j)
      out[i][j] = m[j][i];
  }
  return out;
}

Quaternion NormalizeQuat(const Quaternion& q)
{
  const double n = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
  if (n <= 1e-9)
    return {1.0, 0.0, 0.0, 0.0};
  return {q[0] / n, q[1] / n, q[2] / n, q[3] / n};
}

Quaternion MultiplyQuat(const Quaternion& a, const Quaternion& b)
{
  return {a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
          a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
          a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
          a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0]};
}

Quaternion ConjugateQuat(const Quaternion& q)
{
  return {q[0], -q[1], -q[2], -q[3]};
}

Vec3 RotateVec(const Quaternion& q, const Vec3& v)
{
  const Quaternion vq{0.0, v[0], v[1], v[2]};
  const Quaternion rq = MultiplyQuat(MultiplyQuat(q, vq), ConjugateQuat(q));
  return {rq[1], rq[2], rq[3]};
}

Quaternion QuatFromAxisAngle(const Vec3& axis, double angle)
{
  const double half = angle * 0.5;
  const double s = std::sin(half);
  return NormalizeQuat({std::cos(half), axis[0] * s, axis[1] * s, axis[2] * s});
}

Quaternion QuatFromEuler(double roll, double pitch, double yaw)
{
  const double cr = std::cos(roll * 0.5);
  const double sr = std::sin(roll * 0.5);
  const double cp = std::cos(pitch * 0.5);
  const double sp = std::sin(pitch * 0.5);
  const double cy = std::cos(yaw * 0.5);
  const double sy = std::sin(yaw * 0.5);
  return {cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
          cr * cp * sy - sr * sp * cy};
}

Quaternion Slerp(const Quaternion& a, const Quaternion& b, double t)
{
  if (t <= 0.0)
    return a;
  if (t >= 1.0)
    return b;
  double cosTheta = a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3];
  Quaternion bb = b;
  if (cosTheta < 0.0)
  {
    cosTheta = -cosTheta;
    bb = {-b[0], -b[1], -b[2], -b[3]};
  }
  if (cosTheta > 0.9995)
  {
    const Quaternion out{a[0] + t * (bb[0] - a[0]), a[1] + t * (bb[1] - a[1]),
                         a[2] + t * (bb[2] - a[2]), a[3] + t * (bb[3] - a[3])};
    return NormalizeQuat(out);
  }
  const double angle = std::acos(std::clamp(cosTheta, -1.0, 1.0));
  const double s = std::sin(angle);
  const double wa = std::sin((1.0 - t) * angle) / s;
  const double wb = std::sin(t * angle) / s;
  return {a[0] * wa + bb[0] * wb, a[1] * wa + bb[1] * wb, a[2] * wa + bb[2] * wb,
          a[3] * wa + bb[3] * wb};
}

Vec3 EulerFromQuat(const Quaternion& q)
{
  const double sinr = 2.0 * (q[0] * q[1] + q[2] * q[3]);
  const double cosr = 1.0 - 2.0 * (q[1] * q[1] + q[2] * q[2]);
  const double roll = std::atan2(sinr, cosr);
  const double sinp = 2.0 * (q[0] * q[2] - q[3] * q[1]);
  const double pitch = std::abs(sinp) >= 1.0 ? std::copysign(M_PI / 2.0, sinp) : std::asin(sinp);
  const double siny = 2.0 * (q[0] * q[3] + q[1] * q[2]);
  const double cosy = 1.0 - 2.0 * (q[2] * q[2] + q[3] * q[3]);
  const double yaw = std::atan2(siny, cosy);
  return {roll, pitch, yaw};
}

Quaternion QuatFromRotationMatrix(const Mapping& m)
{
  const double trace = static_cast<double>(m[0][0] + m[1][1] + m[2][2]);
  Quaternion q{1.0, 0.0, 0.0, 0.0};
  if (trace > 0.0)
  {
    const double s = std::sqrt(trace + 1.0) * 2.0;
    q[0] = 0.25 * s;
    q[1] = (m[2][1] - m[1][2]) / s;
    q[2] = (m[0][2] - m[2][0]) / s;
    q[3] = (m[1][0] - m[0][1]) / s;
  }
  else if (m[0][0] > m[1][1] && m[0][0] > m[2][2])
  {
    const double s = std::sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]) * 2.0;
    q[0] = (m[2][1] - m[1][2]) / s;
    q[1] = 0.25 * s;
    q[2] = (m[0][1] + m[1][0]) / s;
    q[3] = (m[0][2] + m[2][0]) / s;
  }
  else if (m[1][1] > m[2][2])
  {
    const double s = std::sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]) * 2.0;
    q[0] = (m[0][2] - m[2][0]) / s;
    q[1] = (m[0][1] + m[1][0]) / s;
    q[2] = 0.25 * s;
    q[3] = (m[1][2] + m[2][1]) / s;
  }
  else
  {
    const double s = std::sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]) * 2.0;
    q[0] = (m[1][0] - m[0][1]) / s;
    q[1] = (m[0][2] + m[2][0]) / s;
    q[2] = (m[1][2] + m[2][1]) / s;
    q[3] = 0.25 * s;
  }
  return NormalizeQuat(q);
}

std::vector<Mapping> GenerateMappings()
{
  std::vector<Mapping> mappings;
  std::array<int, 3> perm{{0, 1, 2}};
  auto parity = [](const std::array<int, 3>& p)
  {
    int inv = 0;
    for (int i = 0; i < 3; ++i)
    {
      for (int j = i + 1; j < 3; ++j)
      {
        if (p[i] > p[j])
          ++inv;
      }
    }
    return (inv % 2 == 0) ? 1 : -1;
  };
  do
  {
    const int permParity = parity(perm);
    for (int sx : {-1, 1})
    {
      for (int sy : {-1, 1})
      {
        for (int sz : {-1, 1})
        {
          const int det = permParity * sx * sy * sz;
          if (det != 1)
            continue;
          Mapping m{{{0, 0, 0}, {0, 0, 0}, {0, 0, 0}}};
          m[0][perm[0]] = sx;
          m[1][perm[1]] = sy;
          m[2][perm[2]] = sz;
          mappings.push_back(m);
        }
      }
    }
  } while (std::next_permutation(perm.begin(), perm.end()));
  return mappings;
}

double EwmaAlpha(double dt, double tau)
{
  if (tau <= 1e-6)
    return 1.0;
  return std::clamp(dt / (tau + dt), 0.0, 1.0);
}

void EwmaUpdate(double x, double alpha, double& mean, double& var, bool& initialized)
{
  if (!initialized)
  {
    mean = x;
    var = 0.0;
    initialized = true;
    return;
  }
  const double delta = x - mean;
  mean += alpha * delta;
  var = (1.0 - alpha) * (var + alpha * delta * delta);
}

} // namespace OASIS::ROS::Mpu6050NodeUtils
