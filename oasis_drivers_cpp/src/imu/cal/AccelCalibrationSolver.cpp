/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "imu/cal/AccelCalibrationSolver.h"

#include <cmath>

namespace OASIS::IMU
{
namespace
{
constexpr std::size_t kMinSamples = 20;
constexpr std::size_t kMaxSamples = 5000;

// Units: (m/s^2)^2. Conservative variance for accel bias estimate
constexpr double kAccelBiasVarianceMps2_2 = 0.05 * 0.05;

// Units: unitless^2. Conservative variance for accel matrix estimate
constexpr double kAccelMatrixVariance = 0.01 * 0.01;

Vec3 Subtract(const Vec3& a, const Vec3& b)
{
  return {a[0] - b[0], a[1] - b[1], a[2] - b[2]};
}

double Dot(const Vec3& a, const Vec3& b)
{
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

double Norm(const Vec3& a)
{
  return std::sqrt(Dot(a, a));
}

Mat3 IdentityMatrix()
{
  return {{{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}}};
}
} // namespace

void AccelCalibrationSolver::Reset()
{
  m_samples.clear();
}

void AccelCalibrationSolver::AddSample(const Vec3& accel_mps2)
{
  m_samples.push_back(accel_mps2);
  if (m_samples.size() > kMaxSamples)
    m_samples.erase(m_samples.begin());
}

std::size_t AccelCalibrationSolver::SampleCount() const
{
  return m_samples.size();
}

bool AccelCalibrationSolver::Solve(double gravity_mps2, Result& out) const
{
  if (m_samples.size() < kMinSamples)
    return false;

  Vec3 mean{0.0, 0.0, 0.0};
  for (const Vec3& sample : m_samples)
  {
    mean[0] += sample[0];
    mean[1] += sample[1];
    mean[2] += sample[2];
  }

  const double sample_count = static_cast<double>(m_samples.size());
  mean[0] /= sample_count;
  mean[1] /= sample_count;
  mean[2] /= sample_count;

  Mat3 accel_A = IdentityMatrix();

  Mat3 ellipsoid_Q{};
  const double gravity_mps2_2 = gravity_mps2 * gravity_mps2;
  const double inv_gravity_mps2_2 = 1.0 / gravity_mps2_2;
  ellipsoid_Q[0][0] = inv_gravity_mps2_2;
  ellipsoid_Q[1][1] = inv_gravity_mps2_2;
  ellipsoid_Q[2][2] = inv_gravity_mps2_2;

  double residual_sum_sq = 0.0;
  for (const Vec3& sample : m_samples)
  {
    const Vec3 centered = Subtract(sample, mean);
    const double residual = Norm(centered) - gravity_mps2;
    residual_sum_sq += residual * residual;
  }

  const double rms_residual_mps2 =
      std::sqrt(residual_sum_sq / sample_count);

  out.accel_bias_mps2 = mean;
  out.accel_A = accel_A;
  out.accel_param_cov = {};
  for (std::size_t i = 0; i < 3; ++i)
    out.accel_param_cov[i][i] = kAccelBiasVarianceMps2_2;
  for (std::size_t i = 3; i < 12; ++i)
    out.accel_param_cov[i][i] = kAccelMatrixVariance;
  out.rms_residual_mps2 = rms_residual_mps2;
  out.ellipsoid.center_mps2 = mean;
  out.ellipsoid.Q = ellipsoid_Q;
  out.sample_count = static_cast<std::uint32_t>(m_samples.size());

  return true;
}
} // namespace OASIS::IMU
