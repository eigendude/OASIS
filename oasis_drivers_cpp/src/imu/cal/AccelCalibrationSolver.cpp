/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "imu/cal/AccelCalibrationSolver.h"

#include <Eigen/Dense>

#include <cmath>

namespace OASIS::IMU
{
namespace
{
constexpr std::size_t kMinSamples = 50;
constexpr std::size_t kMaxSamples = 5000;

// Units: (m/s^2)^2. Conservative variance for accel bias estimate
constexpr double kAccelBiasVarianceMps2_2 = 0.05 * 0.05;

// Units: unitless^2. Conservative variance for accel matrix estimate
constexpr double kAccelMatrixVariance = 0.01 * 0.01;

// Units: 1/(m/s^2)^2. Minimum eigenvalue for a valid ellipsoid
constexpr double kMinEllipsoidEigenvalue = 1e-9;

Eigen::Vector3d ToEigen(const Vec3& value)
{
  return {value[0], value[1], value[2]};
}

Vec3 ToVec3(const Eigen::Vector3d& value)
{
  return {value.x(), value.y(), value.z()};
}

Mat3 ToMat3(const Eigen::Matrix3d& value)
{
  Mat3 out{};
  for (int row = 0; row < 3; ++row)
  {
    for (int col = 0; col < 3; ++col)
      out[row][col] = value(row, col);
  }

  return out;
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

  const std::size_t sample_count = m_samples.size();
  Eigen::MatrixXd design(sample_count, 10);
  for (std::size_t i = 0; i < sample_count; ++i)
  {
    const Vec3& sample = m_samples[i];
    const double x = sample[0];
    const double y = sample[1];
    const double z = sample[2];

    design(static_cast<Eigen::Index>(i), 0) = x * x;
    design(static_cast<Eigen::Index>(i), 1) = y * y;
    design(static_cast<Eigen::Index>(i), 2) = z * z;
    design(static_cast<Eigen::Index>(i), 3) = 2.0 * x * y;
    design(static_cast<Eigen::Index>(i), 4) = 2.0 * x * z;
    design(static_cast<Eigen::Index>(i), 5) = 2.0 * y * z;
    design(static_cast<Eigen::Index>(i), 6) = 2.0 * x;
    design(static_cast<Eigen::Index>(i), 7) = 2.0 * y;
    design(static_cast<Eigen::Index>(i), 8) = 2.0 * z;
    design(static_cast<Eigen::Index>(i), 9) = 1.0;
  }

  Eigen::JacobiSVD<Eigen::MatrixXd> svd(design, Eigen::ComputeThinV);
  if (svd.matrixV().cols() < 10)
    return false;

  const Eigen::VectorXd v = svd.matrixV().col(9);
  if (v.size() != 10)
    return false;

  Eigen::Matrix3d M;
  M << v(0), v(3), v(4), v(3), v(1), v(5), v(4), v(5), v(2);

  const Eigen::Vector3d p(v(6), v(7), v(8));
  const double r = v(9);

  const Eigen::FullPivLU<Eigen::Matrix3d> lu(M);
  if (!lu.isInvertible())
    return false;

  const Eigen::Vector3d center = -0.5 * lu.solve(p);

  // Units: (m/s^2)^2. Ellipsoid normalization scale from center substitution
  const double k = (center.transpose() * M * center)(0, 0) - r;
  if (k <= 0.0)
    return false;

  Eigen::Matrix3d Q = M / k;
  Q = 0.5 * (Q + Q.transpose());

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen_solver(Q);
  if (eigen_solver.info() != Eigen::Success)
    return false;

  const Eigen::Vector3d eigenvalues = eigen_solver.eigenvalues();
  if ((eigenvalues.array() <= kMinEllipsoidEigenvalue).any())
    return false;

  const Eigen::Matrix3d eigenvectors = eigen_solver.eigenvectors();
  const Eigen::Vector3d sqrt_eigenvalues = eigenvalues.array().sqrt();
  const Eigen::Matrix3d sqrt_Q =
      eigenvectors * sqrt_eigenvalues.asDiagonal() * eigenvectors.transpose();

  const Eigen::Matrix3d accel_A = gravity_mps2 * sqrt_Q;

  double residual_sum_sq = 0.0;
  for (const Vec3& sample : m_samples)
  {
    const Eigen::Vector3d centered = ToEigen(sample) - center;
    const Eigen::Vector3d calibrated = accel_A * centered;
    const double residual = std::abs(calibrated.norm() - gravity_mps2);
    residual_sum_sq += residual * residual;
  }

  const double rms_residual_mps2 =
      std::sqrt(residual_sum_sq / static_cast<double>(sample_count));

  out.accel_bias_mps2 = ToVec3(center);
  out.accel_A = ToMat3(accel_A);
  out.accel_param_cov = {};
  for (std::size_t i = 0; i < 3; ++i)
    out.accel_param_cov[i][i] = kAccelBiasVarianceMps2_2;
  for (std::size_t i = 3; i < 12; ++i)
    out.accel_param_cov[i][i] = kAccelMatrixVariance;
  out.rms_residual_mps2 = rms_residual_mps2;
  out.ellipsoid.center_mps2 = ToVec3(center);
  out.ellipsoid.Q = ToMat3(Q);
  out.sample_count = static_cast<std::uint32_t>(sample_count);

  return true;
}
} // namespace OASIS::IMU
