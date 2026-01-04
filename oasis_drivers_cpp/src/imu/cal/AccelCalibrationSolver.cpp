/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "imu/cal/AccelCalibrationSolver.h"

#include <cmath>
#include <iomanip>
#include <iostream>

#include <Eigen/Dense>

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
constexpr double kMinEllipsoidEigenvalue = 1e-12;

// Units: 1/(m/s^2)^2. Negative tolerance for eigenvalues due to numeric error
constexpr double kEigenNegTolerance = 1e-12;

#ifndef OASIS_IMU_ACCEL_CAL_DEBUG
#define OASIS_IMU_ACCEL_CAL_DEBUG 0
#endif

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

  if constexpr (OASIS_IMU_ACCEL_CAL_DEBUG)
  {
    const Eigen::VectorXd singular_values = svd.singularValues();
    const double max_sv = singular_values.size() > 0 ? singular_values(0) : 0.0;
    const double min_sv =
        singular_values.size() > 0 ? singular_values(singular_values.size() - 1) : 0.0;

    std::cerr << std::fixed << std::setprecision(6);
    std::cerr << "[AccelCal] SVD singular values min=" << min_sv << " max=" << max_sv
              << " count=" << singular_values.size() << "\n";
  }

  Eigen::Matrix3d M;
  M << v(0), v(3), v(4), v(3), v(1), v(5), v(4), v(5), v(2);

  const Eigen::Vector3d g(v(6), v(7), v(8));
  const double r = v(9);

  auto try_solution = [&](double sign, Eigen::Vector3d& center_out, Eigen::Matrix3d& Q_out,
                          double& k_out, Eigen::Vector3d& eigenvalues_out,
                          Eigen::Matrix3d& eigenvectors_out) -> bool
  {
    const Eigen::Matrix3d M_signed = sign * M;
    const Eigen::Vector3d g_signed = sign * g;
    const double r_signed = sign * r;

    const Eigen::FullPivLU<Eigen::Matrix3d> lu(M_signed);
    if constexpr (OASIS_IMU_ACCEL_CAL_DEBUG)
    {
      std::cerr << std::fixed << std::setprecision(6);
      std::cerr << "[AccelCal] Sign " << sign << " LU rank=" << lu.rank()
                << " det=" << lu.determinant() << "\n";
    }

    if (!lu.isInvertible())
      return false;

    center_out = -lu.solve(g_signed);

    // Units: (m/s^2)^2. Ellipsoid normalization scale from center substitution
    k_out = (center_out.transpose() * M_signed * center_out)(0, 0) - r_signed;
    if constexpr (OASIS_IMU_ACCEL_CAL_DEBUG)
    {
      std::cerr << std::fixed << std::setprecision(6);
      std::cerr << "[AccelCal] Sign " << sign << " center=[" << center_out.transpose()
                << "] k=" << k_out << "\n";
    }

    if (k_out <= 0.0)
      return false;

    Q_out = M_signed / k_out;
    Q_out = 0.5 * (Q_out + Q_out.transpose());

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen_solver(Q_out);
    if (eigen_solver.info() != Eigen::Success)
      return false;

    const Eigen::Vector3d eigenvalues_raw = eigen_solver.eigenvalues();
    eigenvectors_out = eigen_solver.eigenvectors();

    if constexpr (OASIS_IMU_ACCEL_CAL_DEBUG)
    {
      std::cerr << std::fixed << std::setprecision(6);
      std::cerr << "[AccelCal] Sign " << sign << " eigenvalues raw=[" << eigenvalues_raw.transpose()
                << "]\n";
    }

    // Reject truly indefinite Q while tolerating tiny negative noise
    if ((eigenvalues_raw.array() < -kEigenNegTolerance).any())
    {
      if constexpr (OASIS_IMU_ACCEL_CAL_DEBUG)
      {
        std::cerr << std::fixed << std::setprecision(6);
        std::cerr << "[AccelCal] Sign " << sign << " SPD failed eigenvalues=["
                  << eigenvalues_raw.transpose() << "]\n";
      }
      return false;
    }

    // Clamp tiny negatives to keep sqrt stable for near-zero eigenvalues
    eigenvalues_out = eigenvalues_raw.cwiseMax(kMinEllipsoidEigenvalue);

    if constexpr (OASIS_IMU_ACCEL_CAL_DEBUG)
    {
      std::cerr << std::fixed << std::setprecision(6);
      std::cerr << "[AccelCal] Sign " << sign << " eigenvalues clamped=["
                << eigenvalues_out.transpose() << "]\n";
    }

    return true;
  };

  Eigen::Vector3d center;
  Eigen::Matrix3d Q;
  double k = 0.0;
  Eigen::Vector3d eigenvalues;
  Eigen::Matrix3d eigenvectors;
  double chosen_sign = 1.0;

  if (!try_solution(1.0, center, Q, k, eigenvalues, eigenvectors))
  {
    if (!try_solution(-1.0, center, Q, k, eigenvalues, eigenvectors))
      return false;
    chosen_sign = -1.0;
  }

  if constexpr (OASIS_IMU_ACCEL_CAL_DEBUG)
  {
    std::cerr << std::fixed << std::setprecision(6);
    std::cerr << "[AccelCal] Chosen sign=" << chosen_sign << "\n";
  }

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

  const double rms_residual_mps2 = std::sqrt(residual_sum_sq / static_cast<double>(sample_count));

  if constexpr (OASIS_IMU_ACCEL_CAL_DEBUG)
  {
    std::cerr << std::fixed << std::setprecision(6);
    std::cerr << "[AccelCal] Success sign=" << chosen_sign << " k=" << k << " eigenvalues=["
              << eigenvalues.transpose() << "] rms=" << rms_residual_mps2 << "\n";
  }

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
