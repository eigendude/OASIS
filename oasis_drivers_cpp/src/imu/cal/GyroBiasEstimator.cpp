/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "imu/cal/GyroBiasEstimator.h"

#include <algorithm>

namespace OASIS::IMU
{
namespace
{
Eigen::Matrix3d Symmetrize(const Eigen::Matrix3d& m)
{
  return 0.5 * (m + m.transpose());
}
} // namespace

void GyroBiasEstimator::Reset()
{
  m_initialized = false;
  m_b = Eigen::Vector3d::Zero();
  m_P = Eigen::Matrix3d::Zero();
}

void GyroBiasEstimator::Update(const Vec3& z_mean_rads,
                               const Mat3& cov_sample_rads2_2,
                               std::size_t window_count)
{
  if (window_count == 0)
    return;

  const double n = static_cast<double>(std::max<std::size_t>(1, window_count));

  Eigen::Vector3d z = ToEigen(z_mean_rads);
  Eigen::Matrix3d R = ToEigen(cov_sample_rads2_2) / n;

  R = Symmetrize(R);

  // Units: (rad/s)^2, diagonal jitter to avoid singular R
  constexpr double kDiagJitter = 1e-12;

  R += kDiagJitter * Eigen::Matrix3d::Identity();

  if (!m_initialized)
  {
    m_b = z;
    m_P = R;
    m_initialized = true;
    return;
  }

  const Eigen::Matrix3d S = m_P + R;
  const Eigen::Matrix3d K = m_P * S.inverse();

  m_b = m_b + K * (z - m_b);

  const Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
  const Eigen::Matrix3d IK = I - K;
  m_P = IK * m_P * IK.transpose() + K * R * K.transpose();
  m_P = Symmetrize(m_P);
}

bool GyroBiasEstimator::IsInitialized() const
{
  return m_initialized;
}

Vec3 GyroBiasEstimator::GetBias() const
{
  return FromEigen(m_b);
}

Mat3 GyroBiasEstimator::GetCov() const
{
  return FromEigen(m_P);
}

Eigen::Vector3d GyroBiasEstimator::ToEigen(const Vec3& v)
{
  return {v[0], v[1], v[2]};
}

Eigen::Matrix3d GyroBiasEstimator::ToEigen(const Mat3& m)
{
  Eigen::Matrix3d out = Eigen::Matrix3d::Zero();
  for (std::size_t i = 0; i < 3; ++i)
  {
    for (std::size_t j = 0; j < 3; ++j)
      out(i, j) = m[i][j];
  }
  return out;
}

Vec3 GyroBiasEstimator::FromEigen(const Eigen::Vector3d& v)
{
  return {v[0], v[1], v[2]};
}

Mat3 GyroBiasEstimator::FromEigen(const Eigen::Matrix3d& m)
{
  Mat3 out{};
  for (std::size_t i = 0; i < 3; ++i)
  {
    for (std::size_t j = 0; j < 3; ++j)
      out[i][j] = m(i, j);
  }
  return out;
}
} // namespace OASIS::IMU
