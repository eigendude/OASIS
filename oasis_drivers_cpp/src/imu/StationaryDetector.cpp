/*
 *  Copyright (C) 2025-2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "imu/StationaryDetector.h"

#include <algorithm>
#include <cmath>

namespace OASIS::IMU
{
namespace
{
void Symmetrize(Mat3& mat)
{
  // Units: unitless
  // Meaning: average upper/lower triangle to enforce symmetry
  const double sym_scale = 0.5;

  for (std::size_t i = 0; i < 3; ++i)
  {
    for (std::size_t j = i + 1; j < 3; ++j)
    {
      const double sym = sym_scale * (mat[i][j] + mat[j][i]);
      mat[i][j] = sym;
      mat[j][i] = sym;
    }
  }
}

double Dot(const Vec3& a, const Vec3& b)
{
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

double Norm(const Vec3& a)
{
  return std::sqrt(Dot(a, a));
}
} // namespace

void StationaryDetector::Configure(const Config& cfg)
{
  m_cfg = cfg;
}

void StationaryDetector::Reset()
{
  m_window.clear();
  m_gyro_bias_iir = {0.0, 0.0, 0.0};
  m_gyro_bias_iir_init = false;
}

StationaryDetector::Status StationaryDetector::Update(const ImuSample& sample, const Noise& noise)
{
  Status out{};

  if (!m_window.empty())
  {
    const double dt = sample.stamp_s - m_window.back().stamp_s;
    if (dt <= 0.0 || dt > m_cfg.max_sample_gap_s)
    {
      Reset();
    }
  }

  m_window.push_back(sample);
  if (m_window.size() > m_cfg.window_size)
    m_window.erase(m_window.begin());

  out.window_count = m_window.size();

  double window_duration_s = 0.0;
  if (m_window.size() >= 2)
    window_duration_s = m_window.back().stamp_s - m_window.front().stamp_s;

  out.window_duration_s = window_duration_s;

  if (m_window.size() < m_cfg.window_size)
    return out;

  if (window_duration_s < m_cfg.min_window_duration_s)
    return out;

  const double n = static_cast<double>(m_window.size());

  for (const auto& s : m_window)
  {
    for (std::size_t i = 0; i < 3; ++i)
    {
      out.mean_accel_mps2[i] += s.accel_mps2[i];
      out.mean_gyro_rads[i] += s.gyro_rads[i];
    }
  }
  for (std::size_t i = 0; i < 3; ++i)
  {
    out.mean_accel_mps2[i] /= n;
    out.mean_gyro_rads[i] /= n;
  }

  Mat3 cov_accel_sum{};
  Mat3 cov_gyro_sum{};
  for (const auto& s : m_window)
  {
    for (std::size_t i = 0; i < 3; ++i)
    {
      const double da_i = s.accel_mps2[i] - out.mean_accel_mps2[i];
      const double dg_i = s.gyro_rads[i] - out.mean_gyro_rads[i];
      for (std::size_t j = 0; j < 3; ++j)
      {
        const double da_j = s.accel_mps2[j] - out.mean_accel_mps2[j];
        const double dg_j = s.gyro_rads[j] - out.mean_gyro_rads[j];
        cov_accel_sum[i][j] += da_i * da_j;
        cov_gyro_sum[i][j] += dg_i * dg_j;
      }
    }
  }

  Symmetrize(cov_accel_sum);
  Symmetrize(cov_gyro_sum);

  Mat3 cov_accel = cov_accel_sum;
  Mat3 cov_gyro = cov_gyro_sum;

  // Units: samples
  // Meaning: unbiased sample covariance normalization (n - 1)
  const double denom = std::max(1.0, n - 1.0);

  for (std::size_t i = 0; i < 3; ++i)
  {
    for (std::size_t j = 0; j < 3; ++j)
    {
      cov_accel[i][j] /= denom;
      cov_gyro[i][j] /= denom;
    }
  }

  Symmetrize(cov_accel);
  Symmetrize(cov_gyro);

  out.cov_gyro_rads2_2 = cov_gyro;

  // Units: (m/s^2)^2
  // Meaning: accel variance floor to avoid zero/noise-free gating
  const double accel_var_floor = 1e-12;

  // Units: (rad/s)^2
  // Meaning: gyro variance floor to avoid zero/noise-free gating
  const double gyro_var_floor = 1e-12;

  for (std::size_t axis = 0; axis < 3; ++axis)
  {
    const double accel_var_gate =
        std::max(noise.accel_cov_mps2_2[axis][axis], accel_var_floor) * m_cfg.variance_sigma_mult;

    const double gyro_var_gate =
        std::max(noise.gyro_cov_rads2_2[axis][axis], gyro_var_floor) * m_cfg.variance_sigma_mult;

    if (cov_accel[axis][axis] > accel_var_gate)
      return out;

    if (cov_gyro[axis][axis] > gyro_var_gate)
      return out;
  }

  if (!m_gyro_bias_iir_init)
  {
    m_gyro_bias_iir = out.mean_gyro_rads;
    m_gyro_bias_iir_init = true;
  }
  else
  {
    // Units: unitless
    // Meaning: IIR smoothing factor for gyro bias estimate
    const double alpha = 0.001;

    for (std::size_t axis = 0; axis < 3; ++axis)
      m_gyro_bias_iir[axis] =
          (1.0 - alpha) * m_gyro_bias_iir[axis] + alpha * out.mean_gyro_rads[axis];
  }

  for (std::size_t axis = 0; axis < 3; ++axis)
  {
    const double sigma = std::sqrt(std::max(noise.gyro_cov_rads2_2[axis][axis], gyro_var_floor));

    // Units: samples
    // Meaning: lower bound for mean variance scaling to avoid divide-by-zero
    const double mean_count_floor = 1.0;

    const double sigma_mean = std::max(sigma / std::sqrt(std::max(mean_count_floor, n)),
                                       m_cfg.gyro_sigma_mean_floor_rads);

    const double mean_unbiased = out.mean_gyro_rads[axis] - m_gyro_bias_iir[axis];
    if (std::abs(mean_unbiased) > m_cfg.gyro_mean_sigma_mult * sigma_mean)
      return out;
  }

  out.stationary = true;
  (void)Norm;
  return out;
}
} // namespace OASIS::IMU
