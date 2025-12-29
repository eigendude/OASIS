/*
 *  Copyright (C) 2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "imu/AccelCalibrator.h"

#include <algorithm>
#include <cmath>

namespace OASIS::IMU
{
namespace
{
constexpr double kG = 9.80665;

// Low-pass filter for accel (Hz). Keeps stationary detection stable under vibration.
constexpr double kAccelLpCutoffHz = 2.5;
constexpr double kTwoPi = 6.283185307179586;

// Stationary thresholds: low gyro rate + low LP accel jerk.
constexpr double kGyroStationaryRadS = 0.20; // ~11 deg/s
constexpr double kJerkStationaryMps3 = 2.5; // based on LP accel delta

// Sample requirements: enough stationary samples for stable means.
constexpr std::size_t kMinStationarySamplesForInit = 30;

// Safety clamps
constexpr double kScaleMin = 0.5;
constexpr double kScaleMax = 2.0;
constexpr double kBiasClampMps2 = 5.0;

// Online update rates (small for stability).
constexpr double kAlphaBias = 0.003;
constexpr double kAlphaScale = 0.0002;
constexpr double kBiasLeak = 0.0005;
constexpr double kScaleLeak = 0.0002;

constexpr double kCoverageCosMin = 0.75;

constexpr double kEps = 1e-9;
} // namespace

AccelCalibrator::AccelCalibrator() = default;

const std::array<double, 3>& AccelCalibrator::GetBias() const
{
  return m_bias;
}

const std::array<double, 3>& AccelCalibrator::GetScale() const
{
  return m_scale;
}

void AccelCalibrator::RunningMean::Add(double x)
{
  ++n;
  mean += (x - mean) / static_cast<double>(n);
}

bool AccelCalibrator::RunningMean::Ready(std::size_t min_n) const
{
  return n >= min_n;
}

double AccelCalibrator::Norm3(const std::array<double, 3>& v)
{
  return std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

AccelCalibrator::Diagnostics AccelCalibrator::Update(const std::array<double, 3>& accel_mps2,
                                                     const std::array<double, 3>& gyro_rads,
                                                     double dt_seconds)
{
  Diagnostics d;

  if (dt_seconds <= 0.0)
    dt_seconds = 0.02;

  // Low-pass filter the accelerometer to make "face" detection robust.
  const double lp_alpha = 1.0 - std::exp(-dt_seconds * kAccelLpCutoffHz * kTwoPi);

  if (!m_initialized)
  {
    m_accel_lp = accel_mps2;
    m_prev_accel_lp = accel_mps2;
    m_initialized = true;
  }
  else
  {
    for (std::size_t i = 0; i < 3; ++i)
      m_accel_lp[i] = m_accel_lp[i] + lp_alpha * (accel_mps2[i] - m_accel_lp[i]);
  }

  // Stationary check: gyro magnitude + jerk of LP accel (NO g_err involved).
  const double gyro_mag = Norm3(gyro_rads);

  std::array<double, 3> jerk{0.0, 0.0, 0.0};
  for (std::size_t i = 0; i < 3; ++i)
    jerk[i] = (m_accel_lp[i] - m_prev_accel_lp[i]) / dt_seconds;
  const double jerk_mag = Norm3(jerk);

  const bool stationary = (gyro_mag < kGyroStationaryRadS) && (jerk_mag < kJerkStationaryMps3);
  d.stationary = stationary;

  const double accel_lp_norm = Norm3(m_accel_lp);

  // Initialize uniform scale from the first stationary window to avoid large start-up errors.
  if (stationary && !m_uniform_scale_initialized)
  {
    m_stationary_norm_mean.Add(accel_lp_norm);
    if (m_stationary_norm_mean.Ready(kMinStationarySamplesForInit))
    {
      const double s0 = std::clamp(m_stationary_norm_mean.mean / kG, kScaleMin, kScaleMax);
      m_scale = {s0, s0, s0};
      m_uniform_scale_initialized = true;
    }
  }

  if (stationary && accel_lp_norm > kEps)
  {
    std::array<double, 3> r{0.0, 0.0, 0.0};
    for (std::size_t i = 0; i < 3; ++i)
    {
      const double scale_abs = std::max(std::abs(m_scale[i]), kEps);
      r[i] = (m_accel_lp[i] - m_bias[i]) / scale_abs;
    }

    const double r_norm = Norm3(r);
    if (r_norm > kEps)
    {
      const double e = r_norm - kG;
      std::array<double, 3> unit{0.0, 0.0, 0.0};
      for (std::size_t i = 0; i < 3; ++i)
      {
        unit[i] = r[i] / r_norm;
      }

      // Gradient step on bias from gravity magnitude error.
      for (std::size_t i = 0; i < 3; ++i)
      {
        const double scale_abs = std::max(std::abs(m_scale[i]), kEps);
        m_bias[i] += kAlphaBias * e * (unit[i] / scale_abs);
      }

      // Coverage tracks whether we've seen +g and -g along each axis direction.
      for (std::size_t i = 0; i < 3; ++i)
      {
        if (unit[i] > kCoverageCosMin)
          m_pos_seen[i] = true;
        if (unit[i] < -kCoverageCosMin)
          m_neg_seen[i] = true;
      }

      // Scale learning is gated on coverage to avoid bias-only solutions
      // when gravity has only been observed from one side.
      for (std::size_t i = 0; i < 3; ++i)
      {
        if (!(m_pos_seen[i] && m_neg_seen[i]))
          continue;

        const double scale_abs = std::max(std::abs(m_scale[i]), kEps);
        m_scale[i] += kAlphaScale * e * (r[i] * r[i]) / (scale_abs * r_norm);
      }
    }
  }

  // Apply leaks and clamps for stability.
  for (std::size_t i = 0; i < 3; ++i)
  {
    m_bias[i] *= (1.0 - kBiasLeak);
    m_bias[i] = std::clamp(m_bias[i], -kBiasClampMps2, kBiasClampMps2);

    m_scale[i] += kScaleLeak * (1.0 - m_scale[i]);
    m_scale[i] = std::clamp(m_scale[i], kScaleMin, kScaleMax);
  }

  // Emit diagnostics
  d.bias_mps2 = m_bias;
  d.scale = m_scale;
  d.pos_seen = m_pos_seen;
  d.neg_seen = m_neg_seen;

  m_prev_accel_lp = m_accel_lp;
  return d;
}
} // namespace OASIS::IMU
