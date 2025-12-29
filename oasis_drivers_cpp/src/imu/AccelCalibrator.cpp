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
constexpr double kStationaryDwellSeconds = 0.75;
constexpr double kStationaryDwellMinSeconds = 0.5;
constexpr double kStationaryDwellMaxSeconds = 1.0;
constexpr double kStationaryAccelTolG = 0.06;
constexpr double kStationaryGyroTolRadS = 0.10;
constexpr double kStationaryFallbackDtSeconds = 0.02;
constexpr double kStationaryMaxDtSeconds = 0.1;

// Low-pass filter for accel (Hz). Keeps jerk metric stable under vibration.
constexpr double kAccelLpCutoffHz = 2.5;
constexpr double kTwoPi = 6.283185307179586;

// Stationary thresholds: low gyro rate + low LP accel jerk.
constexpr double kGyroStationaryRadS = 0.20; // ~11 deg/s
constexpr double kJerkStationaryMps3 = 2.5; // based on LP accel delta

// Sample requirements: enough stationary samples for stable mean.
constexpr std::size_t kMinStationarySamplesForInit = 30;

// Online learning rates and stabilization.
constexpr double kAlphaBias = 0.003;
constexpr double kAlphaScale = 0.0002;
constexpr double kBiasLeak = 0.0005;
constexpr double kScaleLeak = 0.0002;

// Safety clamps
constexpr double kScaleMin = 0.5;
constexpr double kScaleMax = 2.0;
constexpr double kBiasClampMps2 = 5.0;

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

void AccelCalibrator::ResetUniformScaleInitialization()
{
  m_uniform_scale_initialized = false;
  m_stationary_norm_mean = RunningMean{};
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

bool AccelCalibrator::IsStrictStationary(const std::array<double, 3>& accel_mps2,
                                         const std::array<double, 3>& gyro_rads) const
{
  const double accel_mag = Norm3(accel_mps2);
  const double gyro_mag = Norm3(gyro_rads);

  const double accel_err = std::abs(accel_mag - kG);
  const bool accel_ok = accel_err < (kStationaryAccelTolG * kG);
  const bool gyro_ok = gyro_mag < kStationaryGyroTolRadS;

  return accel_ok && gyro_ok;
}

bool AccelCalibrator::IsPrecalStationary(bool stationary_motion, double gyro_mag) const
{
  const bool gyro_ok = gyro_mag < kStationaryGyroTolRadS;
  return stationary_motion && gyro_ok;
}

bool AccelCalibrator::UpdateStationaryDwell(bool is_strict, double dt_seconds)
{
  const double dwell_target =
      std::clamp(kStationaryDwellSeconds, kStationaryDwellMinSeconds, kStationaryDwellMaxSeconds);

  double dt = dt_seconds;
  if (dt <= 0.0)
    dt = kStationaryFallbackDtSeconds;
  dt = std::clamp(dt, 0.0, kStationaryMaxDtSeconds);

  if (!is_strict)
  {
    m_stationary_dwell_seconds = 0.0;
    m_stationary_confirmed = false;
    return false;
  }

  m_stationary_dwell_seconds = std::min(m_stationary_dwell_seconds + dt, dwell_target);
  if (m_stationary_dwell_seconds >= dwell_target)
    m_stationary_confirmed = true;

  return m_stationary_confirmed;
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
  const bool strict_stationary = IsStrictStationary(m_accel_lp, gyro_rads);
  const bool precal_stationary = IsPrecalStationary(stationary, gyro_mag);
  const bool phase2_active = m_uniform_scale_initialized;
  d.stationary_strict = strict_stationary;
  d.stationary_phase2 = phase2_active;
  // Two-phase stationary gating avoids the |a|≈g chicken-and-egg before scale init.
  const bool dwell_predicate = phase2_active ? strict_stationary : precal_stationary;
  const bool stationary_confirmed = UpdateStationaryDwell(dwell_predicate, dt_seconds);
  d.stationary_confirmed = stationary_confirmed;
  d.stationary_dwell_seconds = m_stationary_dwell_seconds;
  d.stationary_dwell_target_seconds =
      std::clamp(kStationaryDwellSeconds, kStationaryDwellMinSeconds, kStationaryDwellMaxSeconds);

  // Initialize uniform scale from the first stationary window to avoid large start-up errors.
  if (stationary_confirmed && !m_uniform_scale_initialized)
  {
    m_stationary_norm_mean.Add(accel_lp_norm);
    if (m_stationary_norm_mean.Ready(kMinStationarySamplesForInit))
    {
      const double s0 = std::clamp(m_stationary_norm_mean.mean / kG, kScaleMin, kScaleMax);
      m_scale = {s0, s0, s0};
      m_uniform_scale_initialized = true;
    }
  }

  // Magnitude residual: r = (accel - bias) / scale, e = |r| - g.
  // Scale updates are gated by coverage so the estimator only stretches when
  // gravity has been observed from both directions on an axis.
  if (stationary_confirmed && phase2_active)
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
      std::array<double, 3> unit{
          r[0] / r_norm,
          r[1] / r_norm,
          r[2] / r_norm,
      };

      for (std::size_t i = 0; i < 3; ++i)
      {
        const double scale_abs = std::max(std::abs(m_scale[i]), kEps);
        m_bias[i] += kAlphaBias * e * (unit[i] / scale_abs);
      }

      for (std::size_t i = 0; i < 3; ++i)
      {
        if (unit[i] > 0.75)
          m_pos_seen[i] = true;
        if (unit[i] < -0.75)
          m_neg_seen[i] = true;
      }

      for (std::size_t i = 0; i < 3; ++i)
      {
        if (!(m_pos_seen[i] && m_neg_seen[i]))
          continue;

        const double scale_abs = std::max(std::abs(m_scale[i]), kEps);
        m_scale[i] += kAlphaScale * e * (r[i] * r[i]) / (scale_abs * r_norm);
      }
    }
  }

  for (std::size_t i = 0; i < 3; ++i)
  {
    const bool observable = m_pos_seen[i] && m_neg_seen[i];

    if (observable)
      m_bias[i] *= (1.0 - kBiasLeak);
    m_bias[i] = std::clamp(m_bias[i], -kBiasClampMps2, kBiasClampMps2);

    if (observable)
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
