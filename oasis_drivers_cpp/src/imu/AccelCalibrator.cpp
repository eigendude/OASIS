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
// Phase1 avoids the |a|≈g chicken-and-egg before bias/scale are learned.
constexpr double kStationaryGateTauSeconds = 3.0;
constexpr double kStationaryClipSigma = 3.0;
constexpr double kStationarySigmaFloorGyro = 0.01;
constexpr double kStationarySigmaFloorDeltaA = 0.03;
constexpr double kStationarySigmaCapGyro = kStationaryGyroTolRadS;
constexpr double kStationarySigmaCapDeltaA = kStationaryAccelTolG * kG;
constexpr double kStationaryGyroAbsMaxRadS = 3.0 * kStationaryGyroTolRadS;
constexpr double kStationaryAccelHpAbsMaxMps2 = 3.0 * kStationaryAccelTolG * kG;
constexpr double kStationaryScoreEnter = 6.0;
constexpr double kStationaryScoreExit = 12.0;
constexpr double kStationaryDwellDecayRate = 2.0;
constexpr double kStationaryFallbackDtSeconds = 0.02;
constexpr double kStationaryMaxDtSeconds = 0.1;

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

constexpr bool kUseBootNoiseCalib = true;
constexpr double kBootNoiseCalibSeconds = 2.0;
constexpr std::size_t kBootNoiseCalibMinSamples = 50;
constexpr bool kAllowStationaryNoiseUpdate = false;

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

void AccelCalibrator::OnlineStats::UpdateWinsor(double x,
                                                double alpha,
                                                double sigma_floor,
                                                double sigma_cap,
                                                double k_clip)
{
  const double sigma = Sigma(sigma_floor);
  const double clamp_min = mu - k_clip * sigma;
  const double clamp_max = mu + k_clip * sigma;
  const double x_w = std::clamp(x, clamp_min, clamp_max);

  if (!inited)
  {
    mu = x_w;
    var = std::min(sigma_floor * sigma_floor, sigma_cap * sigma_cap);
    inited = true;
    n = 1;
    return;
  }

  ++n;
  const double delta = x_w - mu;
  mu += alpha * delta;
  const double var_update = delta * delta - var;
  var = std::max(var + alpha * var_update, sigma_floor * sigma_floor);
  var = std::min(var, sigma_cap * sigma_cap);
}

double AccelCalibrator::OnlineStats::Sigma(double sigma_floor) const
{
  return std::max(std::sqrt(var), sigma_floor);
}

double AccelCalibrator::OnlineStats::Z(double x, double sigma_floor) const
{
  const double sigma = Sigma(sigma_floor);
  return (x - mu) / sigma;
}

bool AccelCalibrator::OnlineStats::Ready(std::size_t min_n) const
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

bool AccelCalibrator::UpdateStationaryDwell(bool stationary_candidate,
                                            bool use_leaky,
                                            double dt_seconds)
{
  const double dwell_target =
      std::clamp(kStationaryDwellSeconds, kStationaryDwellMinSeconds, kStationaryDwellMaxSeconds);

  double dt = dt_seconds;
  if (dt <= 0.0)
    dt = kStationaryFallbackDtSeconds;
  dt = std::clamp(dt, 0.0, kStationaryMaxDtSeconds);

  if (!stationary_candidate)
  {
    if (use_leaky)
      m_stationary_dwell_seconds =
          std::max(0.0, m_stationary_dwell_seconds - kStationaryDwellDecayRate * dt);
    else
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

  const double dt_clamped =
      std::clamp(dt_seconds, kStationaryFallbackDtSeconds, kStationaryMaxDtSeconds);
  const double alpha = 1.0 - std::exp(-dt_clamped / kStationaryGateTauSeconds);
  m_noise_calib_elapsed += dt_clamped;

  const double omega = Norm3(gyro_rads);
  double delta_a = 0.0;
  bool have_delta_a = false;
  if (m_have_prev_accel)
  {
    std::array<double, 3> delta{0.0, 0.0, 0.0};
    for (std::size_t i = 0; i < 3; ++i)
      delta[i] = accel_mps2[i] - m_prev_accel_raw_mps2[i];
    delta_a = Norm3(delta);
    have_delta_a = true;
  }

  const double z_a = have_delta_a ? m_delta_a_stats.Z(delta_a, kStationarySigmaFloorDeltaA) : 0.0;
  const double z_g = m_omega_stats.Z(omega, kStationarySigmaFloorGyro);
  const double score = (z_a * z_a) + (z_g * z_g);

  if (!m_stationary_gate_active)
  {
    const bool stats_ready = m_omega_stats.Ready(kBootNoiseCalibMinSamples) &&
                             m_delta_a_stats.Ready(kBootNoiseCalibMinSamples);
    if (stats_ready && score < kStationaryScoreEnter && have_delta_a)
      m_stationary_gate_active = true;
  }
  else if (score > kStationaryScoreExit)
  {
    m_stationary_gate_active = false;
  }

  const bool precal_stationary = m_stationary_gate_active && have_delta_a;

  m_prev_accel_raw_mps2 = accel_mps2;
  m_have_prev_accel = true;

  std::array<double, 3> accel_corr{0.0, 0.0, 0.0};
  for (std::size_t i = 0; i < 3; ++i)
  {
    const double scale_abs = std::max(std::abs(m_scale[i]), kEps);
    accel_corr[i] = (accel_mps2[i] - m_bias[i]) / scale_abs;
  }

  const bool strict_stationary = IsStrictStationary(accel_corr, gyro_rads);
  const bool phase2_active = m_uniform_scale_initialized;
  // Phase 1 uses gyro + accel first-difference only; phase 2 requires corrected |a|≈g.
  const bool dwell_predicate = phase2_active ? strict_stationary : precal_stationary;
  d.stationary = dwell_predicate;
  d.stationary_precal = precal_stationary;

  const double accel_norm = Norm3(accel_mps2);
  d.stationary_strict = strict_stationary;
  d.stationary_phase2 = phase2_active;
  const bool stationary_confirmed =
      UpdateStationaryDwell(dwell_predicate, !phase2_active, dt_seconds);
  d.stationary_confirmed = stationary_confirmed;
  d.stationary_dwell_seconds = m_stationary_dwell_seconds;
  d.stationary_dwell_target_seconds =
      std::clamp(kStationaryDwellSeconds, kStationaryDwellMinSeconds, kStationaryDwellMaxSeconds);

  const bool stats_ready = m_omega_stats.Ready(kBootNoiseCalibMinSamples) &&
                           m_delta_a_stats.Ready(kBootNoiseCalibMinSamples);
  const bool noise_calib_done =
      !kUseBootNoiseCalib ||
      (m_noise_calib_elapsed >= kBootNoiseCalibSeconds && stats_ready);
  const bool noise_calib_active = kUseBootNoiseCalib && !noise_calib_done;
  const bool boot_force_update = noise_calib_active && !stats_ready;
  const bool noise_update_allowed =
      noise_calib_active || (kAllowStationaryNoiseUpdate && m_stationary_confirmed);
  const bool gyro_bounds_ok = omega <= kStationaryGyroAbsMaxRadS;
  const bool delta_a_bounds_ok = have_delta_a && (delta_a <= kStationaryAccelHpAbsMaxMps2);
  const bool bounds_ok = gyro_bounds_ok && (!have_delta_a || delta_a_bounds_ok);
  const bool noise_score_ok = score < kStationaryScoreEnter;

  if (boot_force_update)
  {
    // Boot-time: force updates while within abs bounds to prevent score-gate deadlock.
    if (gyro_bounds_ok)
      m_omega_stats.UpdateWinsor(omega, alpha, kStationarySigmaFloorGyro, kStationarySigmaCapGyro,
                                 kStationaryClipSigma);
    if (have_delta_a && delta_a_bounds_ok)
      m_delta_a_stats.UpdateWinsor(delta_a, alpha, kStationarySigmaFloorDeltaA,
                                   kStationarySigmaCapDeltaA, kStationaryClipSigma);
  }
  else if (noise_update_allowed && bounds_ok && (noise_score_ok || strict_stationary))
  {
    m_omega_stats.UpdateWinsor(omega, alpha, kStationarySigmaFloorGyro, kStationarySigmaCapGyro,
                               kStationaryClipSigma);
    if (have_delta_a)
      m_delta_a_stats.UpdateWinsor(delta_a, alpha, kStationarySigmaFloorDeltaA,
                                   kStationarySigmaCapDeltaA, kStationaryClipSigma);
  }

  // Initialize uniform scale from the first stationary window to avoid large start-up errors.
  if (stationary_confirmed && !m_uniform_scale_initialized)
  {
    m_stationary_norm_mean.Add(accel_norm);
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
      r[i] = (accel_mps2[i] - m_bias[i]) / scale_abs;
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
  d.omega_mean = m_omega_stats.mu;
  d.omega_sigma = m_omega_stats.Sigma(kStationarySigmaFloorGyro);
  d.omega_stats_inited = m_omega_stats.inited;
  d.delta_a_mean = m_delta_a_stats.mu;
  d.delta_a_sigma = m_delta_a_stats.Sigma(kStationarySigmaFloorDeltaA);
  d.delta_a_stats_inited = m_delta_a_stats.inited;
  d.noise_calib_active = noise_calib_active;
  d.noise_calib_done = noise_calib_done;
  d.stationarity_score = score;

  return d;
}
} // namespace OASIS::IMU
