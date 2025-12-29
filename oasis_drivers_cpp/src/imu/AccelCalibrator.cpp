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
constexpr double kAccelLpfTauSeconds = 1.0;
constexpr double kStationaryClipSigma = 3.0;
constexpr double kStationarySigmaFloorGyro = 0.01;
constexpr double kStationarySigmaFloorAccelHp = 0.03;
constexpr double kStationarySigmaCapGyro = 0.5;
constexpr double kStationarySigmaCapAccelHp = 1.0;
constexpr double kStationaryGyroAbsMaxRadS = 0.35;
constexpr double kStationaryAccelHpAbsMaxMps2 = 0.5;
constexpr double kStationaryScoreEnter = 6.0;
constexpr double kStationaryScoreExit = 12.0;
constexpr double kNoiseUpdateScoreMax = 4.0;
constexpr double kStationaryDwellDecayRate = 2.0;
constexpr double kStationaryFallbackDtSeconds = 0.02;
constexpr double kStationaryMaxDtSeconds = 0.1;
constexpr double kBootNoiseCalibSeconds = 5.0;
constexpr std::size_t kBootNoiseCalibMinSamples = 50;
constexpr bool kUseBootNoiseCalib = true;
constexpr bool kAllowStationaryNoiseUpdate = false;

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

void AccelCalibrator::OnlineStats::UpdateWinsor(double x,
                                                double alpha,
                                                double sigma_floor,
                                                double sigma_cap,
                                                double k_clip)
{
  const double sigma = Sigma(sigma_floor, sigma_cap);
  const double clamp_min = mu - k_clip * sigma;
  const double clamp_max = mu + k_clip * sigma;
  const double x_w = std::clamp(x, clamp_min, clamp_max);

  if (!inited)
  {
    mu = x_w;
    var = sigma_floor * sigma_floor;
    inited = true;
    n = 1;
    return;
  }

  ++n;
  const double delta = x_w - mu;
  mu += alpha * delta;
  const double var_update = delta * delta - var;
  var = std::max(var + alpha * var_update, sigma_floor * sigma_floor);
}

double AccelCalibrator::OnlineStats::Sigma(double sigma_floor, double sigma_cap) const
{
  const double sigma = std::max(std::sqrt(var), sigma_floor);
  return std::clamp(sigma, sigma_floor, sigma_cap);
}

double AccelCalibrator::OnlineStats::Z(double x, double sigma_floor, double sigma_cap) const
{
  const double sigma = Sigma(sigma_floor, sigma_cap);
  return (x - mu) / sigma;
}

bool AccelCalibrator::OnlineStats::Ready(std::size_t min_samples) const
{
  return n >= min_samples;
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
  const double alpha_stats = 1.0 - std::exp(-dt_clamped / kStationaryGateTauSeconds);
  const double alpha_lpf = 1.0 - std::exp(-dt_clamped / kAccelLpfTauSeconds);

  if (!m_have_accel_lpf)
  {
    m_accel_lpf_mps2 = accel_mps2;
    m_have_accel_lpf = true;
  }
  else
  {
    for (std::size_t i = 0; i < 3; ++i)
      m_accel_lpf_mps2[i] += alpha_lpf * (accel_mps2[i] - m_accel_lpf_mps2[i]);
  }

  std::array<double, 3> accel_hp{0.0, 0.0, 0.0};
  for (std::size_t i = 0; i < 3; ++i)
    accel_hp[i] = accel_mps2[i] - m_accel_lpf_mps2[i];

  std::array<double, 3> accel_corr{0.0, 0.0, 0.0};
  for (std::size_t i = 0; i < 3; ++i)
  {
    const double scale_abs = std::max(std::abs(m_scale[i]), kEps);
    accel_corr[i] = (accel_mps2[i] - m_bias[i]) / scale_abs;
  }
  const bool strict_stationary = IsStrictStationary(accel_corr, gyro_rads);

  m_noise_calib_elapsed += dt_clamped;
  const bool noise_calib_active = kUseBootNoiseCalib && (m_noise_calib_elapsed < kBootNoiseCalibSeconds);
  const bool noise_calib_done = !kUseBootNoiseCalib || (m_noise_calib_elapsed >= kBootNoiseCalibSeconds);

  bool gyro_bounds_ok = true;
  bool accel_bounds_ok = true;
  for (std::size_t i = 0; i < 3; ++i)
  {
    gyro_bounds_ok = gyro_bounds_ok && (std::abs(gyro_rads[i]) < kStationaryGyroAbsMaxRadS);
    accel_bounds_ok = accel_bounds_ok && (std::abs(accel_hp[i]) < kStationaryAccelHpAbsMaxMps2);
  }

  std::array<double, 3> z_g{0.0, 0.0, 0.0};
  std::array<double, 3> z_a{0.0, 0.0, 0.0};
  for (std::size_t i = 0; i < 3; ++i)
  {
    z_g[i] = m_gyro_stats[i].Z(gyro_rads[i], kStationarySigmaFloorGyro, kStationarySigmaCapGyro);
    z_a[i] = m_accel_hp_stats[i].Z(accel_hp[i], kStationarySigmaFloorAccelHp,
                                   kStationarySigmaCapAccelHp);
  }
  double score = 0.0;
  for (std::size_t i = 0; i < 3; ++i)
    score += (z_g[i] * z_g[i]) + (z_a[i] * z_a[i]);

  const bool noise_score_ok = score < kNoiseUpdateScoreMax;
  const bool noise_update_allowed =
      (noise_calib_active || (kAllowStationaryNoiseUpdate && m_stationary_confirmed));

  if (noise_update_allowed && gyro_bounds_ok && accel_bounds_ok &&
      (noise_score_ok || strict_stationary))
  {
    for (std::size_t i = 0; i < 3; ++i)
    {
      m_gyro_stats[i].UpdateWinsor(gyro_rads[i], alpha_stats, kStationarySigmaFloorGyro,
                                   kStationarySigmaCapGyro, kStationaryClipSigma);
      m_accel_hp_stats[i].UpdateWinsor(accel_hp[i], alpha_stats, kStationarySigmaFloorAccelHp,
                                       kStationarySigmaCapAccelHp, kStationaryClipSigma);
    }
  }

  bool stats_ready = true;
  for (std::size_t i = 0; i < 3; ++i)
  {
    stats_ready = stats_ready && m_gyro_stats[i].Ready(kBootNoiseCalibMinSamples) &&
                  m_accel_hp_stats[i].Ready(kBootNoiseCalibMinSamples);
  }

  if (!m_stationary_gate_active)
  {
    if (stats_ready && score < kStationaryScoreEnter)
      m_stationary_gate_active = true;
  }
  else if (score > kStationaryScoreExit || !stats_ready)
  {
    m_stationary_gate_active = false;
  }

  const bool precal_stationary = m_stationary_gate_active;

  const bool phase2_active = m_uniform_scale_initialized;
  // Phase 1 uses gyro + accel high-pass only; phase 2 requires corrected |a|≈g.
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
  d.stationarity_score = score;
  d.noise_calib_active = noise_calib_active;
  d.noise_calib_done = noise_calib_done;
  for (std::size_t i = 0; i < 3; ++i)
  {
    d.gyro_mu[i] = m_gyro_stats[i].mu;
    d.gyro_sigma[i] =
        m_gyro_stats[i].Sigma(kStationarySigmaFloorGyro, kStationarySigmaCapGyro);
    d.gyro_inited[i] = m_gyro_stats[i].Ready(kBootNoiseCalibMinSamples);
    d.accel_hp_mu[i] = m_accel_hp_stats[i].mu;
    d.accel_hp_sigma[i] =
        m_accel_hp_stats[i].Sigma(kStationarySigmaFloorAccelHp, kStationarySigmaCapAccelHp);
    d.accel_hp_inited[i] = m_accel_hp_stats[i].Ready(kBootNoiseCalibMinSamples);
  }

  return d;
}
} // namespace OASIS::IMU
