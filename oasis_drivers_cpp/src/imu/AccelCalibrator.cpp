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

// Low-pass filter for accel (Hz). Keeps face detection stable under vibration.
constexpr double kAccelLpCutoffHz = 2.5;
constexpr double kTwoPi = 6.283185307179586;

// Stationary thresholds: low gyro rate + low LP accel jerk.
constexpr double kGyroStationaryRadS = 0.20; // ~11 deg/s
constexpr double kJerkStationaryMps3 = 2.5; // based on LP accel delta

// Face detection: dominant axis must be close to ±1 in unit vector.
constexpr double kFaceCosMin = 0.92; // ~23 degrees cone

// Sample requirements: enough stationary samples for stable means.
constexpr std::size_t kMinStationarySamplesForInit = 30;
constexpr std::size_t kMinFaceSamplesPerSide = 60;

// Smoothing when updating solved parameters to avoid step changes.
constexpr double kSolveAlpha = 0.15;

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

  // Face detection (only while stationary): find dominant gravity axis.
  bool face_valid = false;
  std::size_t face_axis = 0;
  int face_sign = 0;
  double face_cos = 0.0;

  if (stationary && accel_lp_norm > kEps)
  {
    std::array<double, 3> u{
        m_accel_lp[0] / accel_lp_norm,
        m_accel_lp[1] / accel_lp_norm,
        m_accel_lp[2] / accel_lp_norm,
    };

    double best = 0.0;
    std::size_t best_axis = 0;
    for (std::size_t i = 0; i < 3; ++i)
    {
      const double a = std::abs(u[i]);
      if (a > best)
      {
        best = a;
        best_axis = i;
      }
    }

    if (best >= kFaceCosMin)
    {
      face_valid = true;
      face_axis = best_axis;
      face_sign = (u[best_axis] >= 0.0) ? +1 : -1;
      face_cos = best;
    }
  }

  d.face_valid = face_valid;
  d.face_axis = face_axis;
  d.face_sign = face_sign;
  d.face_cos = face_cos;

  // Accumulate face means (axis component only): +/- g along one axis.
  if (face_valid)
  {
    const double v = m_accel_lp[face_axis];
    if (face_sign > 0)
    {
      m_pos_face_mean[face_axis].Add(v);
      m_pos_seen[face_axis] = true;
    }
    else
    {
      m_neg_face_mean[face_axis].Add(v);
      m_neg_seen[face_axis] = true;
    }
  }

  // Solve bias+scale per-axis when both sides are available with enough samples.
  for (std::size_t i = 0; i < 3; ++i)
  {
    if (!m_pos_face_mean[i].Ready(kMinFaceSamplesPerSide) ||
        !m_neg_face_mean[i].Ready(kMinFaceSamplesPerSide))
    {
      continue;
    }

    // mu_pos/neg are mean axis measurements with gravity aligned to +/- axis.
    const double mu_pos = m_pos_face_mean[i].mean; // ~ b_i + s_i * (+g)
    const double mu_neg = m_neg_face_mean[i].mean; // ~ b_i + s_i * (-g)

    const double b_new = 0.5 * (mu_pos + mu_neg);
    const double s_new = (mu_pos - mu_neg) / (2.0 * kG);

    // Smooth + clamp
    const double b_smooth = (1.0 - kSolveAlpha) * m_bias[i] + kSolveAlpha * b_new;
    const double s_smooth = (1.0 - kSolveAlpha) * m_scale[i] + kSolveAlpha * s_new;

    m_bias[i] = std::clamp(b_smooth, -kBiasClampMps2, kBiasClampMps2);
    m_scale[i] = std::clamp(s_smooth, kScaleMin, kScaleMax);
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
