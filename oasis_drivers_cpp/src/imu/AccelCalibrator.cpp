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
constexpr double GRAVITY = 9.80665;
constexpr double kEps = 1e-6;
constexpr double kAlphaBias = 0.006;
constexpr double kAlphaScale = 0.0003;
constexpr double kAccelLpCutoff = 2.5;
constexpr double kTwoPi = 6.283185307179586;
constexpr double kWindowSeconds = 0.4;
constexpr double kStrongAxisLeak = 0.0006;
constexpr double kWeakAxisLeak = 0.003;
constexpr double kUnknownAxisLeak = 0.008;
constexpr double kWeakAxisAlphaScale = 0.3;
constexpr double kUnknownAxisAlphaScale = 0.1;
constexpr double kBiasClamp = 3.0;
constexpr double kWeakAxisClamp = 1.5;
constexpr double kUnknownAxisClamp = 1.0;
constexpr double kCoverageThreshold = 0.35;
constexpr double kVarScale = 0.25;
constexpr double kJerkScale = 1.2;
constexpr double kGyroScale = 0.6;
constexpr double kMinConf = 0.001;
constexpr double kMaxConf = 1.0;
constexpr double kStationaryThreshold = 0.5;
constexpr double kLearnConfThreshold = 0.8;
constexpr double kBiasSanityScale = 0.3;
constexpr double kScaleLeakStrong = 0.0004;
constexpr double kScaleLeakWeak = 0.002;
constexpr double kScaleLeakUnknown = 0.004;
constexpr double kScaleClampStrongMin = 0.7;
constexpr double kScaleClampStrongMax = 1.3;
// Allow weaker/unknown axes to represent large initial scale errors (e.g. ~1.25g).
constexpr double kScaleClampWeakMin = 0.6;
constexpr double kScaleClampWeakMax = 1.5;
constexpr double kScaleClampUnknownMin = 0.6;
constexpr double kScaleClampUnknownMax = 1.5;
constexpr double kGSanity = 1.5;

enum class AxisStrength
{
  Strong,
  Weak,
  Unknown
};

struct AxisTuning
{
  double alpha{0.0};
  double leak{0.0};
  double clamp{0.0};
};

struct ScaleTuning
{
  double leak{0.0};
  double clamp_min{1.0};
  double clamp_max{1.0};
};

AxisStrength GetAxisStrength(bool pos_seen, bool neg_seen)
{
  if (pos_seen && neg_seen)
  {
    return AxisStrength::Strong;
  }
  if (pos_seen || neg_seen)
  {
    return AxisStrength::Weak;
  }
  return AxisStrength::Unknown;
}

AxisTuning GetAxisTuning(AxisStrength strength)
{
  switch (strength)
  {
    case AxisStrength::Strong:
      return {kAlphaBias, kStrongAxisLeak, kBiasClamp};
    case AxisStrength::Weak:
      return {kAlphaBias * kWeakAxisAlphaScale, kWeakAxisLeak, kWeakAxisClamp};
    case AxisStrength::Unknown:
    default:
      return {kAlphaBias * kUnknownAxisAlphaScale, kUnknownAxisLeak, kUnknownAxisClamp};
  }
}

ScaleTuning GetScaleTuning(AxisStrength strength)
{
  switch (strength)
  {
    case AxisStrength::Strong:
      return {kScaleLeakStrong, kScaleClampStrongMin, kScaleClampStrongMax};
    case AxisStrength::Weak:
      return {kScaleLeakWeak, kScaleClampWeakMin, kScaleClampWeakMax};
    case AxisStrength::Unknown:
    default:
      return {kScaleLeakUnknown, kScaleClampUnknownMin, kScaleClampUnknownMax};
  }
}
} // namespace

AccelCalibrator::AccelCalibrator() = default;

const std::array<double, 3>& AccelCalibrator::GetBias() const
{
  return m_bias_mps2;
}

const std::array<double, 3>& AccelCalibrator::GetScale() const
{
  return m_scale;
}

void AccelCalibrator::WindowStats::AddSample(double value, std::size_t target_size)
{
  samples.push_back(value);
  while (samples.size() > target_size)
  {
    samples.pop_front();
  }
}

double AccelCalibrator::WindowStats::Mean() const
{
  if (samples.empty())
  {
    return 0.0;
  }
  double sum = 0.0;
  for (double value : samples)
  {
    sum += value;
  }
  return sum / static_cast<double>(samples.size());
}

double AccelCalibrator::WindowStats::Variance() const
{
  if (samples.size() < 2)
  {
    return 0.0;
  }
  const double mean = Mean();
  double accum = 0.0;
  for (double value : samples)
  {
    const double diff = value - mean;
    accum += diff * diff;
  }
  return accum / static_cast<double>(samples.size());
}

bool AccelCalibrator::WindowStats::Empty() const
{
  return samples.empty();
}

std::size_t AccelCalibrator::WindowStats::Size() const
{
  return samples.size();
}

void AccelCalibrator::UpdateCoverage(const std::array<double, 3>& direction)
{
  for (std::size_t axis = 0; axis < 3; ++axis)
  {
    if (direction[axis] > kCoverageThreshold)
    {
      m_axis_pos_seen[axis] = true;
    }
    else if (direction[axis] < -kCoverageThreshold)
    {
      m_axis_neg_seen[axis] = true;
    }
  }
}

AccelCalibrator::Diagnostics AccelCalibrator::Update(const std::array<double, 3>& accel_mps2,
                                                     const std::array<double, 3>& gyro_rads,
                                                     double dt_seconds)
{
  Diagnostics diag;
  if (dt_seconds <= 0.0)
  {
    dt_seconds = 0.02;
  }

  const double lp_alpha = 1.0 - std::exp(-dt_seconds * kAccelLpCutoff * kTwoPi);

  if (!m_initialized)
  {
    m_accel_lp_mps2 = accel_mps2;
    m_prev_accel_lp_mps2 = accel_mps2;
    m_initialized = true;
  }

  for (std::size_t i = 0; i < 3; ++i)
  {
    m_accel_lp_mps2[i] = m_accel_lp_mps2[i] + lp_alpha * (accel_mps2[i] - m_accel_lp_mps2[i]);
  }

  double accel_norm = 0.0;
  for (double value : m_accel_lp_mps2)
  {
    accel_norm += value * value;
  }
  accel_norm = std::sqrt(accel_norm);

  std::array<double, 3> jerk{};
  for (std::size_t i = 0; i < 3; ++i)
  {
    jerk[i] = (m_accel_lp_mps2[i] - m_prev_accel_lp_mps2[i]) / dt_seconds;
  }
  double jerk_mag = 0.0;
  for (double value : jerk)
  {
    jerk_mag += value * value;
  }
  jerk_mag = std::sqrt(jerk_mag);

  double gyro_mag = 0.0;
  for (double value : gyro_rads)
  {
    gyro_mag += value * value;
  }
  gyro_mag = std::sqrt(gyro_mag);

  const std::size_t window_size =
      std::max<std::size_t>(1, static_cast<std::size_t>(std::round(kWindowSeconds / dt_seconds)));
  m_accel_mag_window.AddSample(accel_norm, window_size);
  m_jerk_window.AddSample(jerk_mag, window_size);
  m_gyro_mag_window.AddSample(gyro_mag, window_size);

  const double var_mag = m_accel_mag_window.Variance();
  const double jerk_avg = m_jerk_window.Mean();
  const double gyro_avg = m_gyro_mag_window.Mean();

  const double score = (var_mag / kVarScale) + (jerk_avg / kJerkScale) + (gyro_avg / kGyroScale);
  const double w_conf = std::clamp(std::exp(-score), kMinConf, kMaxConf);
  const bool learn_enabled = (w_conf >= kLearnConfThreshold);

  std::array<double, 3> residual{};
  for (std::size_t i = 0; i < 3; ++i)
  {
    residual[i] = m_accel_lp_mps2[i] - m_bias_mps2[i];
  }

  if (!m_scale_initialized && learn_enabled && m_accel_mag_window.Size() >= window_size)
  {
    double residual_norm_raw = 0.0;
    for (double value : residual)
    {
      residual_norm_raw += value * value;
    }
    residual_norm_raw = std::sqrt(residual_norm_raw);
    if (residual_norm_raw > kEps)
    {
      const double init_scale = residual_norm_raw / GRAVITY;
      for (std::size_t i = 0; i < 3; ++i)
      {
        m_scale[i] = std::clamp(m_scale[i] * init_scale, 0.5, 2.0);
      }
      m_scale_initialized = true;
    }
  }

  std::array<double, 3> accel_cal_lp{};
  for (std::size_t i = 0; i < 3; ++i)
  {
    const double scale = std::max(std::abs(m_scale[i]), kEps);
    accel_cal_lp[i] = residual[i] / scale;
  }

  double accel_cal_norm = 0.0;
  for (double value : accel_cal_lp)
  {
    accel_cal_norm += value * value;
  }
  accel_cal_norm = std::sqrt(accel_cal_norm);

  const double g_err = accel_cal_norm - GRAVITY;
  const bool g_err_sane = (std::abs(g_err) <= kGSanity);
  const bool learn_bias = (learn_enabled && accel_cal_norm > kEps);
  const bool learn_scale = (learn_enabled && accel_cal_norm > kEps);

  std::array<double, 3> direction{};
  if (accel_cal_norm > kEps)
  {
    for (std::size_t i = 0; i < 3; ++i)
    {
      direction[i] = accel_cal_lp[i] / accel_cal_norm;
    }
    UpdateCoverage(direction);
  }

  if (learn_bias)
  {
    const double bias_scale = g_err_sane ? 1.0 : kBiasSanityScale;
    for (std::size_t i = 0; i < 3; ++i)
    {
      const AxisStrength strength = GetAxisStrength(m_axis_pos_seen[i], m_axis_neg_seen[i]);
      const AxisTuning tuning = GetAxisTuning(strength);
      const double denom = std::max(std::abs(m_scale[i]), kEps);
      const double unit = accel_cal_lp[i] / accel_cal_norm;
      const double grad = (unit / denom);
      m_bias_mps2[i] += tuning.alpha * bias_scale * w_conf * g_err * grad;
    }
  }

  if (learn_scale)
  {
    for (std::size_t i = 0; i < 3; ++i)
    {
      const bool coverage_complete = m_axis_pos_seen[i] && m_axis_neg_seen[i];
      if (!coverage_complete)
      {
        continue;
      }
      const double denom = std::max(std::abs(m_scale[i]), kEps);
      const double unit = accel_cal_lp[i] / accel_cal_norm;
      const double grad = (unit * residual[i]) / (denom * denom);
      m_scale[i] += kAlphaScale * w_conf * g_err * grad;
    }
  }

  for (std::size_t i = 0; i < 3; ++i)
  {
    const AxisStrength strength = GetAxisStrength(m_axis_pos_seen[i], m_axis_neg_seen[i]);
    const AxisTuning tuning = GetAxisTuning(strength);
    m_bias_mps2[i] *= (1.0 - tuning.leak);
    m_bias_mps2[i] = std::clamp(m_bias_mps2[i], -tuning.clamp, tuning.clamp);
  }

  for (std::size_t i = 0; i < 3; ++i)
  {
    const AxisStrength strength = GetAxisStrength(m_axis_pos_seen[i], m_axis_neg_seen[i]);
    const ScaleTuning tuning = GetScaleTuning(strength);
    m_scale[i] += tuning.leak * (1.0 - m_scale[i]);
    m_scale[i] = std::clamp(m_scale[i], tuning.clamp_min, tuning.clamp_max);
  }

  diag.bias_mps2 = m_bias_mps2;
  diag.scale = m_scale;
  diag.accel_lp_mps2 = m_accel_lp_mps2;
  diag.accel_cal_lp_mps2 = accel_cal_lp;
  diag.accel_norm_mps2 = accel_cal_norm;
  diag.accel_g = accel_cal_norm / GRAVITY;
  diag.g_err_mps2 = g_err;
  diag.w_conf = w_conf;
  diag.var_mag_mps4 = var_mag;
  diag.jerk_avg_mps3 = jerk_avg;
  diag.gyro_mag_avg_rads = gyro_avg;
  diag.axis_pos_seen = m_axis_pos_seen;
  diag.axis_neg_seen = m_axis_neg_seen;
  diag.stationary = (w_conf > kStationaryThreshold);
  diag.learn_bias = learn_bias;
  diag.learn_scale = learn_scale;

  m_prev_accel_lp_mps2 = m_accel_lp_mps2;

  return diag;
}
} // namespace OASIS::IMU
