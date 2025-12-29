/*
 *  Copyright (C) 2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "imu/AccelBiasEstimator.h"

#include <algorithm>
#include <cmath>

namespace OASIS::IMU
{
namespace
{
constexpr double GRAVITY = 9.80665;
constexpr double kEps = 1e-6;
constexpr double kAlpha = 0.006;
constexpr double kAccelLpCutoff = 2.5;
constexpr double kTwoPi = 6.283185307179586;
constexpr double kWindowSeconds = 0.4;
constexpr double kStrongAxisLeak = 0.0006;
constexpr double kWeakAxisLeak = 0.003;
constexpr double kUnknownAxisLeak = 0.008;
constexpr double kWeakAxisAlphaScale = 0.2;
constexpr double kUnknownAxisAlphaScale = 0.05;
constexpr double kBiasClamp = 3.0;
constexpr double kWeakAxisClamp = 2.0;
constexpr double kUnknownAxisClamp = 1.2;
constexpr double kCoverageThreshold = 0.35;
constexpr double kVarScale = 0.25;
constexpr double kJerkScale = 1.2;
constexpr double kGyroScale = 0.6;
constexpr double kGateGErrScale = 0.5;
constexpr double kGateGyroThreshold = 2.0;
constexpr double kGateGyroScale = 0.1;
constexpr double kMinConf = 0.001;
constexpr double kMaxConf = 1.0;
constexpr double kStationaryThreshold = 0.5;
constexpr double kGainAlpha = 0.004;
constexpr double kGainMin = 0.7;
constexpr double kGainMax = 1.3;
constexpr double kGainGateMin = 0.2;
constexpr double kGainConfMin = 0.7;

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
      return {kAlpha, kStrongAxisLeak, kBiasClamp};
    case AxisStrength::Weak:
      return {kAlpha * kWeakAxisAlphaScale, kWeakAxisLeak, kWeakAxisClamp};
    case AxisStrength::Unknown:
    default:
      return {kAlpha * kUnknownAxisAlphaScale, kUnknownAxisLeak, kUnknownAxisClamp};
  }
}
} // namespace

AccelBiasEstimator::AccelBiasEstimator() = default;

const std::array<double, 3>& AccelBiasEstimator::GetBias() const
{
  return m_bias_mps2;
}

void AccelBiasEstimator::WindowStats::AddSample(double value, std::size_t target_size)
{
  samples.push_back(value);
  while (samples.size() > target_size)
  {
    samples.pop_front();
  }
}

double AccelBiasEstimator::WindowStats::Mean() const
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

double AccelBiasEstimator::WindowStats::Variance() const
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

bool AccelBiasEstimator::WindowStats::Empty() const
{
  return samples.empty();
}

std::size_t AccelBiasEstimator::WindowStats::Size() const
{
  return samples.size();
}

void AccelBiasEstimator::UpdateCoverage(const std::array<double, 3>& direction)
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

AccelBiasEstimator::Diagnostics AccelBiasEstimator::Update(const std::array<double, 3>& accel_mps2,
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

  std::array<double, 3> accel_lp_scaled{};
  for (std::size_t i = 0; i < 3; ++i)
  {
    accel_lp_scaled[i] = m_accel_gain * m_accel_lp_mps2[i];
  }

  std::array<double, 3> residual{};
  for (std::size_t i = 0; i < 3; ++i)
  {
    residual[i] = accel_lp_scaled[i] - m_bias_mps2[i];
  }
  double residual_norm = 0.0;
  for (double value : residual)
  {
    residual_norm += value * value;
  }
  residual_norm = std::sqrt(residual_norm);

  const double g_err = residual_norm - GRAVITY;
  double w_gate = w_conf;
  w_gate *= std::exp(-std::abs(g_err) / kGateGErrScale);
  if (gyro_avg > kGateGyroThreshold)
  {
    w_gate *= kGateGyroScale;
  }
  w_gate = std::clamp(w_gate, 0.0, 1.0);
  if (residual_norm > kEps)
  {
    for (std::size_t i = 0; i < 3; ++i)
    {
      const AxisStrength strength = GetAxisStrength(m_axis_pos_seen[i], m_axis_neg_seen[i]);
      const AxisTuning tuning = GetAxisTuning(strength);
      const double grad = (g_err / residual_norm) * residual[i];
      m_bias_mps2[i] += tuning.alpha * w_gate * grad;
    }
  }

  for (std::size_t i = 0; i < 3; ++i)
  {
    const AxisStrength strength = GetAxisStrength(m_axis_pos_seen[i], m_axis_neg_seen[i]);
    const AxisTuning tuning = GetAxisTuning(strength);
    m_bias_mps2[i] *= (1.0 - tuning.leak);
  }

  for (std::size_t i = 0; i < 3; ++i)
  {
    const AxisStrength strength = GetAxisStrength(m_axis_pos_seen[i], m_axis_neg_seen[i]);
    const AxisTuning tuning = GetAxisTuning(strength);
    m_bias_mps2[i] = std::clamp(m_bias_mps2[i], -tuning.clamp, tuning.clamp);
  }

  if (residual_norm > kEps && w_gate > kGainGateMin && w_conf > kGainConfMin)
  {
    const double ratio = GRAVITY / std::max(residual_norm, kEps);
    const double gain_step = std::pow(ratio, kGainAlpha * w_gate);
    m_accel_gain = std::clamp(m_accel_gain * gain_step, kGainMin, kGainMax);
  }

  std::array<double, 3> direction{};
  if (residual_norm > kEps)
  {
    for (std::size_t i = 0; i < 3; ++i)
    {
      direction[i] = residual[i] / residual_norm;
    }
    UpdateCoverage(direction);
  }

  diag.bias_mps2 = m_bias_mps2;
  diag.accel_lp_mps2 = m_accel_lp_mps2;
  diag.accel_gain = m_accel_gain;
  diag.accel_norm_mps2 = accel_norm;
  diag.accel_g = accel_norm / GRAVITY;
  diag.g_err_mps2 = g_err;
  diag.w_conf = w_conf;
  diag.w_gate = w_gate;
  diag.var_mag_mps4 = var_mag;
  diag.jerk_avg_mps3 = jerk_avg;
  diag.gyro_mag_avg_rads = gyro_avg;
  diag.axis_pos_seen = m_axis_pos_seen;
  diag.axis_neg_seen = m_axis_neg_seen;
  diag.stationary = (w_conf > kStationaryThreshold);

  m_prev_accel_lp_mps2 = m_accel_lp_mps2;

  return diag;
}
} // namespace OASIS::IMU
