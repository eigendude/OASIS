/*
 *  Copyright (C) 2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "imu/ForwardAxisLearner.h"

#include "imu/ImuMath.h"

#include <algorithm>
#include <cmath>

namespace OASIS::IMU
{
namespace
{
constexpr double kG = 9.80665;
constexpr double kYawStraightMax = 0.06;
constexpr double kYawStraightMaxInBurst = 0.06;
constexpr double kABurstStart = 0.6;
constexpr double kABurstEnd = 0.2;
constexpr double kMinBurstSeconds = 0.150;
constexpr double kMaxBurstSeconds = 1.500;
constexpr double kMinImpulse = 0.25;
constexpr double kMaxYawInBurst = 0.10;
constexpr std::size_t kMinBurstsToLock = 2;
constexpr double kConsistencyDotMin = 0.90;
constexpr std::size_t kStartSamples = 4;
constexpr std::size_t kEndSamples = 5;
constexpr std::size_t kYawOverSamplesAbort = 3;
constexpr double kBeta = 0.2;
constexpr double kEps = 1e-9;
} // namespace

ForwardAxisLearner::ForwardAxisLearner() = default;

ForwardAxisLearner::Diagnostics ForwardAxisLearner::Update(const std::array<double, 3>& accel_mps2,
                                                           const std::array<double, 3>& gyro_rads,
                                                           double dt_seconds,
                                                           const std::array<double, 3>& u_hat,
                                                           bool u_hat_valid)
{
  Diagnostics diag;
  diag.state = m_state;
  diag.burst_count = m_burst_count;
  diag.consistency_dot = m_consistency_dot;
  diag.locked = (m_state == State::LOCKED);

  if (dt_seconds <= 0.0)
    dt_seconds = 0.02;

  if (!u_hat_valid)
    return diag;

  const double yaw_rate = Math::Dot(gyro_rads, u_hat);
  const std::array<double, 3> a_lin = Math::Subtract(accel_mps2, Math::Scale(u_hat, kG));
  const std::array<double, 3> a_h =
      Math::Subtract(a_lin, Math::Scale(u_hat, Math::Dot(a_lin, u_hat)));
  const double a_h_mag = Math::Norm(a_h);

  diag.yaw_rate_rads = yaw_rate;
  diag.a_h_mag_mps2 = a_h_mag;

  switch (m_state)
  {
    case State::ARMED:
    {
      if (std::abs(yaw_rate) < kYawStraightMax && a_h_mag > kABurstStart)
        ++m_start_counter;
      else
        m_start_counter = 0;

      if (m_start_counter >= kStartSamples)
      {
        StartBurst();
        m_state = State::IN_BURST;
      }
      break;
    }
    case State::IN_BURST:
    {
      if (std::abs(yaw_rate) > kYawStraightMaxInBurst)
        ++m_yaw_over_counter;
      else
        m_yaw_over_counter = 0;

      if (m_yaw_over_counter >= kYawOverSamplesAbort)
      {
        ResetBurst();
        m_state = State::ARMED;
        break;
      }

      m_sum_vec = Math::Add(m_sum_vec, a_h);
      m_impulse += a_h_mag * dt_seconds;
      m_yaw_accum += std::abs(yaw_rate) * dt_seconds;
      m_t_burst += dt_seconds;

      if (a_h_mag < kABurstEnd)
        ++m_end_counter;
      else
        m_end_counter = 0;

      if (m_end_counter >= kEndSamples || m_t_burst >= kMaxBurstSeconds)
      {
        EvaluateBurst(u_hat);
        ResetBurst();
        m_state = (m_state == State::LOCKED) ? State::LOCKED : State::ARMED;
      }
      break;
    }
    case State::LOCKED:
      break;
  }

  diag.state = m_state;
  diag.burst_count = m_burst_count;
  diag.impulse_mps = m_impulse;
  diag.yaw_accum_rad = m_yaw_accum;
  diag.burst_time_s = m_t_burst;
  diag.consistency_dot = m_consistency_dot;
  diag.locked = (m_state == State::LOCKED);
  return diag;
}

bool ForwardAxisLearner::IsLocked() const
{
  return m_state == State::LOCKED;
}

const std::array<double, 3>& ForwardAxisLearner::GetForwardAxisUnsigned() const
{
  return m_f_hat_unsigned;
}

const std::array<double, 3>& ForwardAxisLearner::GetForwardAxisOrtho() const
{
  return m_f_hat;
}

ForwardAxisLearner::State ForwardAxisLearner::GetState() const
{
  return m_state;
}

void ForwardAxisLearner::StartBurst()
{
  m_sum_vec = {0.0, 0.0, 0.0};
  m_impulse = 0.0;
  m_yaw_accum = 0.0;
  m_t_burst = 0.0;
  m_end_counter = 0;
  m_yaw_over_counter = 0;
}

void ForwardAxisLearner::ResetBurst()
{
  m_start_counter = 0;
  m_end_counter = 0;
  m_yaw_over_counter = 0;
  m_sum_vec = {0.0, 0.0, 0.0};
  m_impulse = 0.0;
  m_yaw_accum = 0.0;
  m_t_burst = 0.0;
}

void ForwardAxisLearner::EvaluateBurst(const std::array<double, 3>& u_hat)
{
  if (m_t_burst < kMinBurstSeconds || m_t_burst > kMaxBurstSeconds)
    return;

  if (m_impulse < kMinImpulse)
    return;

  if (m_yaw_accum > kMaxYawInBurst)
    return;

  const double sum_norm = Math::Norm(m_sum_vec);
  if (sum_norm <= kEps)
    return;

  std::array<double, 3> candidate = Math::Normalize(m_sum_vec);
  m_last_candidate = candidate;

  if (m_burst_count == 0)
  {
    m_f_mean = candidate;
  }
  else
  {
    if (Math::Dot(candidate, m_f_mean) < 0.0)
      candidate = Math::Scale(candidate, -1.0);

    m_f_mean = Math::Normalize(
        Math::Add(Math::Scale(m_f_mean, 1.0 - kBeta), Math::Scale(candidate, kBeta)));
  }

  ++m_burst_count;
  m_consistency_dot = Math::Dot(candidate, m_f_mean);

  if (m_burst_count >= kMinBurstsToLock && m_consistency_dot >= kConsistencyDotMin)
  {
    const std::array<double, 3> r_hat = Math::Normalize(Math::Cross(u_hat, m_f_mean));
    const double r_norm = Math::Norm(r_hat);
    if (r_norm <= kEps)
      return;

    m_f_hat_unsigned = m_f_mean;
    m_f_hat = Math::Normalize(Math::Cross(r_hat, u_hat));
    m_state = State::LOCKED;
  }
}
} // namespace OASIS::IMU
