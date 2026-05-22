/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "localization/ZuptDetector.hpp"

#include <cmath>

using OASIS::Localization::Vector3;
using OASIS::Localization::ZuptDecision;
using OASIS::Localization::ZuptDetector;
using OASIS::Localization::ZuptDetectorConfig;

namespace
{
bool Vector3IsFinite(const Vector3& values)
{
  return std::isfinite(values[0]) && std::isfinite(values[1]) && std::isfinite(values[2]);
}

double Norm3(const Vector3& values)
{
  return std::sqrt(values[0] * values[0] + values[1] * values[1] + values[2] * values[2]);
}

double ElapsedSec(std::optional<double> timestamp_sec, std::optional<double> candidate_start_sec)
{
  if (!timestamp_sec.has_value() || !candidate_start_sec.has_value())
  {
    return 0.0;
  }

  if (*timestamp_sec < *candidate_start_sec)
  {
    return 0.0;
  }

  return *timestamp_sec - *candidate_start_sec;
}

bool DurationReached(double elapsed_sec, double threshold_sec)
{
  if (elapsed_sec >= threshold_sec)
  {
    return true;
  }

  return std::abs(elapsed_sec - threshold_sec) <= 1.0e-12;
}
} // namespace

ZuptDetector::ZuptDetector(ZuptDetectorConfig config) : m_config(config)
{
  m_state.current_linear_zupt_variance_mps2 = config.moving_linear_variance_mps2;
  m_state.current_angular_zupt_variance_rads2 = config.moving_angular_variance_rads2;
}

std::optional<ZuptDecision> ZuptDetector::Update(double timestamp_sec,
                                                 const Vector3& angular_velocity_rads,
                                                 const Vector3& linear_accel_mps2)
{
  const std::optional<double> previous_timestamp_sec = m_state.last_timestamp_sec;
  if (!std::isfinite(timestamp_sec))
  {
    m_state.last_reason = "invalid_timestamp";
    return std::nullopt;
  }

  if (previous_timestamp_sec.has_value() && timestamp_sec < *previous_timestamp_sec)
  {
    m_state.enter_candidate_start_sec.reset();
    m_state.exit_candidate_start_sec.reset();
    m_state.last_reason = "non_monotonic_timestamp";
    return std::nullopt;
  }

  m_state.last_timestamp_sec = timestamp_sec;
  if (!Vector3IsFinite(angular_velocity_rads) || !Vector3IsFinite(linear_accel_mps2))
  {
    m_state.last_reason = "invalid_imu_sample";
    return BuildDecision(m_state.last_reason);
  }

  const double gyro_norm_rads = Norm3(angular_velocity_rads);
  const double accel_norm_mps2 = Norm3(linear_accel_mps2);
  m_state.last_gyro_norm_rads = gyro_norm_rads;
  m_state.last_accel_norm_mps2 = accel_norm_mps2;

  const bool enter_candidate = gyro_norm_rads <= m_config.gyro_enter_threshold_rads &&
                               accel_norm_mps2 <= m_config.accel_enter_threshold_mps2;

  // Exit uses raw norm threshold breaches with a very short dwell so motion
  // onset clears stationary quickly
  const bool exit_candidate = gyro_norm_rads >= m_config.gyro_exit_threshold_rads ||
                              accel_norm_mps2 >= m_config.accel_exit_threshold_mps2;

  if (m_state.stationary)
  {
    UpdateExitCandidate(timestamp_sec, previous_timestamp_sec, exit_candidate);
  }
  else
  {
    UpdateEnterCandidate(timestamp_sec, enter_candidate);
  }

  m_state.current_linear_zupt_variance_mps2 =
      m_state.stationary ? StationaryLinearVarianceMps2() : m_config.moving_linear_variance_mps2;
  m_state.current_angular_zupt_variance_rads2 = m_state.stationary
                                                    ? StationaryAngularVarianceRads2()
                                                    : m_config.moving_angular_variance_rads2;

  return BuildDecision(m_state.last_reason);
}

void ZuptDetector::UpdateEnterCandidate(double timestamp_sec, bool enter_candidate)
{
  if (!enter_candidate)
  {
    m_state.enter_candidate_start_sec.reset();
    m_state.last_reason = "moving";
    return;
  }

  if (!m_state.enter_candidate_start_sec.has_value())
  {
    m_state.enter_candidate_start_sec = timestamp_sec;
    m_state.last_reason = "enter_candidate_started";
    return;
  }

  const double enter_dwell_sec = timestamp_sec - *m_state.enter_candidate_start_sec;
  if (DurationReached(enter_dwell_sec, m_config.min_stationary_sec))
  {
    m_state.stationary = true;
    m_state.enter_candidate_start_sec.reset();
    m_state.exit_candidate_start_sec.reset();
    m_state.last_reason = "stationary_asserted";
    return;
  }

  m_state.last_reason = "enter_candidate_pending";
}

void ZuptDetector::UpdateExitCandidate(double timestamp_sec,
                                       std::optional<double> previous_timestamp_sec,
                                       bool exit_candidate)
{
  if (!exit_candidate)
  {
    m_state.exit_candidate_start_sec.reset();
    m_state.last_reason = "stationary_held";
    return;
  }

  if (!m_state.exit_candidate_start_sec.has_value())
  {
    m_state.exit_candidate_start_sec =
        previous_timestamp_sec.has_value() ? previous_timestamp_sec : timestamp_sec;
  }

  const double exit_dwell_sec = timestamp_sec - *m_state.exit_candidate_start_sec;
  if (DurationReached(exit_dwell_sec, m_config.min_moving_sec))
  {
    m_state.stationary = false;
    m_state.enter_candidate_start_sec.reset();
    m_state.exit_candidate_start_sec.reset();
    m_state.last_reason = "stationary_cleared";
    return;
  }

  if (previous_timestamp_sec == m_state.exit_candidate_start_sec)
  {
    m_state.last_reason = "exit_candidate_started";
    return;
  }

  m_state.last_reason = "exit_candidate_pending";
}

double ZuptDetector::StationaryLinearVarianceMps2() const
{
  const double sigma_mps = m_config.stationary_linear_velocity_sigma_mps;
  return sigma_mps * sigma_mps;
}

double ZuptDetector::StationaryAngularVarianceRads2() const
{
  const double sigma_rads = m_config.stationary_angular_velocity_sigma_rads;
  return sigma_rads * sigma_rads;
}

ZuptDecision ZuptDetector::BuildDecision(const std::string& reason) const
{
  ZuptDecision decision;
  decision.stationary = m_state.stationary;
  decision.gyro_norm_rads = m_state.last_gyro_norm_rads;
  decision.accel_norm_mps2 = m_state.last_accel_norm_mps2;
  decision.linear_zupt_variance_mps2 = m_state.current_linear_zupt_variance_mps2;
  decision.angular_zupt_variance_rads2 = m_state.current_angular_zupt_variance_rads2;
  decision.enter_dwell_sec =
      ElapsedSec(m_state.last_timestamp_sec, m_state.enter_candidate_start_sec);
  decision.exit_dwell_sec =
      ElapsedSec(m_state.last_timestamp_sec, m_state.exit_candidate_start_sec);
  decision.reason = reason;
  return decision;
}
