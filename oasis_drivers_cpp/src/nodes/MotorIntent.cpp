/*
 *  Copyright (C) 2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "MotorIntent.h"

#include "Mpu6050NodeUtils.h"

#include <cmath>

namespace OASIS
{
namespace ROS
{
namespace
{
// Duty magnitude below which the motor is treated as not actively commanded.
constexpr double DUTY_COMMANDED_DEADBAND = 0.02;
} // namespace

MotorIntent::MotorIntent(double ewma_tau, double stale_seconds)
  : m_ewmaTau(ewma_tau), m_staleSeconds(stale_seconds)
{
}

void MotorIntent::Update(const rclcpp::Time& stamp, double duty_cycle)
{
  if (!m_initialized)
  {
    m_dutyRaw = duty_cycle;
    m_dutyFilt = duty_cycle;
    m_dutyFiltDvdt = 0.0;
    m_initialized = true;
    m_lastStamp = stamp;
    return;
  }

  const double dt = (stamp - m_lastStamp).seconds();
  if (dt <= 0.0 || dt > 1.0)
  {
    m_lastStamp = stamp;
    m_dutyRaw = duty_cycle;
    return;
  }

  const double alpha = Mpu6050NodeUtils::EwmaAlpha(dt, m_ewmaTau);
  const double prevDutyFilt = m_dutyFilt;
  m_dutyFilt = prevDutyFilt + alpha * (duty_cycle - prevDutyFilt);
  const double dvdtRaw = (m_dutyFilt - prevDutyFilt) / dt;
  m_dutyFiltDvdt = m_dutyFiltDvdt + alpha * (dvdtRaw - m_dutyFiltDvdt);

  m_lastStamp = stamp;
  m_dutyRaw = duty_cycle;
}

void MotorIntent::SetEwmaTau(double ewma_tau)
{
  m_ewmaTau = ewma_tau;
}

void MotorIntent::SetStaleSeconds(double stale_seconds)
{
  m_staleSeconds = stale_seconds;
}

MotorIntentOutput MotorIntent::Get(const rclcpp::Time& now) const
{
  MotorIntentOutput output;
  output.duty_raw = m_dutyRaw;
  output.duty_filt = m_dutyFilt;
  output.fresh = m_initialized && (now - m_lastStamp).seconds() <= m_staleSeconds;
  output.commanded =
      output.fresh && std::abs(m_dutyFilt) > DUTY_COMMANDED_DEADBAND;
  output.duty_dvdt = output.fresh ? m_dutyFiltDvdt : 0.0;
  output.sign = (m_dutyFilt >= 0.0) ? 1.0 : -1.0;
  return output;
}

bool MotorIntent::IsInitialized() const
{
  return m_initialized;
}

bool MotorIntentEvent(const MotorIntentOutput& motor,
                      double accelLinXY,
                      double dvdt_thresh,
                      double alin_thresh)
{
  return motor.commanded && (std::abs(motor.duty_dvdt) > dvdt_thresh || accelLinXY > alin_thresh);
}

} // namespace ROS
} // namespace OASIS
