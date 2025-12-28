/*
 *  Copyright (C) 2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#pragma once

#include <rclcpp/time.hpp>

namespace OASIS
{
namespace ROS
{

struct MotorIntentOutput
{
  double duty_raw = 0.0;
  double duty_filt = 0.0;
  double duty_dvdt = 0.0;
  bool fresh = false;
  bool commanded = false;
  double sign = 1.0;
};

class MotorIntent
{
public:
  MotorIntent(double ewma_tau, double stale_seconds);

  void Update(const rclcpp::Time& stamp, double duty_cycle);
  void SetEwmaTau(double ewma_tau);
  void SetStaleSeconds(double stale_seconds);
  MotorIntentOutput Get(const rclcpp::Time& now) const;
  bool IsInitialized() const;

private:
  double m_ewmaTau = 0.0;
  double m_staleSeconds = 0.0;
  double m_dutyRaw = 0.0;
  double m_dutyFilt = 0.0;
  double m_dutyFiltDvdt = 0.0;
  rclcpp::Time m_lastStamp;
  bool m_initialized = false;
};

bool MotorIntentEvent(const MotorIntentOutput& motor,
                      double accelLinXY,
                      double dvdt_thresh,
                      double alin_thresh);

} // namespace ROS
} // namespace OASIS
