/*
 *  Copyright (C) 2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "imu/GyroBiasCalibrator.h"

#include <algorithm>
#include <cmath>

using namespace OASIS::IMU;

GyroBiasCalibrator::GyroBiasCalibrator(const GyroBiasCalibratorConfig& config) : m_config(config)
{
  StartWarmup();
}

void GyroBiasCalibrator::Reset()
{
  StartWarmup();
}

GyroBiasCalibrator::State GyroBiasCalibrator::GetState() const
{
  return m_state;
}

const GyroBiasEstimate& GyroBiasCalibrator::GetEstimate() const
{
  return m_estimate;
}

void GyroBiasCalibrator::Update(const std::array<double, 3>& accel_mps2,
                                const std::array<double, 3>& gyro_rads,
                                double dt_seconds)
{
  if (dt_seconds <= 0.0)
  {
    return;
  }

  const double accel_mag = std::sqrt(accel_mps2[0] * accel_mps2[0] +
                                     accel_mps2[1] * accel_mps2[1] +
                                     accel_mps2[2] * accel_mps2[2]);
  double accel_jerk = 0.0;
  if (m_hasPrevAccel)
  {
    accel_jerk = std::abs((accel_mag - m_prevAccelMag) / dt_seconds);
  }
  m_prevAccelMag = accel_mag;
  m_hasPrevAccel = true;

  switch (m_state)
  {
    case State::WARMUP:
      m_warmupElapsed += dt_seconds;
      if (m_warmupElapsed >= m_config.warmup_duration_s)
      {
        StartWaitStill();
      }
      break;

    case State::WAIT_STILL:
    {
      const bool accel_mag_ok =
          std::abs(accel_mag - m_config.gravity_mps2) <= m_config.accel_mag_tolerance_mps2;
      const bool accel_jerk_ok = accel_jerk <= m_config.accel_jerk_threshold_mps3;
      if (accel_mag_ok && accel_jerk_ok)
      {
        m_stillElapsed += dt_seconds;
      }
      else
      {
        m_stillElapsed = 0.0;
      }

      if (m_stillElapsed >= m_config.stillness_duration_s)
      {
        StartCalibrate();
      }
      break;
    }

    case State::CALIBRATE:
    {
      ++m_samples;
      for (std::size_t i = 0; i < 3; ++i)
      {
        const double sample = gyro_rads[i];
        const double delta = sample - m_mean[i];
        m_mean[i] += delta / static_cast<double>(m_samples);
        const double delta2 = sample - m_mean[i];
        m_m2[i] += delta * delta2;
      }

      m_calibrateElapsed += dt_seconds;
      if (m_calibrateElapsed >= m_config.calibrate_duration_s)
      {
        std::array<double, 3> stddev{};
        const double denom = std::max<std::size_t>(m_samples, 1);
        for (std::size_t i = 0; i < 3; ++i)
        {
          stddev[i] = std::sqrt(m_m2[i] / static_cast<double>(denom));
        }

        const bool vibration =
            stddev[0] > m_config.gyro_stddev_threshold_rads ||
            stddev[1] > m_config.gyro_stddev_threshold_rads ||
            stddev[2] > m_config.gyro_stddev_threshold_rads;
        if (vibration)
        {
          StartWaitStill();
          break;
        }

        m_estimate.bias_rads = m_mean;
        m_estimate.stddev_rads = stddev;
        StartReady();
      }
      break;
    }

    case State::READY:
      break;
  }
}

void GyroBiasCalibrator::StartWarmup()
{
  m_state = State::WARMUP;
  m_warmupElapsed = 0.0;
  m_stillElapsed = 0.0;
  m_calibrateElapsed = 0.0;
  m_hasPrevAccel = false;
  m_prevAccelMag = 0.0;
  m_mean = {0.0, 0.0, 0.0};
  m_m2 = {0.0, 0.0, 0.0};
  m_samples = 0;
  m_estimate = {};
}

void GyroBiasCalibrator::StartWaitStill()
{
  m_state = State::WAIT_STILL;
  m_stillElapsed = 0.0;
  m_calibrateElapsed = 0.0;
  m_mean = {0.0, 0.0, 0.0};
  m_m2 = {0.0, 0.0, 0.0};
  m_samples = 0;
  m_estimate = {};
}

void GyroBiasCalibrator::StartCalibrate()
{
  m_state = State::CALIBRATE;
  m_calibrateElapsed = 0.0;
  m_mean = {0.0, 0.0, 0.0};
  m_m2 = {0.0, 0.0, 0.0};
  m_samples = 0;
}

void GyroBiasCalibrator::StartReady()
{
  m_state = State::READY;
}
