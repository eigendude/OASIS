/*
 *  Copyright (C) 2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#pragma once

#include <cstddef>
#include <cstdint>

namespace OASIS::IMU
{
// Converts raw MPU6050 temperature readings and estimates immediate measurement noise variance
// using a second-difference residual in raw counts (d2 = x[n] - 2*x[n-1] + x[n-2]).
class ImuTemperature
{
public:
  struct Sample
  {
    // Temperature in degrees Celsius, computed from the raw sensor reading
    double temperature_c{0.0};

    // Temperature noise variance in (degrees Celsius)^2 from the second-difference residual
    // normalized to a fixed nominal sampling interval.
    double variance_c2{0.0};
  };

  ImuTemperature() = default;

  // Processes a raw temperature sample and returns the converted temperature with variance.
  Sample ProcessRaw(int16_t tempRaw, double dt_s);

  // Sets the minimum temperature noise standard deviation in degrees Celsius.
  void SetMinStdDev(double min_stddev_c);

  // Clears raw-sample history without changing configuration.
  void Reset();

private:
  // Oldest raw temperature sample x[n-2] in MPU6050 counts.
  int16_t m_r0{0};

  // Middle raw temperature sample x[n-1] in MPU6050 counts.
  int16_t m_r1{0};

  // Newest raw temperature sample x[n] in MPU6050 counts.
  int16_t m_r2{0};

  // Number of raw samples captured, capped at 3.
  std::size_t m_count{0};

  // Minimum temperature variance in (degrees Celsius)^2.
  double m_minVarianceC2{(0.02 * 0.02)};
};
} // namespace OASIS::IMU
