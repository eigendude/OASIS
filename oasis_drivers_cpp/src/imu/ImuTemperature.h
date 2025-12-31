/*
 *  Copyright (C) 2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#pragma once

#include <cstdint>

namespace OASIS::IMU
{
// Converts raw MPU6050 temperature readings and estimates immediate measurement noise variance
// using a second-difference residual (d2 = x[n] - 2*x[n-1] + x[n-2]).
class ImuTemperature
{
public:
  struct Sample
  {
    // Temperature in degrees Celsius, computed from the raw sensor reading
    double temperature_c{0.0};

    // Temperature noise variance in (degrees Celsius)^2 from the second-difference residual
    double variance_c2{0.0};
  };

  ImuTemperature() = default;

  Sample ProcessRaw(int16_t tempRaw, double dt_s);
  void Reset();

private:
  // True once the most recent sample x[n-1] is available
  bool m_hasX1{false};

  // True once the second most recent sample x[n-2] is available
  bool m_hasX2{false};

  // Last temperature sample x[n-1] in degrees Celsius
  double m_x1{0.0};

  // Second last temperature sample x[n-2] in degrees Celsius
  double m_x2{0.0};
};
} // namespace OASIS::IMU
