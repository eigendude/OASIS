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
class ImuTemperature
{
public:
  struct Sample
  {
    // Temperature in degrees Celsius, computed from the raw sensor reading
    double temperature_c{0.0};

    // Temperature noise variance in (degrees Celsius)^2, computed from a 2nd-difference residual
    double variance_c2{0.0};
  };

  ImuTemperature() = default;

  Sample ProcessRaw(int16_t tempRaw, double dt_s);
  void Reset();

private:
  bool m_hasX1{false};
  bool m_hasX2{false};
  double m_x1{0.0};
  double m_x2{0.0};
};
} // namespace OASIS::IMU
