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
class ImuTemperature
{
public:
  struct Sample
  {
    // Temperature in degrees Celsius, computed from the raw sensor reading
    double temperature_c{0.0};

    // Temperature variance in (degrees Celsius)^2, computed from running samples
    double variance_c2{0.0};
  };

  ImuTemperature() = default;

  Sample ProcessRaw(int16_t tempRaw);
  void Reset();

private:
  std::size_t m_sampleCount{0};
  double m_meanC{0.0};
  double m_m2{0.0};
};
} // namespace OASIS::IMU
