/*
 *  Copyright (C) 2025-2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace OASIS::IMU
{
/*!
 * \brief Converts raw MPU6050 temperature readings and estimates noise variance.
 *
 * Converts TEMP_OUT (raw counts) to degrees Celsius using the datasheet
 * formula, and produces an online estimate of temperature measurement-noise
 * variance for downstream filters.
 *
 * Variance is estimated from a time-aware 2nd-difference residual in raw counts
 * so small sampling-interval jitter does not bias the result:
 *
 *   d2 = c0*r[n-2] + c1*r[n-1] + c2*r[n]
 *
 * Coefficients are derived from the nonuniform 3-point second-derivative
 * stencil, and the variance estimate uses the normalization:
 *
 *   var_counts2 = (d2*d2) / (c0^2 + c1^2 + c2^2)
 *
 * A short history of var_counts2 is averaged to reduce quantization-induced
 * zeros and spikes, then converted to (deg C)^2 using (1/340)^2. A minimum
 * variance floor is applied so covariance is never zero.
 *
 * Note: during warmup/cooldown, nonlinearity can increase the estimate.
 */
class ImuTemperature
{
public:
  struct Sample
  {
    // Temperature in degrees Celsius, computed from the raw sensor reading
    double temperatureC{0.0};

    // Temperature noise variance in (degrees Celsius)^2 from the
    // second-difference residual with sampling-interval jitter compensation.
    double varianceC2{0.0};
  };

  ImuTemperature() = default;

  // Processes a raw temperature sample and returns the converted temperature
  // with variance.
  Sample ProcessRaw(int16_t tempRaw, double dt_s);

  // Sets the minimum temperature noise standard deviation in degrees Celsius.
  void SetMinStdDev(double min_stddev_c);

  // Clears raw-sample history without changing configuration.
  void Reset();

private:
  static constexpr std::size_t kRawHistory = 16;
  static constexpr std::size_t kVarHistory = 16;

  // Raw temperature samples in MPU6050 counts (ring buffer).
  std::array<int16_t, kRawHistory> m_raw{};

  // Second-difference variance estimates in raw counts^2 (ring buffer).
  std::array<double, kVarHistory> m_varCounts2Instant{};

  // Next index to write in the raw sample ring buffer.
  std::size_t m_rawIndex{0};

  // Next index to write in the variance ring buffer.
  std::size_t m_varIndex{0};

  // Number of raw samples captured, capped at kRawHistory.
  std::size_t m_rawCount{0};

  // Number of variance samples captured, capped at kVarHistory.
  std::size_t m_varCount{0};

  // Running sum of variance samples in counts^2 for fast averaging.
  double m_varSumCounts2Instant{0.0};

  // Tracks whether a previous dt sample is available for the 3-sample stencil.
  bool m_hasPrevDt{false};

  // Previous sampling interval in seconds for the 3-sample stencil.
  double m_prevDtS{0.0};

  // Minimum temperature variance in (degrees Celsius)^2.
  double m_minVarianceC2{(0.02 * 0.02)};
};
} // namespace OASIS::IMU
