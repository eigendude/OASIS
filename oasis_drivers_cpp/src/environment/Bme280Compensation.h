/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#pragma once

#include <array>
#include <cstdint>

namespace OASIS::Environment
{
struct Bme280Calibration
{
  //! Unsigned temperature calibration coefficient from NVM
  std::uint16_t dig_t1{0};
  //! Signed temperature calibration coefficients from NVM
  std::int16_t dig_t2{0};
  std::int16_t dig_t3{0};
  //! Unsigned pressure calibration coefficient from NVM
  std::uint16_t dig_p1{0};
  //! Signed pressure calibration coefficients from NVM
  std::int16_t dig_p2{0};
  std::int16_t dig_p3{0};
  std::int16_t dig_p4{0};
  std::int16_t dig_p5{0};
  std::int16_t dig_p6{0};
  std::int16_t dig_p7{0};
  std::int16_t dig_p8{0};
  std::int16_t dig_p9{0};
  //! Unsigned humidity calibration coefficients from NVM
  std::uint8_t dig_h1{0};
  std::uint16_t dig_h3{0};
  //! Signed humidity calibration coefficients from NVM
  std::int16_t dig_h2{0};
  std::int16_t dig_h4{0};
  std::int16_t dig_h5{0};
  std::int8_t dig_h6{0};
};

struct Bme280RawSample
{
  //! Uncompensated 20-bit pressure ADC code
  std::uint32_t pressure{0};
  //! Uncompensated 20-bit temperature ADC code
  std::uint32_t temperature{0};
  //! Uncompensated 16-bit relative-humidity ADC code
  std::uint16_t humidity{0};
};

struct Bme280Sample
{
  //! Compensated ambient temperature in degrees Celsius
  double temperature_c{0.0};
  //! Compensated ambient pressure in pascals
  double pressure_pa{0.0};
  //! Compensated relative humidity in percent, normally in [0, 100]
  double relative_humidity_percent{0.0};
};

class Bme280Compensation
{
public:
  static Bme280Calibration DecodeCalibration(const std::array<std::uint8_t, 26>& primary,
                                             const std::array<std::uint8_t, 7>& humidity);
  static Bme280RawSample DecodeRawSample(const std::array<std::uint8_t, 8>& data);
  static Bme280Sample Compensate(const Bme280Calibration& calibration, const Bme280RawSample& raw);
};
} // namespace OASIS::Environment
