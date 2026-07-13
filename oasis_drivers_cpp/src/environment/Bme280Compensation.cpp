/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "Bme280Compensation.h"

#include <cmath>

namespace OASIS::Environment
{
namespace
{
std::uint16_t ReadU16Le(const std::uint8_t* data)
{
  return static_cast<std::uint16_t>(data[0]) |
         static_cast<std::uint16_t>(static_cast<std::uint16_t>(data[1]) << 8);
}

std::int16_t ReadS16Le(const std::uint8_t* data)
{
  return static_cast<std::int16_t>(ReadU16Le(data));
}

std::int16_t SignExtend12(std::uint16_t value)
{
  if ((value & 0x0800U) != 0U)
    value |= 0xF000U;
  return static_cast<std::int16_t>(value);
}
} // namespace

Bme280Calibration Bme280Compensation::DecodeCalibration(const std::array<std::uint8_t, 26>& primary,
                                                        const std::array<std::uint8_t, 7>& humidity)
{
  Bme280Calibration c;
  c.dig_t1 = ReadU16Le(&primary[0]);
  c.dig_t2 = ReadS16Le(&primary[2]);
  c.dig_t3 = ReadS16Le(&primary[4]);
  c.dig_p1 = ReadU16Le(&primary[6]);
  c.dig_p2 = ReadS16Le(&primary[8]);
  c.dig_p3 = ReadS16Le(&primary[10]);
  c.dig_p4 = ReadS16Le(&primary[12]);
  c.dig_p5 = ReadS16Le(&primary[14]);
  c.dig_p6 = ReadS16Le(&primary[16]);
  c.dig_p7 = ReadS16Le(&primary[18]);
  c.dig_p8 = ReadS16Le(&primary[20]);
  c.dig_p9 = ReadS16Le(&primary[22]);
  c.dig_h1 = primary[25];
  c.dig_h2 = ReadS16Le(&humidity[0]);
  c.dig_h3 = humidity[2];
  c.dig_h4 = SignExtend12(static_cast<std::uint16_t>(humidity[3]) << 4 | (humidity[4] & 0x0FU));
  c.dig_h5 = SignExtend12(static_cast<std::uint16_t>(humidity[5]) << 4 | (humidity[4] >> 4));
  c.dig_h6 = static_cast<std::int8_t>(humidity[6]);
  return c;
}

Bme280RawSample Bme280Compensation::DecodeRawSample(const std::array<std::uint8_t, 8>& data)
{
  Bme280RawSample raw;
  raw.pressure = static_cast<std::uint32_t>(data[0]) << 12 |
                 static_cast<std::uint32_t>(data[1]) << 4 | data[2] >> 4;
  raw.temperature = static_cast<std::uint32_t>(data[3]) << 12 |
                    static_cast<std::uint32_t>(data[4]) << 4 | data[5] >> 4;
  raw.humidity = static_cast<std::uint16_t>(data[6]) << 8 | data[7];
  return raw;
}

Bme280Sample Bme280Compensation::Compensate(const Bme280Calibration& c, const Bme280RawSample& raw)
{
  // Bosch double-precision formula. t_fine is deliberately retained because
  // pressure and humidity calibration are temperature dependent.
  const double temp_var1 =
      (static_cast<double>(raw.temperature) / 16384.0 - c.dig_t1 / 1024.0) * c.dig_t2;
  const double temp_delta = static_cast<double>(raw.temperature) / 131072.0 - c.dig_t1 / 8192.0;
  const double temp_var2 = temp_delta * temp_delta * c.dig_t3;
  const double t_fine = temp_var1 + temp_var2;

  Bme280Sample sample;
  sample.temperature_c = t_fine / 5120.0;

  double pressure_var1 = t_fine / 2.0 - 64000.0;
  double pressure_var2 = pressure_var1 * pressure_var1 * c.dig_p6 / 32768.0;
  pressure_var2 += pressure_var1 * c.dig_p5 * 2.0;
  pressure_var2 = pressure_var2 / 4.0 + c.dig_p4 * 65536.0;
  pressure_var1 =
      (c.dig_p3 * pressure_var1 * pressure_var1 / 524288.0 + c.dig_p2 * pressure_var1) / 524288.0;
  pressure_var1 = (1.0 + pressure_var1 / 32768.0) * c.dig_p1;
  if (std::abs(pressure_var1) > 1.0e-12)
  {
    double pressure = 1048576.0 - raw.pressure;
    pressure = (pressure - pressure_var2 / 4096.0) * 6250.0 / pressure_var1;
    pressure_var1 = c.dig_p9 * pressure * pressure / 2147483648.0;
    pressure_var2 = pressure * c.dig_p8 / 32768.0;
    sample.pressure_pa = pressure + (pressure_var1 + pressure_var2 + c.dig_p7) / 16.0;
  }

  double humidity = t_fine - 76800.0;
  humidity = (raw.humidity - (c.dig_h4 * 64.0 + c.dig_h5 / 16384.0 * humidity)) *
             (c.dig_h2 / 65536.0 *
              (1.0 + c.dig_h6 / 67108864.0 * humidity * (1.0 + c.dig_h3 / 67108864.0 * humidity)));
  humidity *= 1.0 - c.dig_h1 * humidity / 524288.0;
  sample.relative_humidity_percent = humidity;
  return sample;
}
} // namespace OASIS::Environment
