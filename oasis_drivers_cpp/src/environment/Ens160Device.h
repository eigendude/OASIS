/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>

namespace OASIS::Environment
{
enum class Ens160Validity : std::uint8_t
{
  //! Fully conditioned output suitable for normal publication
  Normal = 0,
  //! Output during the typical three-minute warm-up period
  Warmup = 1,
  //! Output during first-use conditioning, which may take up to one hour
  InitialStartup = 2,
  //! Output explicitly marked invalid and unsuitable for publication
  Invalid = 3,
};

struct Ens160Config
{
  //! Linux I2C device path, such as /dev/i2c-1
  std::string i2c_device;
  //! Supported 7-bit I2C address, 0x52 or 0x53
  std::uint8_t i2c_address{0x53};
};

struct Ens160Status
{
  //! Output validity state decoded from DEVICE_STATUS bits 3:2
  Ens160Validity validity{Ens160Validity::Invalid};
  //! True when DEVICE_STATUS reports new sensor output
  bool new_data{false};
  //! Device error field decoded from DEVICE_STATUS bits 7:6
  std::uint8_t error{0};
};

struct Ens160Sample
{
  //! ENS160 AQI-UBA index, valid range 1 through 5
  std::uint8_t air_quality_index{0};
  //! Equivalent CO2 calculated by the ENS160, in ppm, range 400..65000
  std::uint16_t equivalent_co2_ppm{0};
  //! Total volatile organic compounds, in ppb, range 0..65000
  std::uint16_t tvoc_ppb{0};
};

class Ens160Device
{
public:
  bool Initialize(const Ens160Config& config);
  bool WriteEnvironmentalCompensation(double temperature_c, double relative_humidity_percent);
  bool ReadStatus(Ens160Status& status);
  bool ReadSample(Ens160Sample& sample);
  [[nodiscard]] std::uint16_t PartId() const { return m_partId; }
  [[nodiscard]] const std::string& LastError() const { return m_lastError; }

  static std::uint16_t DecodePartId(const std::array<std::uint8_t, 2>& data);
  static Ens160Status DecodeStatus(std::uint8_t status);
  static Ens160Sample DecodeSample(const std::array<std::uint8_t, 5>& data);
  static bool IsSampleValid(const Ens160Status& status, const Ens160Sample& sample);
  static std::uint16_t EncodeTemperature(double temperature_c);
  static std::uint16_t EncodeHumidity(double relative_humidity_percent);
  static double TvocPpbToPpm(std::uint16_t tvoc_ppb);

private:
  bool WriteRegister(std::uint8_t reg, std::uint8_t value);
  bool WriteRegisters(std::uint8_t reg, std::uint8_t* values, std::size_t length);
  bool ReadRegister(std::uint8_t reg, std::uint8_t& value);
  bool ReadRegisters(std::uint8_t reg, std::uint8_t* values, std::size_t length);
  bool Fail(const char* operation);

  Ens160Config m_config;
  std::uint16_t m_partId{0};
  std::string m_lastError;
};
} // namespace OASIS::Environment
