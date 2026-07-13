/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "Ens160Device.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <thread>

#include <I2Cdev.h>

namespace OASIS::Environment
{
namespace
{
constexpr std::uint8_t kRegPartId = 0x00;
constexpr std::uint8_t kRegOpMode = 0x10;
constexpr std::uint8_t kRegConfig = 0x11;
constexpr std::uint8_t kRegCommand = 0x12;
constexpr std::uint8_t kRegTempIn = 0x13;
constexpr std::uint8_t kRegRhIn = 0x15;
constexpr std::uint8_t kRegDeviceStatus = 0x20;
constexpr std::uint8_t kRegDataAqi = 0x21;
[[maybe_unused]] constexpr std::uint8_t kRegDataTvoc = 0x22;
[[maybe_unused]] constexpr std::uint8_t kRegDataEco2 = 0x24;

constexpr std::uint16_t kPartId = 0x0160;
constexpr std::uint8_t kOpModeIdle = 0x01;
constexpr std::uint8_t kOpModeStandard = 0x02;
constexpr std::uint8_t kCommandNop = 0x00;
constexpr std::uint8_t kCommandClearGpr = 0xCC;
} // namespace

bool Ens160Device::Initialize(const Ens160Config& config)
{
  m_config = config;
  m_lastError.clear();
  I2Cdev::initialize(m_config.i2c_device.c_str());

  std::array<std::uint8_t, 2> part_id{};
  if (!ReadRegisters(kRegPartId, part_id.data(), part_id.size()))
    return false;
  m_partId = DecodePartId(part_id);
  if (m_partId != kPartId)
    return Fail("verify part ID");

  if (!WriteRegister(kRegOpMode, kOpModeIdle))
    return false;
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  if (!WriteRegister(kRegConfig, 0x00))
    return false;
  if (!WriteRegister(kRegCommand, kCommandNop) || !WriteRegister(kRegCommand, kCommandClearGpr))
    return false;
  if (!WriteRegister(kRegOpMode, kOpModeStandard))
    return false;
  std::this_thread::sleep_for(std::chrono::milliseconds(10));

  std::uint8_t mode = 0;
  if (!ReadRegister(kRegOpMode, mode))
    return false;
  if (mode != kOpModeStandard)
    return Fail("confirm standard operating mode");

  std::uint8_t raw_status = 0;
  if (!ReadRegister(kRegDeviceStatus, raw_status))
    return false;
  if (DecodeStatus(raw_status).error != 0)
    return Fail("inspect device error status");
  return true;
}

bool Ens160Device::WriteEnvironmentalCompensation(double temperature_c,
                                                  double relative_humidity_percent)
{
  const std::uint16_t temperature = EncodeTemperature(temperature_c);
  std::uint8_t temperature_data[2] = {static_cast<std::uint8_t>(temperature & 0xFFU),
                                      static_cast<std::uint8_t>(temperature >> 8)};
  if (!WriteRegisters(kRegTempIn, temperature_data, 2))
    return false;

  const std::uint16_t humidity = EncodeHumidity(relative_humidity_percent);
  std::uint8_t humidity_data[2] = {static_cast<std::uint8_t>(humidity & 0xFFU),
                                   static_cast<std::uint8_t>(humidity >> 8)};
  return WriteRegisters(kRegRhIn, humidity_data, 2);
}

bool Ens160Device::ReadStatus(Ens160Status& status)
{
  std::uint8_t raw_status = 0;
  if (!ReadRegister(kRegDeviceStatus, raw_status))
    return false;
  status = DecodeStatus(raw_status);
  return true;
}

bool Ens160Device::ReadSample(Ens160Sample& sample)
{
  std::array<std::uint8_t, 5> data{};
  if (!ReadRegisters(kRegDataAqi, data.data(), data.size()))
    return false;
  sample = DecodeSample(data);
  return true;
}

std::uint16_t Ens160Device::DecodePartId(const std::array<std::uint8_t, 2>& data)
{
  return static_cast<std::uint16_t>(data[0]) |
         static_cast<std::uint16_t>(static_cast<std::uint16_t>(data[1]) << 8);
}

Ens160Status Ens160Device::DecodeStatus(std::uint8_t status)
{
  Ens160Status decoded;
  decoded.validity = static_cast<Ens160Validity>((status >> 2) & 0x03U);
  decoded.new_data = (status & 0x02U) != 0U;
  decoded.error = (status >> 6) & 0x03U;
  return decoded;
}

Ens160Sample Ens160Device::DecodeSample(const std::array<std::uint8_t, 5>& data)
{
  Ens160Sample sample;
  sample.air_quality_index = data[0] & 0x07U;
  sample.tvoc_ppb = static_cast<std::uint16_t>(data[1]) |
                    static_cast<std::uint16_t>(static_cast<std::uint16_t>(data[2]) << 8);
  sample.equivalent_co2_ppm = static_cast<std::uint16_t>(data[3]) |
                              static_cast<std::uint16_t>(static_cast<std::uint16_t>(data[4]) << 8);
  return sample;
}

bool Ens160Device::IsSampleValid(const Ens160Status& status, const Ens160Sample& sample)
{
  return status.validity != Ens160Validity::Invalid && status.error == 0 &&
         sample.air_quality_index >= 1 && sample.air_quality_index <= 5 &&
         sample.equivalent_co2_ppm >= 400 && sample.equivalent_co2_ppm <= 65000 &&
         sample.tvoc_ppb <= 65000;
}

std::uint16_t Ens160Device::EncodeTemperature(double temperature_c)
{
  // TEMP_IN uses unsigned Kelvin in Q9.6 format
  return static_cast<std::uint16_t>(
      std::clamp(std::lround((temperature_c + 273.15) * 64.0), 0L, 65535L));
}

std::uint16_t Ens160Device::EncodeHumidity(double relative_humidity_percent)
{
  // RH_IN uses percent relative humidity in Q7.9 format
  return static_cast<std::uint16_t>(
      std::clamp(std::lround(relative_humidity_percent * 512.0), 0L, 65535L));
}

double Ens160Device::TvocPpbToPpm(std::uint16_t tvoc_ppb)
{
  return static_cast<double>(tvoc_ppb) / 1000.0;
}

bool Ens160Device::WriteRegister(std::uint8_t reg, std::uint8_t value)
{
  if (!I2Cdev::writeByte(m_config.i2c_address, reg, value))
    return Fail("write register");
  return true;
}

bool Ens160Device::WriteRegisters(std::uint8_t reg, std::uint8_t* values, std::size_t length)
{
  if (!I2Cdev::writeBytes(m_config.i2c_address, reg, static_cast<std::uint8_t>(length), values))
    return Fail("write register block");
  return true;
}

bool Ens160Device::ReadRegister(std::uint8_t reg, std::uint8_t& value)
{
  value = 0;
  if (I2Cdev::readBytes(m_config.i2c_address, reg, 1, &value) != 1)
    return Fail("read register");
  return true;
}

bool Ens160Device::ReadRegisters(std::uint8_t reg, std::uint8_t* values, std::size_t length)
{
  const int result =
      I2Cdev::readBytes(m_config.i2c_address, reg, static_cast<std::uint8_t>(length), values);
  if (result != static_cast<int>(length))
    return Fail("read register block");
  return true;
}

bool Ens160Device::Fail(const char* operation)
{
  char error[192]{};
  std::snprintf(error, sizeof(error), "ENS160 on %s address 0x%02x: %s failed",
                m_config.i2c_device.c_str(), m_config.i2c_address, operation);
  m_lastError = error;
  return false;
}
} // namespace OASIS::Environment
