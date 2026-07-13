/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "Bme280Device.h"

#include <array>
#include <chrono>
#include <cstdio>
#include <thread>

#include <I2Cdev.h>

namespace OASIS::Environment
{
namespace
{
constexpr std::uint8_t kRegCalibration = 0x88;
constexpr std::uint8_t kRegChipId = 0xD0;
constexpr std::uint8_t kRegCtrlHumidity = 0xF2;
constexpr std::uint8_t kRegStatus = 0xF3;
constexpr std::uint8_t kRegCtrlMeasurement = 0xF4;
constexpr std::uint8_t kRegConfig = 0xF5;
constexpr std::uint8_t kRegPressure = 0xF7;
constexpr std::uint8_t kRegHumidityCalibration = 0xE1;

constexpr std::uint8_t kChipId = 0x60;
constexpr std::uint8_t kStatusNvmUpdating = 0x01;

// Humidity oversampling x2, encoded as 010
constexpr std::uint8_t kCtrlHumidity = 0x02;

// Temperature x2 (010), pressure x4 (011), normal mode (11)
constexpr std::uint8_t kCtrlMeasurement = 0x4F;

// Standby 62.5 ms (001), IIR filter x4 (010), three-wire SPI disabled
constexpr std::uint8_t kConfig = 0x28;
} // namespace

bool Bme280Device::Initialize(const Bme280Config& config)
{
  m_config = config;
  m_lastError.clear();
  I2Cdev::initialize(m_config.i2c_device.c_str());

  std::uint8_t chip_id = 0;
  if (!ReadRegister(kRegChipId, chip_id))
    return false;
  if (chip_id != kChipId)
    return Fail("verify chip ID");

  bool nvm_ready = false;
  for (unsigned int attempt = 0; attempt < 10; ++attempt)
  {
    std::uint8_t status = 0;
    if (!ReadRegister(kRegStatus, status))
      return false;
    if ((status & kStatusNvmUpdating) == 0U)
    {
      nvm_ready = true;
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  if (!nvm_ready)
    return Fail("wait for calibration NVM copy");

  if (!ReadCalibration())
    return false;

  // ctrl_hum changes become effective only after the subsequent ctrl_meas write
  if (!WriteRegister(kRegCtrlHumidity, kCtrlHumidity))
    return false;
  if (!WriteRegister(kRegConfig, kConfig))
    return false;
  if (!WriteRegister(kRegCtrlMeasurement, kCtrlMeasurement))
    return false;
  return true;
}

bool Bme280Device::ReadSample(Bme280Sample& sample)
{
  std::array<std::uint8_t, 8> data{};
  if (!ReadRegisters(kRegPressure, data.data(), data.size()))
    return false;
  sample = Bme280Compensation::Compensate(m_calibration, Bme280Compensation::DecodeRawSample(data));
  return true;
}

bool Bme280Device::ReadCalibration()
{
  std::array<std::uint8_t, 26> primary{};
  std::array<std::uint8_t, 7> humidity{};
  if (!ReadRegisters(kRegCalibration, primary.data(), primary.size()))
    return false;
  if (!ReadRegisters(kRegHumidityCalibration, humidity.data(), humidity.size()))
    return false;
  m_calibration = Bme280Compensation::DecodeCalibration(primary, humidity);
  if (m_calibration.dig_t1 == 0 || m_calibration.dig_p1 == 0)
    return Fail("validate calibration coefficients");
  return true;
}

bool Bme280Device::WriteRegister(std::uint8_t reg, std::uint8_t value)
{
  if (!I2Cdev::writeByte(m_config.i2c_address, reg, value))
    return Fail("write register");
  return true;
}

bool Bme280Device::ReadRegister(std::uint8_t reg, std::uint8_t& value)
{
  value = 0;
  if (I2Cdev::readBytes(m_config.i2c_address, reg, 1, &value) != 1)
    return Fail("read register");
  return true;
}

bool Bme280Device::ReadRegisters(std::uint8_t reg, std::uint8_t* values, std::size_t length)
{
  const int result =
      I2Cdev::readBytes(m_config.i2c_address, reg, static_cast<std::uint8_t>(length), values);
  if (result != static_cast<int>(length))
    return Fail("read register block");
  return true;
}

bool Bme280Device::Fail(const char* operation)
{
  char error[192]{};
  std::snprintf(error, sizeof(error), "BME280 on %s address 0x%02x: %s failed",
                m_config.i2c_device.c_str(), m_config.i2c_address, operation);
  m_lastError = error;
  return false;
}
} // namespace OASIS::Environment
