/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#pragma once

#include "Bme280Compensation.h"

#include <cstddef>
#include <cstdint>
#include <string>

namespace OASIS::Environment
{
struct Bme280Config
{
  //! Linux I2C device path, such as /dev/i2c-1
  std::string i2c_device;
  //! Supported 7-bit I2C address, 0x76 or 0x77
  std::uint8_t i2c_address{0x77};
};

class Bme280Device
{
public:
  bool Initialize(const Bme280Config& config);
  bool ReadSample(Bme280Sample& sample);
  [[nodiscard]] const std::string& LastError() const { return m_lastError; }

private:
  bool ReadCalibration();
  bool WriteRegister(std::uint8_t reg, std::uint8_t value);
  bool ReadRegister(std::uint8_t reg, std::uint8_t& value);
  bool ReadRegisters(std::uint8_t reg, std::uint8_t* values, std::size_t length);
  bool Fail(const char* operation);

  Bme280Config m_config;
  Bme280Calibration m_calibration;
  std::string m_lastError;
};
} // namespace OASIS::Environment
