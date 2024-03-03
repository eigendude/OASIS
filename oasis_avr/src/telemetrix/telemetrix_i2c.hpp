/*
 *  Copyright (C) 2022-2024 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  This file is derived from Telemetrix4Arduino under the AGPL 3.0 License
 *  Copyright (c) 2020-2021 Alan Yorinks
 *
 *  SPDX-License-Identifier: Apache-2.0 AND AGPL-3.0
 *  See DOCS/LICENSING.md for more information.
 */

#pragma once

#include <stdint.h>

#define MAX_I2C_PORTS 2

namespace OASIS
{
class I2CPort;

class TelemetrixI2C
{
public:
  void I2CBegin(uint8_t i2cPortIndex);

  void I2CRead(uint8_t i2cPortIndex,
               uint8_t address,
               uint8_t theRegister,
               uint8_t byteCount,
               bool stopTransmitting,
               bool writeByte);
  void I2CWrite(uint8_t i2cPortIndex, uint8_t i2cAddress, uint8_t byteCount, const uint8_t* data);

  /*!
   * \brief Begin communicating with a CCS811 air quality sensor
   *
   * Default I2C address is 0x5B. Alternate I2C address is 0x5A.
   */
  void BeginCCS811(uint8_t i2cPortIndex, uint8_t i2cAddress);

  void EndCCS811(uint8_t i2cPortIndex, uint8_t i2cAddress);

  /*!
   * \brief Begin communicating with an MPU6050 IMU sensor
   *
   * Default I2C address is 0x68 (address pin low). Alternative I2C address is
   * 0x69 (address pin high).
   */
  void BeginMPU6050(uint8_t i2cPortIndex, uint8_t i2cAddress);

  void EndMPU6050(uint8_t i2cPortIndex, uint8_t i2cAddress);

  void ScanSensors();

private:
  I2CPort* m_ports[MAX_I2C_PORTS];
};
} // namespace OASIS
